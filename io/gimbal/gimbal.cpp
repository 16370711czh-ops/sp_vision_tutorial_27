#include "gimbal.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");
  auto command_timeout_ms = tools::read_or<int>(yaml, "command_timeout_ms", 0);
  if (command_timeout_ms < 0 || command_timeout_ms > 1000)
    throw std::invalid_argument("Invalid command timeout.");
  command_timeout_ = std::chrono::milliseconds(command_timeout_ms);
  // 显式读取波特率，默认为 921600
  int baudrate = tools::read_or<int>(yaml, "baudrate", 921600);

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(baudrate);
    // 设置硬超时：如果 10ms 没收到数据，read 函数会释放 CPU 挂起，不再空转
    serial::Timeout time_out = serial::Timeout::simpleTimeout(10);
    serial_.setTimeout(time_out);
    serial_.open();
    is_connected_ = true;
    tools::logger()->info("[Gimbal] Serial opened at {} baudrate.", baudrate);
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    throw;
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  // 等待首包，确保链路通畅
  if (!poses_.wait_for_data(std::chrono::milliseconds(1000))) {
    quit_ = true;
    if (thread_.joinable()) thread_.join();
    if (serial_.isOpen()) serial_.close();
    throw std::runtime_error("[Gimbal] No valid feedback within 1s.");
  }
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  try {
    send(false, false, 0, 0, 8);
  } catch (...) {
  }
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (serial_.isOpen()) serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return feedback_valid_ &&
             tools::delta_time(std::chrono::steady_clock::now(), feedback_time_) <= 0.1
           ? mode_
           : GimbalMode::IDLE;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

GimbalFeedback Gimbal::feedback() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {state_, mode_, feedback_time_, feedback_valid_};
}

GimbalCommandStatus Gimbal::last_command()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  return {tx_data_, last_write_ok_};
}

void Gimbal::send(bool control, bool fire, float yaw, float pitch, uint8_t target)
{
  auto sample = feedback();
  auto age = tools::delta_time(std::chrono::steady_clock::now(), sample.timestamp);
  if (
    !sample.valid || age < 0 || age > 0.1 || sample.mode != GimbalMode::AUTO_AIM ||
    !std::isfinite(yaw) || !std::isfinite(pitch)) {
    control = fire = false;
    yaw = pitch = 0;
    target = 8;
  }
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (!is_connected_ || !serial_.isOpen()) {
    last_write_ok_ = false;
    return;
  }
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.target = target;
  tx_data_.yaw = yaw;
  tx_data_.pitch = pitch;
  tx_data_.yaw_vel = tx_data_.yaw_acc = tx_data_.pitch_vel = tx_data_.pitch_acc = 0;
  last_write_ok_ = write_command_locked();
  if (last_write_ok_) {
    command_time_ = std::chrono::steady_clock::now();
    stop_pending_ = false;
  }
}

bool Gimbal::write_command_locked()
{
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));
  try {
    return serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_)) ==
           sizeof(tx_data_);
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Write failed: {}", e.what());
    return false;
  }
}

void Gimbal::expire_command()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (command_timeout_.count() == 0 || (tx_data_.mode == 0 && !stop_pending_) || !serial_.isOpen())
    return;
  if (std::chrono::steady_clock::now() - command_time_ <= command_timeout_) return;
  tx_data_.mode = 0;
  tx_data_.target = 8;
  last_write_ok_ = write_command_locked();
  stop_pending_ = !last_write_ok_;
  if (last_write_ok_) tools::logger()->warn("[Gimbal] Command expired; control disabled.");
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    case GimbalMode::HANGING:
      return "HANGING";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  Eigen::Quaterniond result;
  if (!try_q(t, result, std::chrono::milliseconds(100)))
    throw std::runtime_error("[Gimbal] Pose unavailable at image timestamp.");
  return result;
}

bool Gimbal::try_q(
  std::chrono::steady_clock::time_point t, Eigen::Quaterniond & q,
  std::chrono::milliseconds timeout)
{
  return poses_.sample(t, q, timeout);
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");

  // 使用时间戳作为 Watchdog，而非循环错误计数
  auto last_valid_packet_time = std::chrono::steady_clock::now();
  const auto timeout_threshold = std::chrono::seconds(1);

  while (!quit_) {
    expire_command();
    // 1. 硬件 Watchdog：如果超过 2 秒没收到合法包，且串口确实打开，触发重连
    auto now = std::chrono::steady_clock::now();
    if (now - last_valid_packet_time > timeout_threshold) {
      tools::logger()->warn("[Gimbal] No valid data for 1s, attempting to reconnect...");
      reconnect();
      last_valid_packet_time = std::chrono::steady_clock::now();  // 重置计时器
      continue;
    }

    // 2. 线性同步逻辑：逐字节搜索包头 'S'
    uint8_t head_byte;
    if (!read(&head_byte, 1)) {
      // 串口缓冲区为空，由于设置了 Timeout，这里会自动阻塞 10ms，不会疯狂空转
      continue;
    }

    if (head_byte != 'S') continue;

    // 3. 确认第二个包头 'P'
    if (!read(&head_byte, 1)) continue;
    if (head_byte != 'P') continue;

    // 4. 头部确认，读取剩余载荷 (rx_data_ 已包含 head[2])
    size_t remaining_len = sizeof(GimbalToVision) - 2;
    if (!read(reinterpret_cast<uint8_t *>(&rx_data_) + 2, remaining_len)) {
      continue;
    }

    // 5. CRC 校验
    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }

    // --- 校验通过，更新时间戳并处理数据 ---
    last_valid_packet_time = std::chrono::steady_clock::now();
    auto t = std::chrono::steady_clock::now();

    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    if (
      !q.coeffs().allFinite() || q.norm() < 1e-6 || !std::isfinite(rx_data_.yaw) ||
      !std::isfinite(rx_data_.pitch) || !std::isfinite(rx_data_.yaw_vel) ||
      !std::isfinite(rx_data_.pitch_vel) || !std::isfinite(rx_data_.bullet_speed))
      continue;
    if (!poses_.push(q, t)) continue;

    std::lock_guard<std::mutex> lock(mutex_);

    feedback_valid_ = true;
    feedback_time_ = t;
    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.bullet_count = rx_data_.bullet_count;

    state_.supercap_power_in = rx_data_.supercap_power_in;
    state_.supercap_power_out = rx_data_.supercap_power_out;
    state_.supercap_voltage = rx_data_.supercap_voltage;
    state_.supercap_temputer = rx_data_.supercap_temputer;
    state_.supercap_status = rx_data_.supercap_status;
    state_.supercap_cap_energy = 0.5f * 5.0f * state_.supercap_voltage * state_.supercap_voltage;

    switch (rx_data_.mode) {
      case 0:
        mode_ = GimbalMode::IDLE;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
      case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  is_connected_ = false;
  poses_.clear();
  {
    std::lock_guard<std::mutex> state_lock(mutex_);
    feedback_valid_ = false;
    mode_ = GimbalMode::IDLE;
  }
  std::lock_guard<std::mutex> lock(serial_mutex_);

  try {
    if (serial_.isOpen()) serial_.close();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    serial_.open();

    is_connected_ = true;
    tools::logger()->info("[Gimbal] Reconnected serial successfully.");
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
  }
}

}  // namespace io
