#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <stdexcept>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/task3_pipeline.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? | | 输出参数说明}"
  "{@config-path | | YAML配置路径}"
  "{show | false | 显示识别画面}"
  "{dry-run | false | 只估计与记录，不接管云台}"
  "{fire | true | 默认允许连发电平；--fire=false 禁止发射}";

int run(const std::string & config_path, bool show, bool dry_run, bool enable_fire)
{
  auto settings = auto_aim::load_task3_settings(config_path);
  auto yaml = YAML::LoadFile(config_path);
  auto enemy_color = yaml["enemy_color"].as<std::string>();
  if (enemy_color != "red" && enemy_color != "blue")
    throw std::invalid_argument("enemy_color must be red or blue");
  auto color = enemy_color == "red" ? auto_aim::Color::red : auto_aim::Color::blue;
  auto command_timeout_ms = tools::read_or<int>(yaml, "command_timeout_ms", 0);
  if (command_timeout_ms < 20 || command_timeout_ms > 200)
    throw std::invalid_argument("Task3 requires command_timeout_ms within [20,200]");
  auto_aim::Task3Pipeline pipeline(settings, enable_fire, dry_run);
  if (!settings.middle_height_verified)
    tools::logger()->warn("[Task3] Middle height bounds are unverified; this advisory does not block fire.");
  if (!settings.model_geometry_verified)
    tools::logger()->warn("[Task3] Model geometry is unverified; this advisory does not block fire.");
  tools::Exiter exiter;
  tools::Plotter plotter;
  auto_aim::Solver solver(config_path);
  auto_aim::YOLO yolo(config_path, show);
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point image_time;
  auto start = std::chrono::steady_clock::now();
  int frame_count = 0;
  tools::logger()->info(
    "[Task3] dry_run={}, fire_requested={}, controller must use 12Hz continuous mode",
    dry_run, enable_fire && !dry_run);

  while (!exiter.exit()) {
    auto_aim::Task3Frame frame;
    auto processing_start = std::chrono::steady_clock::now();
    int detected = 0, pnp_valid = 0;
    std::vector<double> candidate_z;
    try {
      frame.image_valid = camera.try_read_for(img, image_time, 100ms) && !img.empty();
      if (frame.image_valid) {
        frame.image_time =
          image_time - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                         std::chrono::duration<double>(settings.camera_latency));
        Eigen::Quaterniond q;
        frame.pose_valid =
          gimbal.try_q(frame.image_time, q, std::chrono::milliseconds(settings.pose_timeout_ms));
        if (frame.pose_valid) {
          solver.set_R_gimbal2world(q);
          auto armors = yolo.detect(img, frame_count++);
          detected = static_cast<int>(armors.size());
          for (auto & armor : armors) {
            if (armor.color != color || armor.name != auto_aim::ArmorName::outpost ||
                armor.type != auto_aim::ArmorType::special)
              continue;
            try {
              solver.solve(armor);
              if (
                !armor.xyz_in_world.allFinite() || !armor.ypr_in_world.allFinite() ||
                !armor.xyz_in_gimbal.allFinite() || armor.xyz_in_gimbal[0] <= 0)
                continue;
              ++pnp_valid;
              candidate_z.push_back(armor.xyz_in_world[2]);
              frame.observations.push_back(
                {armor.xyz_in_world[0], armor.xyz_in_world[1], armor.xyz_in_world[2],
                 armor.ypr_in_world[0]});
              if (show) tools::draw_points(img, armor.points, {0, 255, 0});
            } catch (const std::exception & e) {
              tools::logger()->debug("[Task3] PnP rejected: {}", e.what());
            }
          }
        }
      }
    } catch (const std::exception & e) {
      frame.image_valid = frame.pose_valid = false;
      tools::logger()->warn("[Task3] Vision frame rejected: {}", e.what());
    }
    auto feedback = gimbal.feedback();
    frame.now = std::chrono::steady_clock::now();
    auto feedback_age = tools::delta_time(frame.now, feedback.timestamp);
    frame.feedback_fresh =
      feedback.valid && feedback_age >= 0 && feedback_age <= settings.feedback_timeout;
    frame.auto_mode = feedback.mode == io::GimbalMode::AUTO_AIM;
    frame.yaw = feedback.state.yaw;
    frame.pitch = feedback.state.pitch;
    frame.bullet_speed = feedback.state.bullet_speed;
    auto_aim::Task3Result result;
    try {
      result = pipeline.step(frame);
    } catch (const std::exception & e) {
      tools::logger()->error("[Task3] Tracking frame rejected: {}", e.what());
    }
    if (exiter.exit()) result.control = result.fire = false;
    auto yaw = result.control ? static_cast<float>(result.aim.yaw) : 0.0f;
    auto pitch = result.control ? static_cast<float>(result.aim.pitch) : 0.0f;
    uint8_t target_number = 8;
    const auto & tracker = pipeline.tracker();
    if (result.control && tracker.target())
      target_number = static_cast<uint8_t>(tracker.target()->name);
    gimbal.send(result.control, result.fire, yaw, pitch, target_number);
    auto sent = gimbal.last_command();
    double omega = 0, variance = 0, radius = 0, z0 = 0, dh1 = 0, dh2 = 0;
    if (tracker.target()) {
      auto x = tracker.target()->ekf_x();
      omega = x[7];
      variance = tracker.target()->ekf().P(7, 7);
      radius = x[8];
      z0 = x[4];
      dh1 = x[9];
      dh2 = x[10];
    }
    plotter.plot({
      {"t", tools::delta_time(frame.now, start)},
      {"image_ok", frame.image_valid ? 1 : 0},
      {"pose_ok", frame.pose_valid ? 1 : 0},
      {"feedback_fresh", frame.feedback_fresh ? 1 : 0},
      {"feedback_age", feedback_age},
      {"detected", detected},
      {"pnp_valid", pnp_valid},
      {"observed", result.observed ? 1 : 0},
      {"tracking", tracker.target() ? 1 : 0},
      {"updates", tracker.updates()},
      {"seen_mask", tracker.seen_mask()},
      {"estimate_ready", result.estimate_ready ? 1 : 0},
      {"angular_velocity", omega},
      {"angular_speed", std::abs(omega)},
      {"omega_variance", variance},
      {"radius", radius},
      {"z0", z0},
      {"z1", z0 + dh1},
      {"z2", z0 + dh2},
      {"dh1", dh1},
      {"dh2", dh2},
      {"middle_height_verified", settings.middle_height_verified ? 1 : 0},
      {"model_geometry_verified", settings.model_geometry_verified ? 1 : 0},
      {"nis", tracker.last_nis()},
      {"requested_control", result.control ? 1 : 0},
      {"requested_fire", result.fire ? 1 : 0},
      {"control", sent.command.mode != 0 ? 1 : 0},
      {"fire", sent.command.mode == 2 ? 1 : 0},
      {"send_ok", sent.written ? 1 : 0},
      {"yaw", static_cast<double>(sent.command.yaw)},
      {"pitch", static_cast<double>(sent.command.pitch)},
      {"aim_yaw", result.aim.yaw},
      {"aim_pitch", result.aim.pitch},
      {"gimbal_yaw", frame.yaw},
      {"gimbal_pitch", frame.pitch},
      {"yaw_error", result.aim.valid ? std::remainder(result.aim.yaw - frame.yaw, 2 * CV_PI) : 0.0},
      {"pitch_error", result.aim.valid ? result.aim.pitch - frame.pitch : 0.0},
      {"ballistic_valid", result.aim.ballistic ? 1 : 0},
      {"fly_time", result.aim.fly_time},
      {"lead_time", result.aim.lead_time},
      {"facing_angle", result.aim.facing_angle},
      {"armor_id", result.aim.armor_id},
      {"bullet_count", feedback.state.bullet_count},
      {"bullet_speed", frame.bullet_speed},
      {"frame_age", frame.image_valid ? tools::delta_time(frame.now, frame.image_time) : 0},
      {"processing_time", tools::delta_time(frame.now, processing_start)},
    });
    // Finite numeric fields remain compatible with the course Plotter viewer.
    for (size_t i = 0; i < candidate_z.size(); ++i)
      plotter.plot({{"t", tools::delta_time(frame.now, start)},
                    {"candidate_z_" + std::to_string(i), candidate_z[i]}});
    if (show) {
      if (frame.image_valid) cv::imshow("task_3", img);
      auto key = cv::waitKey(1);
      if (key == 'q' || key == 27) break;
    }
  }
  gimbal.send(false, false, 0, 0, 8);
  if (show) cv::destroyAllWindows();
  return 0;
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto path = cli.get<std::string>("@config-path");
  auto show = cli.get<bool>("show");
  auto dry_run = cli.get<bool>("dry-run");
  auto fire = cli.get<bool>("fire");
  if (cli.has("help") || path.empty()) {
    cli.printMessage();
    return 0;
  }
  if (!cli.check()) {
    cli.printErrors();
    return 1;
  }
  try {
    return run(path, show, dry_run, fire);
  } catch (const std::exception & e) {
    tools::logger()->error("[Task3] Startup/runtime failure: {}", e.what());
    return 1;
  }
}
