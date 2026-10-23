#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>

#include "src/task_2_fire.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_aim/task2_aimer.hpp"
#include "tools/math_tools.hpp"

namespace
{
constexpr double PI = 3.14159265358979323846;
int checks = 0;

void expect(bool condition, const char * message)
{
  ++checks;
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

Eigen::Vector4d measurement(double time, double omega, int armor_num)
{
  Eigen::Vector4d best;
  double best_facing = 10;
  for (int id = 0; id < armor_num; id++) {
    auto angle = tools::limit_rad(0.2 + omega * time + id * 2 * PI / armor_num);
    auto r = (id % 2 == 0 || armor_num == 2) ? 0.23 : 0.27;
    auto z = (id % 2 == 0 || armor_num == 2) ? 0.1 : 0.13;
    Eigen::Vector4d point{3.0 - r * std::cos(angle), 0.15 - r * std::sin(angle), z, angle};
    auto facing = std::abs(tools::limit_rad(angle - std::atan2(point[1], point[0])));
    if (facing < best_facing) {
      best = point;
      best_facing = facing;
    }
  }
  return best;
}

auto_aim::Target simulate(double omega, double fps, int armor_num, int seed)
{
  std::mt19937 random(seed);
  std::normal_distribution<double> position_noise(0, 0.008);
  std::normal_distribution<double> angle_noise(0, 0.025);
  std::uniform_real_distribution<double> jitter(0.8, 1.2);
  auto time_point = std::chrono::steady_clock::time_point{};
  auto_aim::Target target(
    measurement(0, omega, armor_num), auto_aim::ArmorName::three, auto_aim::ArmorType::small,
    time_point, 0.2, armor_num);

  double time = 0, sum_error = 0, max_error = 0;
  int samples = 0, switches = 0, previous_id = 0;
  for (int frame = 1; time < 10; frame++) {
    auto dt = jitter(random) / fps;
    time += dt;
    target.predict(dt);
    // 模拟偶发漏检，不捏造新观测。
    if (frame % 29 != 0) {
      auto pose = measurement(time, omega, armor_num);
      for (int axis = 0; axis < 3; axis++) pose[axis] += position_noise(random);
      pose[3] = tools::limit_rad(pose[3] + angle_noise(random));
      target.update(pose);
      if (target.last_id != previous_id) ++switches;
      previous_id = target.last_id;
    }
    expect(!target.diverged(), "EKF remains finite with valid radii");
    expect((target.ekf().P - target.ekf().P.transpose()).norm() < 1e-7, "symmetric covariance");
    if (time > 3) {
      auto error = std::abs(target.ekf_x()[7] - omega);
      sum_error += error;
      max_error = std::max(max_error, error);
      ++samples;
    }
  }
  auto mean_error = sum_error / samples;
  std::cout << "omega=" << omega << " fps=" << fps << " plates=" << armor_num
            << " mean_error=" << mean_error << " max_error=" << max_error
            << " omega_variance=" << target.ekf().P(7, 7) << " switches=" << switches << '\n';
  expect(switches > 5, "test includes repeated armor switching");
  expect(mean_error < 0.3, "mean omega error below 0.3 rad/s in synthetic data");
  expect(max_error < (std::abs(omega) <= 5 ? 0.6 : 1.0), "synthetic omega error within rubric");
  expect(target.ekf().P(7, 7) < 0.25, "omega uncertainty reaches fire readiness threshold");
  expect(std::abs(target.ekf_x()[8] - 0.23) < 0.08, "radius can learn from nonzero P0");
  return target;
}

void test_aimer(auto_aim::Target & target)
{
  auto before_x = target.ekf_x();
  auto before_p = target.ekf().P;
  auto_aim::Task2Aimer aimer;
  expect(!aimer.aim(auto_aim::Target{}, 0.02, 23, 0).valid, "uninitialized target rejected");
  auto command = aimer.aim(target, 0.025, 23, 8 * PI);
  expect(command.valid, "prediction and ballistic aim valid");
  expect((target.ekf_x() - before_x).norm() == 0, "aim does not mutate live state");
  expect((target.ekf().P - before_p).norm() == 0, "aim does not mutate live covariance");
  expect(std::abs(command.yaw - 8 * PI) <= PI, "nearest equivalent continuous yaw");

  auto future = target;
  future.predict(command.lead_time);
  auto point = future.armor_xyza_list()[command.armor_id].head<3>().eval();
  expect((point - command.xyz).norm() < 1e-9, "point matches selected future armor");
  expect(
    std::abs(command.lead_time - 0.025 - 0.03 - command.fly_time) < 1e-4,
    "prediction includes frame age, delay and converged flight time");

  auto elevation = -command.pitch;
  auto horizontal = 23 * std::cos(elevation) * command.fly_time;
  auto height =
    23 * std::sin(elevation) * command.fly_time - 0.5 * 9.794 * command.fly_time * command.fly_time;
  expect(
    std::abs(horizontal - std::hypot(command.xyz[0], command.xyz[1])) < 1e-6,
    "ballistic horizontal position matches future armor");
  expect(std::abs(height - command.xyz[2]) < 1e-6, "ballistic height matches future armor");
  expect(!aimer.aim(target, 0.5, 23, 0).valid, "stale image rejected");
  expect(!aimer.aim(target, -0.1, 23, 0).valid, "negative frame age rejected");
  expect(!aimer.aim(target, 0.02, 0, 0).valid, "zero speed does not use invented fallback");
  expect(!aimer.aim(target, 0.02, 100, 0).valid, "invalid bullet speed rejected");
  expect(
    !aimer.aim(target, 0.02, std::numeric_limits<double>::quiet_NaN(), 0).valid,
    "NaN bullet speed rejected");
}

void test_fire()
{
  auto_aim::SingleShot shooter;
  double last = -1;
  bool previous = false;
  for (int frame = 0; frame < 2000; frame++) {
    auto time = frame / 100.0;
    bool ready = !(time >= 3 && time < 6);
    auto fire = shooter.request(ready, time);
    expect(!(fire && previous), "fire pulses have false frames between them");
    expect(!fire || ready, "no request while not ready");
    if (fire) {
      expect(last < 0 || time - last >= 0.2 - 1e-9, "at most 5Hz");
      last = time;
    }
    previous = fire;
  }
  expect(shooter.shots() > 60, "unlimited mode survives pauses without resetting rate");

  auto_aim::SingleShot limited(10);
  for (int frame = 0; frame < 2000; frame++) limited.request(true, frame / 100.0);
  expect(limited.shots() == 10, "optional process-wide limit is respected");
  expect(!limited.request(true, 100), "budget does not reset on target loss");
  auto_aim::SingleShot invalid;
  expect(!invalid.request(true, -1), "negative clock rejected");
  expect(!invalid.request(true, std::numeric_limits<double>::quiet_NaN()), "NaN clock rejected");
  expect(invalid.shots() == 0, "invalid clock consumes no requests");
}

void test_speed_change()
{
  auto_aim::Target target(
    measurement(0, 4, 4), auto_aim::ArmorName::three, auto_aim::ArmorType::small,
    std::chrono::steady_clock::time_point{}, 0.2, 4);
  double max_error = 0;
  for (int frame = 1; frame <= 600; ++frame) {
    double time = frame * 0.02;
    double omega = time <= 5 ? 4 : 7;
    // 保持相位连续：5秒时由4rad/s切换为7rad/s。
    double phase = time <= 5 ? 4 * time : 20 + 7 * (time - 5);
    target.predict(0.02);
    target.update(measurement(phase / omega, omega, 4));
    expect(!target.diverged(), "speed change does not require reinitialization");
    if (time > 8) max_error = std::max(max_error, std::abs(target.ekf_x()[7] - 7));
  }
  std::cout << "speed_change=4_to_7 settled_max_error=" << max_error << '\n';
  expect(max_error < 1, "medium speed estimate recovers after speed transition");
}
}  // namespace

int main()
{
  std::cout << std::fixed << std::setprecision(4);
  test_fire();
  test_speed_change();
  for (auto omega : {3.0, 4.8, 5.0, 6.0, 9.0, -4.0, -8.0}) {
    auto target = simulate(omega, 50, 4, 17);
    test_aimer(target);
  }
  auto slow_frames = simulate(9, 25, 4, 23);
  test_aimer(slow_frames);
  auto two_plates = simulate(4, 50, 2, 31);
  test_aimer(two_plates);

  bool threw = false;
  try {
    two_plates.predict(-0.01);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  expect(threw, "backwards prediction rejected");
  std::cout << "PASS: " << checks << " task2 checks\n";
  return 0;
}
