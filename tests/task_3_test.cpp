#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>

#include "src/task_3_fire.hpp"
#include "tasks/auto_aim/outpost_tracker.hpp"
#include "tools/math_tools.hpp"

namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr double RADIUS = 0.2765;
const double HEIGHTS[3] = {0.18, 0.30, 0.41};
int checks = 0;

void expect(bool condition, const char * message)
{
  ++checks;
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

std::chrono::steady_clock::time_point stamp(double seconds)
{
  return std::chrono::steady_clock::time_point{} +
         std::chrono::duration_cast<std::chrono::steady_clock::duration>(
           std::chrono::duration<double>(seconds));
}

Eigen::Vector4d truth(double phase, int id)
{
  double angle = tools::limit_rad(phase + id * 2 * PI / 3);
  return {3.0 - RADIUS * std::cos(angle), 0.15 - RADIUS * std::sin(angle), HEIGHTS[id], angle};
}

void test_model_and_jacobian()
{
  auto_aim::Target target(
    truth(0.2, 0), auto_aim::ArmorName::outpost, auto_aim::ArmorType::special, stamp(0), RADIUS, 3);
  expect(target.ekf_x()[7] == 0, "outpost omega starts at zero, not preset to 2.5");
  expect(target.armor_xyza_list().size() == 3, "outpost generates exactly three plates");
  auto x = target.ekf_x();
  x[6] = PI - 0.03;
  x[9] = -0.23;
  x[10] = -0.11;
  for (int id = 0; id < 3; ++id) {
    auto h = target.observation_model(x, id);
    auto xyz = tools::ypd2xyz(h.head<3>());
    double expected_z = x[4] + (id == 1 ? x[9] : id == 2 ? x[10] : 0);
    expect(std::abs(xyz[2] - expected_z) < 1e-10, "three independent plate heights");
    expect(
      std::abs(std::hypot(xyz[0] - x[0], xyz[1] - x[2]) - RADIUS) < 1e-10,
      "all outpost plates share one radius, l never changes radius");
    auto analytic = target.observation_jacobian(x, id);
    for (int column = 0; column < 11; ++column) {
      Eigen::VectorXd plus = x, minus = x;
      plus[column] += 1e-6;
      minus[column] -= 1e-6;
      Eigen::Vector4d diff =
        target.observation_model(plus, id) - target.observation_model(minus, id);
      for (int row : {0, 1, 3}) diff[row] = tools::limit_rad(diff[row]);
      expect(
        (diff / 2e-6 - analytic.col(column)).norm() < 1e-6,
        "analytic observation Jacobian agrees with central finite differences");
    }
  }
  target.predict(0.1);
  auto after = target.ekf_x();
  expect(after[1] == 0 && after[3] == 0 && after[5] == 0, "outpost center velocities stay zero");
  expect(after[0] == x[0] && after[2] == x[2], "predict does not translate fixed center");
}

void test_process_and_wrap()
{
  auto_aim::TargetConfig config;
  config.outpost_center_noise = 2e-5;
  config.outpost_angular_acceleration_variance = 0.7;
  auto_aim::Target target(
    truth(PI - 0.01, 0), auto_aim::ArmorName::outpost, auto_aim::ArmorType::special,
    stamp(0), RADIUS, 3, config);
  const auto prior = target.ekf().P;
  expect(prior(7, 7) == 100 && prior(6, 6) == 0.4, "outpost P0 phase/omega uncertainty");
  expect(prior(8, 8) == config.outpost_radius_variance, "outpost radius prior actually used");
  const double dt = 0.07, q = config.outpost_angular_acceleration_variance;
  target.predict(dt);
  const auto & p = target.ekf().P;
  expect(std::abs(p(6, 6) - prior(6, 6) - dt * dt * prior(7, 7) - std::pow(dt, 4) * q / 4) < 1e-12,
         "phase covariance follows F P F^T + piecewise-white-acceleration Q");
  expect(std::abs(p(6, 7) - dt * prior(7, 7) - std::pow(dt, 3) * q / 2) < 1e-12,
         "phase/omega cross covariance uses dt and dt^3/2");
  expect(std::abs(p(7, 7) - prior(7, 7) - dt * dt * q) < 1e-12,
         "omega process noise uses dt^2, not dt or dt^4");
  for (int axis : {0, 2, 4}) {
    expect(std::abs(p(axis, axis) - prior(axis, axis) - config.outpost_center_noise * dt) < 1e-12,
           "fixed center covariance random walk uses m^2/s times dt");
    expect(p(axis, axis + 1) == 0, "unused center velocity is decoupled");
  }
  const Eigen::Vector4d pose{3, 0, 0, 0};
  const auto r = auto_aim::Target::measurement_covariance(pose);
  expect(std::abs(r(0, 0) - 5e-4) < 1e-12 && std::abs(r(1, 1) - 5e-4) < 1e-12,
         "R bearing variances use radians squared");
  expect(std::abs(r(2, 2) - 0.055) < 1e-12 &&
         std::abs(r(3, 3) - (0.05 + 0.09 + 0.2 / 3)) < 1e-12,
         "R distance and armor yaw match actual adaptive measurement model");
  expect(target.try_update(truth(-PI + 0.01, 0), 0), "wrapped yaw residual across pi accepted");
  expect(target.ekf_x().allFinite() && !target.diverged(), "pi crossing does not diverge");
}

void test_lifecycle()
{
  auto_aim::OutpostTracker tracker;
  Eigen::Vector4d top{2.72, 0.1, 0.85, 0.1};
  expect(!tracker.update({top}, stamp(0)), "top armor alone cannot initialize middle tracker");
  expect(!tracker.target(), "height rejection leaves no target");
  auto good = truth(0.2, 0);
  expect(tracker.update({good, top}, stamp(0.02)), "middle plate initializes despite top clutter");
  expect(tracker.updates() == 1 && tracker.seen_mask() == 1, "initial reference id only");
  expect(!tracker.ready(), "initial target cannot fire");
  expect(!tracker.update({good}, stamp(0.02)), "duplicate image timestamp rejected");
  expect(!tracker.update({good}, stamp(0.01)), "backwards image timestamp rejected");
  expect(tracker.updates() == 1, "repeated image is not fused twice");
  expect(!tracker.update({top}, stamp(0.12)), "only top observation is a missed middle detection");
  expect(tracker.target().has_value(), "short detection loss retains model");
  tracker.expire(stamp(0.5));
  expect(
    !tracker.target() && tracker.seen_mask() == 0, "long loss clears model and geometry history");
  expect(tracker.update({good}, stamp(0.52)), "reacquire after loss");
  expect(tracker.updates() == 1 && !tracker.ready(), "reacquisition warms up again");
  tracker.reset();
  expect(!tracker.target() && tracker.updates() == 0, "mode exit reset clears target");

  for (int frame = 0; frame < 150; ++frame) {
    tracker.update({good}, stamp(1.0 + frame * 0.02));
    expect(!tracker.ready(), "one stationary plate cannot satisfy three-plate spin readiness");
  }
  expect(
    std::abs(tracker.target()->ekf_x()[7]) < 0.01,
    "stationary observations do not invent 2.5rad/s");

  bool threw = false;
  try {
    auto_aim::OutpostTrackConfig bad;
    bad.middle_z_min = bad.middle_z_max;
    auto_aim::OutpostTracker invalid(bad);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  expect(threw, "empty middle height window rejected");
}

void test_fire()
{
  auto_aim::OutpostFireConfig config;
  auto_aim::Task2AimCommand command;
  command.valid = true;
  command.ballistic = true;
  command.yaw = 0.2;
  command.pitch = -0.15;
  command.facing_angle = 0.1;
  for (int frame = 0; frame < 100; ++frame) {
    expect(
      auto_aim::outpost_fire_ready(true, true, true, command, 0.2, -0.15, config),
      "continuous permission stays high on consecutive ready frames, no 5Hz pulse limiter");
  }
  expect(
    !auto_aim::outpost_fire_ready(false, true, true, command, 0.2, -0.15, config),
    "CLI fire disabled");
  expect(
    !auto_aim::outpost_fire_ready(true, false, true, command, 0.2, -0.15, config),
    "lost observation or non-auto mode blocks fire");
  expect(
    !auto_aim::outpost_fire_ready(true, true, false, command, 0.2, -0.15, config),
    "unconverged model blocks fire");
  expect(
    !auto_aim::outpost_fire_ready(true, true, true, command, 0.3, -0.15, config),
    "yaw tracking error blocks fire");
  expect(
    !auto_aim::outpost_fire_ready(true, true, true, command, 0.2, 0, config),
    "pitch tracking error blocks fire");
  auto nan = std::numeric_limits<double>::quiet_NaN();
  expect(
    !auto_aim::outpost_fire_ready(true, true, true, command, nan, -0.15, config),
    "invalid feedback blocks fire");
  command.facing_angle = PI / 3;
  expect(
    !auto_aim::outpost_fire_ready(true, true, true, command, 0.2, -0.15, config),
    "side-on armor blocks fire");
  command.valid = false;
  expect(
    !auto_aim::outpost_fire_ready(true, true, true, command, 0.2, -0.15, config),
    "invalid aim blocks fire");
  command.valid = true;
  command.facing_angle = 0.1;
  command.ballistic = false;
  expect(!auto_aim::outpost_fire_ready(true, true, true, command, 0.2, -0.15, config),
         "non-ballistic follow cannot request fire");
  command.ballistic = true;
  command.facing_angle = nan;
  expect(!auto_aim::outpost_fire_ready(true, true, true, command, 0.2, -0.15, config),
         "NaN facing angle blocks fire");
}

void simulate(double omega, double fps, double initial_phase, int seed)
{
  auto_aim::OutpostTracker tracker;
  auto_aim::Task2AimConfig aim_config;
  aim_config.max_facing_angle = 65 * PI / 180;
  auto_aim::Task2Aimer aimer(aim_config);
  std::mt19937 random(seed);
  std::normal_distribution<double> position_noise(0, 0.008);
  std::normal_distribution<double> angle_noise(0, 0.025);
  std::uniform_real_distribution<double> jitter(0.8, 1.2);
  double time = 0, sum_error = 0, max_error = 0, max_height_error = 0;
  int samples = 0, ready_frames = 0, valid_aims = 0, switches = 0, previous_id = -1;
  int missing = 0;
  for (int frame = 0; time < 12; ++frame) {
    time += jitter(random) / fps;
    std::vector<Eigen::Vector4d> observations;
    if (frame % 29 != 0) {
      for (int id = 0; id < 3; ++id) {
        auto pose = truth(initial_phase + omega * time, id);
        auto facing = std::abs(tools::limit_rad(pose[3] - std::atan2(pose[1], pose[0])));
        if (facing > 75 * PI / 180) continue;
        for (int axis = 0; axis < 3; ++axis) pose[axis] += position_noise(random);
        pose[3] = tools::limit_rad(pose[3] + angle_noise(random));
        observations.push_back(pose);
      }
    }
    // 顶部板和高度合法但空间不匹配的误检，均不应劫持已跟踪的中部目标。
    observations.push_back({2.72, 0.1, 0.85, 0.1});
    if (frame > 10) observations.push_back({8, 4, 0.3, 0.1});
    bool observed = tracker.update(observations, stamp(time));
    if (!observed) ++missing;
    if (!tracker.target()) continue;
    const auto & target = *tracker.target();
    expect(!target.diverged(), "outpost remains finite with valid radius/heights");
    expect(
      (target.ekf().P - target.ekf().P.transpose()).norm() < 1e-7,
      "outpost covariance stays symmetric");
    auto x = target.ekf_x();
    expect(x[1] == 0 && x[3] == 0 && x[5] == 0, "fixed center is not fitted as target translation");
    if (observed && target.last_id != previous_id) ++switches;
    if (observed) previous_id = target.last_id;
    if (time <= 4) continue;
    auto error = std::abs(x[7] - omega);
    sum_error += error;
    max_error = std::max(max_error, error);
    ++samples;
    if (tracker.ready() && observed) ++ready_frames;

    // 模型板号由首次观测决定，将三板高度排序后比较，避免把物理板号硬编码。
    auto armors = target.armor_xyza_list();
    std::vector<double> heights;
    for (const auto & armor : armors) heights.push_back(armor[2]);
    std::sort(heights.begin(), heights.end());
    for (int id = 0; id < 3; ++id)
      max_height_error = std::max(max_height_error, std::abs(heights[id] - HEIGHTS[id]));
    if (!observed) continue;
    auto before_x = x;
    auto before_p = target.ekf().P;
    auto command = aimer.aim(target, 0.025, 23, 8 * PI);
    expect((target.ekf_x() - before_x).norm() == 0, "future aim preserves live EKF state");
    expect((target.ekf().P - before_p).norm() == 0, "future aim preserves live covariance");
    if (!command.valid) continue;
    ++valid_aims;
    auto future = target;
    future.predict(command.lead_time);
    auto selected = future.armor_xyza_list()[command.armor_id];
    expect(
      (selected.head<3>() - command.xyz).norm() < 1e-9,
      "aim uses future three-plate geometry including height");
    expect(
      std::abs(command.lead_time - 0.025 - 0.03 - command.fly_time) < 1e-4,
      "aim includes capture age, firing delay and flight");
    expect(std::abs(command.yaw - 8 * PI) <= PI, "outpost aim handles continuous yaw");
    double elevation = -command.pitch;
    double horizontal = 23 * std::cos(elevation) * command.fly_time;
    double height = 23 * std::sin(elevation) * command.fly_time -
                    0.5 * 9.794 * command.fly_time * command.fly_time;
    expect(
      std::abs(horizontal - std::hypot(command.xyz[0], command.xyz[1])) < 1e-6,
      "ballistic horizontal substitution");
    expect(
      std::abs(height - command.xyz[2]) < 1e-6,
      "ballistic vertical substitution for selected height");
    expect(!aimer.aim(target, 0.5, 23, 0).valid, "stale frame rejects aim");
    expect(!aimer.aim(target, 0.025, 0, 0).valid, "invalid bullet speed rejects aim");
  }
  std::cout << "omega=" << omega << " fps=" << fps << " phase=" << initial_phase
            << " mean_error=" << sum_error / std::max(1, samples) << " max_error=" << max_error
            << " max_height_error=" << max_height_error << " ready=" << ready_frames
            << " aims=" << valid_aims << " switches=" << switches << " missed=" << missing << '\n';
  expect(samples > 100, "simulation exercises sustained tracking");
  expect(switches > 8, "simulation includes repeated 120 degree armor switches");
  expect(max_error < 0.4, "synthetic steady-state omega error within task3 tolerance");
  expect(max_height_error < 0.04, "three unequal heights learned despite noise and switches");
  expect(ready_frames > 100, "estimated model reaches fire readiness after all three plates seen");
  expect(valid_aims > 100, "future aim available across repeated switches");
}
}  // namespace

int main()
{
  std::cout << std::fixed << std::setprecision(5);
  test_model_and_jacobian();
  test_process_and_wrap();
  test_lifecycle();
  test_fire();
  for (auto omega : {2.1, 2.5, 2.9, -2.1, -2.5, -2.9}) simulate(omega, 50, 0.2, 17);
  simulate(2.5, 25, 2.2, 31);
  simulate(-2.5, 25, -2.2, 43);
  std::cout << "PASS: " << checks << " task3 checks\n";
  return 0;
}
