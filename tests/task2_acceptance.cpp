#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <thread>

#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/task2_pipeline.hpp"
#include "tools/crc.hpp"
#include "tools/math_tools.hpp"
#include "tools/quaternion_history.hpp"

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
template <typename F>
void rejects(F function, const char * message)
{
  bool threw = false;
  try {
    function();
  } catch (const std::exception &) {
    threw = true;
  }
  expect(threw, message);
}
auto stamp(double seconds)
{
  return std::chrono::steady_clock::time_point{} +
         std::chrono::duration_cast<std::chrono::steady_clock::duration>(
           std::chrono::duration<double>(seconds));
}
auto_aim::Task2Observation truth(double phase, int id, int plates = 4)
{
  auto angle = std::remainder(0.2 + phase + id * 2 * PI / plates, 2 * PI);
  auto radius = plates == 4 && id % 2 ? 0.27 : 0.23;
  auto height = plates == 4 && id % 2 ? 0.13 : 0.1;
  return {
    {3 - radius * std::cos(angle), 0.15 - radius * std::sin(angle), height, angle},
    auto_aim::ArmorName::three,
    auto_aim::ArmorType::small,
    0};
}
std::vector<auto_aim::Task2Observation> visible(double phase, int plates = 4)
{
  std::vector<auto_aim::Task2Observation> result;
  for (int id = 0; id < plates; ++id) {
    auto o = truth(phase, id, plates);
    if (
      std::abs(std::remainder(o.xyza[3] - std::atan2(o.xyza[1], o.xyza[0]), 2 * PI)) <
      75 * PI / 180)
      result.push_back(o);
  }
  return result;
}

void model()
{
  for (int plates : {2, 4}) {
    auto_aim::Target target(
      truth(0, 0, plates).xyza, auto_aim::ArmorName::three, auto_aim::ArmorType::small, stamp(0),
      0.2, plates);
    expect(target.ekf_x().size() == 11, "11 state dimensions");
    expect(
      target.ekf().P.diagonal().tail<3>().minCoeff() > 0, "geometry covariance permits learning");
    for (auto angle : {PI - 1e-8, -PI + 1e-8, 0.3}) {
      auto x = target.ekf_x();
      x[6] = angle;
      x[8] = 0.23;
      x[9] = 0.04;
      x[10] = 0.03;
      for (int id = 0; id < plates; ++id) {
        auto H = target.observation_jacobian(x, id);
        for (int column = 0; column < 11; ++column) {
          Eigen::VectorXd plus = x, minus = x;
          plus[column] += 1e-6;
          minus[column] -= 1e-6;
          Eigen::Vector4d difference =
            target.observation_model(plus, id) - target.observation_model(minus, id);
          for (int row : {0, 1, 3}) difference[row] = std::remainder(difference[row], 2 * PI);
          expect(
            (difference / 2e-6 - H.col(column)).norm() < 2e-6,
            "full actual H matches finite differences across angle wrap");
        }
      }
    }
    auto before_x = target.ekf_x();
    auto before_p = target.ekf().P;
    auto bad = truth(0, 0).xyza;
    bad[0] = std::numeric_limits<double>::quiet_NaN();
    expect(!target.try_update(bad, 0), "NaN observation rejected");
    expect(!target.try_update(Eigen::Vector4d(100, 100, 20, 2), 0), "large innovation rejected");
    expect(
      (target.ekf_x() - before_x).norm() == 0 && (target.ekf().P - before_p).norm() == 0,
      "rejected measurement preserves x/P exactly");
    rejects([&] { target.predict(-0.01); }, "negative dt rejected");
    rejects([&] { target.predict(std::numeric_limits<double>::quiet_NaN()); }, "NaN dt rejected");
    rejects([&] { target.predict(100); }, "unbounded dt rejected");
    expect((target.ekf_x() - before_x).norm() == 0, "invalid prediction preserves state");
    auto_aim::Task2Aimer aimer;
    auto follow = aimer.follow(target, 0.02, 8 * PI);
    expect(follow.valid && !follow.ballistic, "non-ballistic following explicitly marked");
    expect(!aimer.aim(target, 0.02, 0, 0).valid, "zero speed cannot create ballistic command");
  }
  tools::ExtendedKalmanFilter singular(Eigen::VectorXd::Zero(4), Eigen::MatrixXd::Zero(4, 4));
  Eigen::Vector4d z = Eigen::Vector4d::Zero();
  auto identity = [](const Eigen::VectorXd & x) { return x; };
  auto subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    return a - b;
  };
  expect(
    !singular.try_update(
      z, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Zero(), identity, subtract),
    "singular innovation rejected without inverse");
  expect(
    singular.x.allFinite() && singular.P.allFinite(), "singular innovation does not corrupt state");
  tools::ExtendedKalmanFilter noise_test(Eigen::VectorXd::Zero(2), Eigen::Matrix2d::Identity());
  Eigen::Vector2d gain{0.0002, 0.02};
  Eigen::Matrix2d semidefinite = gain * gain.transpose();
  noise_test.predict(Eigen::Matrix2d::Identity(), semidefinite);
  expect(noise_test.P.allFinite(), "rank-one positive-semidefinite process covariance accepted");
  Eigen::Matrix2d invalid_noise = Eigen::Matrix2d::Zero();
  invalid_noise(0, 0) = -1;
  auto before_noise = noise_test.P;
  rejects(
    [&] { noise_test.predict(Eigen::Matrix2d::Identity(), invalid_noise); },
    "negative process covariance rejected");
  expect(
    (noise_test.P - before_noise).norm() == 0, "rejected process covariance does not mutate P");
  auto chi = 9.487729036781154;
  expect(
    std::abs(1 - std::exp(-chi / 2) * (1 + chi / 2) - 0.95) < 1e-12,
    "four-dimensional chi-square diagnostic threshold");
}

void simulate(double omega, int fps, int plates, bool changing)
{
  auto_aim::Task2TrackConfig config;
  config.armor_num = plates;
  auto_aim::Task2Tracker tracker(config);
  std::mt19937 random(29);
  std::normal_distribution<double> pos(0, 0.008), yaw(0, 0.025);
  std::uniform_real_distribution<double> jitter(0.8, 1.2);
  double time = 0, phase = 0, maximum = 0, sum = 0;
  int samples = 0, switches = 0, previous_id = -1, observed = 0;
  for (int frame = 0; time < 12; ++frame) {
    auto dt = jitter(random) / fps;
    time += dt;
    double actual = changing && time > 5 ? 7 : omega;
    phase += actual * dt;
    auto observations = visible(phase, plates);
    for (auto & o : observations) {
      for (int axis = 0; axis < 3; ++axis) o.xyza[axis] += pos(random);
      o.xyza[3] = std::remainder(o.xyza[3] + yaw(random), 2 * PI);
    }
    if (frame % 29 == 0) observations.clear();
    if (time > 1)
      observations.push_back(
        {{8, 4, 0.1, 0.1}, auto_aim::ArmorName::three, auto_aim::ArmorType::small, 100});
    bool updated = tracker.update(observations, stamp(time));
    if (!tracker.target()) continue;
    expect(!tracker.target()->diverged(), "tracked EKF finite");
    expect(
      (tracker.target()->ekf().P - tracker.target()->ekf().P.transpose()).norm() < 1e-8,
      "Joseph covariance symmetric");
    expect(
      tracker.target()->ekf().P.ldlt().isPositive(), "Joseph covariance positive semidefinite");
    if (updated) {
      ++observed;
      if (previous_id != tracker.target()->last_id) ++switches;
      previous_id = tracker.target()->last_id;
    }
    if (time > (changing ? 8 : 3)) {
      auto error = std::abs(tracker.target()->ekf_x()[7] - actual);
      maximum = std::max(maximum, error);
      sum += error;
      ++samples;
      expect(tracker.track_id() == 1, "no frequent resets or distractor target switches");
    }
  }
  std::cout << "omega=" << omega << " fps=" << fps << " plates=" << plates
            << " changing=" << changing << " mean_error=" << sum / std::max(1, samples)
            << " max_error=" << maximum << " switches=" << switches << " observations=" << observed
            << " tracks=" << tracker.track_id() << '\n';
  expect(samples > 50, "steady-state samples available");
  expect(
    maximum < (std::abs(omega) <= 5 && !changing ? 0.6 : 1.0),
    "true synthetic omega error meets rubric");
  if (omega != 0)
    expect(switches > 5, "repeated armor switches exercised");
  else
    expect(!tracker.ready(), "stationary target does not become spin-fire ready");
}

void tracking()
{
  for (auto omega : {0.0, 3.0, 4.8, 5.0, 6.0, 9.0, -4.0, -8.0}) simulate(omega, 50, 4, false);
  simulate(9, 25, 4, false);
  simulate(4, 50, 2, false);
  simulate(4, 50, 4, true);
  auto_aim::Task2TrackConfig config;
  config.confirm_frames = 1;
  auto_aim::Task2Tracker tracker(config);
  auto o = truth(0, 0);
  expect(tracker.update({o}, stamp(0)), "first target established");
  auto other = o;
  other.name = auto_aim::ArmorName::four;
  expect(
    !tracker.update({other}, stamp(0.02)), "different identity does not hijack current target");
  auto x = tracker.target()->ekf_x();
  expect(!tracker.update({o}, stamp(0.02)), "duplicate timestamp ignored");
  expect(!tracker.update({o}, stamp(0.01)), "backwards timestamp ignored");
  expect((tracker.target()->ekf_x() - x).norm() == 0, "invalid timestamp does not mutate state");
  expect(!tracker.update({}, stamp(0.12)) && tracker.target(), "short loss retains model");
  tracker.expire(stamp(0.5));
  expect(!tracker.target(), "long loss clears model");
  expect(tracker.update({other}, stamp(0.52)), "new identity can acquire after old target is lost");
  expect(
    tracker.target()->name == other.name && tracker.track_id() == 2 && !tracker.ready(),
    "reinitialization resets readiness");
}

void moving_vehicle()
{
  auto_aim::Task2Tracker moving;
  for (int frame = 0; frame < 600; ++frame) {
    double time = frame * 0.02;
    auto observations = visible(4 * time);
    for (auto & measurement : observations) {
      measurement.xyza[0] += 0.08 * time;
      measurement.xyza[1] -= 0.03 * time;
      measurement.xyza[2] += 0.01 * time;
    }
    moving.update(observations, stamp(time));
    if (time > 8) {
      expect(moving.target().has_value(), "translating and rotating vehicle remains tracked");
      auto state = moving.target()->ekf_x();
      expect(std::abs(state[7] - 4) < 0.1, "translation does not bias omega");
      expect(
        std::abs(state[1] - 0.08) < 0.03 && std::abs(state[3] + 0.03) < 0.03 &&
          std::abs(state[5] - 0.01) < 0.03,
        "three center velocity states follow known translation");
      expect(
        std::hypot(state[0] - 3 - 0.08 * time, state[2] - 0.15 + 0.03 * time) < 0.1,
        "horizontal rotation center follows translated ground truth");
      expect(
        std::abs(state[8] - 0.23) < 0.04 && std::abs(state[8] + state[9] - 0.27) < 0.04,
        "both alternating radii remain observable");
      expect(std::abs(state[10] - 0.03) < 0.02, "alternating height offset remains observable");
    }
  }
}

void pipeline()
{
  auto_aim::Task2Settings settings;
  auto_aim::Task2Pipeline following(settings), dry(settings, true, true), firing(settings, true);
  double yaw = 0, pitch = 0, last_fire = -1;
  bool previous_fire = false;
  int controls = 0, fires = 0;
  for (int frame = 0; frame < 1000; ++frame) {
    double time = frame * 0.02;
    auto_aim::Task2Frame f;
    f.image_time = stamp(time);
    f.now = stamp(time + 0.025);
    f.observations = visible(4 * time);
    f.image_valid = f.pose_valid = f.auto_mode = f.feedback_fresh = true;
    f.yaw = yaw;
    f.pitch = pitch;
    f.bullet_speed = 23;
    auto no_fire = following.step(f);
    auto no_control = dry.step(f);
    auto allowed = firing.step(f);
    expect(!no_fire.fire, "default pipeline never emits fire");
    expect(
      !no_control.control && !no_control.fire,
      "dry-run never controls or fires even when fire flag is true");
    if (no_fire.control) ++controls;
    expect(!(allowed.fire && previous_fire), "false output frame separates single shots");
    if (allowed.fire) {
      expect(
        last_fire < 0 || time - last_fire >= 0.2 - 1e-9, "single-shot request rate at most 5Hz");
      expect(
        allowed.estimate_ready && allowed.aim.ballistic,
        "fire requires estimated model and ballistic command");
      ++fires;
      last_fire = time;
    }
    previous_fire = allowed.fire;
    if (no_fire.aim.valid) {
      yaw = no_fire.aim.yaw;
      pitch = no_fire.aim.pitch;
    }
    if (frame == 900) {
      auto invalid = f;
      invalid.image_time = stamp(time + 0.001);
      invalid.now = stamp(time + 0.026);
      invalid.pose_valid = false;
      auto r = firing.step(invalid);
      expect(!r.control && !r.fire, "missing synchronized pose disables output");
    }
  }
  expect(controls > 200 && fires > 10, "synthetic complete control and shot gate exercised");
  auto_aim::Task2Frame f;
  f.image_time = stamp(20);
  f.now = stamp(20.025);
  f.observations = visible(80);
  f.image_valid = f.pose_valid = f.auto_mode = f.feedback_fresh = true;
  f.yaw = yaw;
  f.pitch = pitch;
  f.bullet_speed = 0;
  auto r = following.step(f);
  expect(
    r.control && !r.aim.ballistic && !r.fire,
    "zero-speed feedback permits explicit non-firing following");
  f.pose_valid = false;
  r = following.step(f);
  expect(!r.control && !r.fire, "pose failure disables control");
  f.pose_valid = true;
  f.now = stamp(21);
  r = following.step(f);
  expect(!r.control && !r.fire, "stale image disables control");
  f.feedback_fresh = false;
  r = following.step(f);
  expect(!r.control && !following.tracker().target(), "stale feedback resets and disables");
  f.feedback_fresh = true;
  f.auto_mode = false;
  r = following.step(f);
  expect(!r.control && !r.fire, "mode exit disables output");
  std::cout << "control_frames=" << controls << " simulated_shot_requests=" << fires << '\n';
}

void timing()
{
  static_assert(sizeof(io::VisionToGimbal) == 30, "TX wire layout changed");
  static_assert(sizeof(io::GimbalToVision) == 57, "RX wire layout changed");
  io::VisionToGimbal packet{};
  packet.mode = 1;
  packet.yaw = 0.3f;
  packet.pitch = -0.1f;
  packet.target = 2;
  packet.crc16 = tools::get_crc16(reinterpret_cast<uint8_t *>(&packet), sizeof(packet) - 2);
  expect(
    tools::check_crc16(reinterpret_cast<uint8_t *>(&packet), sizeof(packet)),
    "actual TX packet CRC validates");
  packet.mode = 2;
  expect(
    !tools::check_crc16(reinterpret_cast<uint8_t *>(&packet), sizeof(packet)),
    "packet corruption is detected");
  tools::QuaternionHistory poses;
  Eigen::Quaterniond q;
  expect(
    !poses.sample(stamp(0), q, std::chrono::milliseconds(0)),
    "empty history fails without blocking");
  auto a = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond b(Eigen::AngleAxisd(PI / 2, Eigen::Vector3d::UnitZ()));
  expect(
    poses.push(a, stamp(1)) && poses.push(b, stamp(1.02)), "ordered quaternion samples inserted");
  expect(!poses.push(b, stamp(1.02)), "duplicate IMU timestamp rejected");
  expect(
    poses.sample(stamp(1.01), q, std::chrono::milliseconds(0)), "bracketed interpolation succeeds");
  Eigen::Quaterniond expected(Eigen::AngleAxisd(PI / 4, Eigen::Vector3d::UnitZ()));
  expect(q.angularDistance(expected) < 1e-10, "slerp fraction and direction correct");
  expect(
    !poses.sample(stamp(0.9), q, std::chrono::milliseconds(0)), "past extrapolation forbidden");
  auto start = std::chrono::steady_clock::now();
  expect(
    !poses.sample(stamp(1.5), q, std::chrono::milliseconds(20)),
    "future sample timeout returns failure");
  expect(
    std::chrono::steady_clock::now() - start < std::chrono::seconds(1), "pose timeout is bounded");
  poses.push(a, stamp(1.2));
  expect(
    !poses.sample(stamp(1.1), q, std::chrono::milliseconds(0)),
    "large IMU gap cannot be interpolated");
  Eigen::Quaterniond invalid(0, 0, 0, 0);
  expect(!poses.push(invalid, stamp(2)), "zero quaternion rejected");
  invalid.coeffs()[0] = std::numeric_limits<double>::quiet_NaN();
  expect(!poses.push(invalid, stamp(2)), "NaN quaternion rejected");
  poses.clear();
  expect(
    !poses.sample(stamp(1.01), q, std::chrono::milliseconds(0)), "reconnect clears stale poses");
}

void config(const std::string & path)
{
  auto settings = auto_aim::load_task2_settings(path);
  expect(
    std::abs(settings.yaw_tolerance - 0.8 * PI / 180) < 1e-12, "YAML degrees converted to radians");
  expect(
    settings.pose_timeout_ms == 20 && settings.camera_latency == 0, "YAML time units preserved");
  expect(
    settings.tracking.armor_num == 4 && settings.tracking.filter.max_nis == 18.5,
    "YAML tracker/EKF fields loaded");
  auto bad = settings;
  bad.tracking.filter.radius_variance = 0;
  rejects([&] { auto_aim::validate_task2_settings(bad); }, "zero geometry covariance rejected");
  bad = settings;
  bad.aiming.fire_delay = std::numeric_limits<double>::quiet_NaN();
  rejects([&] { auto_aim::validate_task2_settings(bad); }, "NaN delay rejected");
  bad = settings;
  bad.tracking.armor_num = 3;
  rejects([&] { auto_aim::validate_task2_settings(bad); }, "Task2 rejects unsupported plate count");
  const char * temporary = "task2_invalid_config_test.yaml";
  {
    std::ofstream stream(temporary);
    stream << "task2:\n  armor_num: wrong_type\n";
  }
  rejects([&] { auto_aim::load_task2_settings(temporary); }, "invalid YAML scalar type rejected");
  std::remove(temporary);
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) return 2;
  std::cout << std::fixed << std::setprecision(5);
  std::string suite = argv[1];
  if (suite == "model")
    model();
  else if (suite == "tracking") {
    tracking();
    moving_vehicle();
  } else if (suite == "pipeline")
    pipeline();
  else if (suite == "timing")
    timing();
  else if (suite == "config")
    config(argv[2]);
  else
    return 2;
  std::cout << "PASS " << suite << ": " << checks << " checks\n";
}
