#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "tasks/auto_aim/task3_pipeline.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace
{
int checks = 0;
void expect(bool ok, const char * message)
{
  ++checks;
  if (!ok) throw std::runtime_error(message);
}
auto stamp(double seconds)
{
  return std::chrono::steady_clock::time_point{} +
         std::chrono::duration_cast<std::chrono::steady_clock::duration>(
           std::chrono::duration<double>(seconds));
}

std::vector<Eigen::Vector4d> plates(double phase)
{
  std::vector<Eigen::Vector4d> observations;
  // Hypothetical test fixture, NOT confirmed drawing center heights. Assume a 0.95m
  // gimbal datum for this calculation only. The real center/datum heights are unmeasured.
  const double ground_heights[3] = {1.216, 1.316, 1.116};
  for (int id = 0; id < 3; ++id) {
    const double angle = std::remainder(phase + id * 2 * CV_PI / 3, 2 * CV_PI);
    Eigen::Vector4d p{
      3 - 0.2765 * std::cos(angle), -0.2765 * std::sin(angle), ground_heights[id] - 0.95, angle};
    if (std::abs(std::remainder(angle - std::atan2(p[1], p[0]), 2 * CV_PI)) < CV_PI / 3)
      observations.push_back(p);
  }
  return observations;
}

void pipeline_test(
  auto_aim::Task3Settings settings, bool enable, bool dry_run, bool verified, bool geometry_verified = true)
{
  settings.aiming.yaw_offset = settings.aiming.pitch_offset = 0;
  settings.middle_height_verified = verified;
  settings.model_geometry_verified = geometry_verified;
  auto_aim::Task3Pipeline pipeline(settings, enable, dry_run);
  auto_aim::Task3Frame frame;
  frame.image_valid = frame.pose_valid = frame.auto_mode = frame.feedback_fresh = true;
  frame.bullet_speed = 23;
  int observed = 0, ready = 0, controls = 0, fires = 0, consecutive_fires = 0;
  bool previous_fire = false;
  for (int i = 0; i < 650; ++i) {
    const double time = 1 + i * 0.02;
    frame.image_time = stamp(time);
    frame.now = stamp(time + 0.025);
    frame.observations = plates(-2.5 * time);
    const auto result = pipeline.step(frame);
    observed += result.observed;
    ready += result.estimate_ready;
    controls += result.control;
    fires += result.fire;
    consecutive_fires += previous_fire && result.fire;
    previous_fire = result.fire;
    if (result.aim.valid) {
      frame.yaw = result.aim.yaw;
      frame.pitch = result.aim.pitch;
    }
    if (result.fire)
      expect(result.aim.ballistic && result.control && result.observed && result.estimate_ready,
             "fire requires valid ballistic aim, observation, control and ready model");
    if (i > 230 && pipeline.tracker().target())
      expect(std::abs(pipeline.tracker().target()->ekf_x()[7] + 2.5) < 0.4,
             "hypothetical height fixture omega accuracy");
  }
  expect(observed > 500 && ready > 300, "pipeline acquires all three simulated-height plates");
  expect((controls > 300) == !dry_run, "dry-run suppresses all control while retaining estimation");
  if (enable && !dry_run && verified && geometry_verified)
    expect(fires > 100 && consecutive_fires > 50, "continuous fire permission has no 5Hz pulse limiter");
  else
    expect(fires == 0, "disabled fire, dry-run, unverified heights/geometry block every fire request");

  // Each invalid input must produce an immediate stop, not a stale last command.
  auto check_stop = [&](const auto_aim::Task3Frame & invalid) {
    const auto result = pipeline.step(invalid);
    expect(!result.control && !result.fire && !result.observed, "invalid frame must stop control/fire");
  };
  frame.now = stamp(14.1);
  frame.image_time = stamp(14.08);
  frame.observations = plates(-2.5 * 14.08);
  auto bad = frame;
  bad.pose_valid = false;
  check_stop(bad);
  bad = frame;
  bad.image_valid = false;
  check_stop(bad);
  bad = frame;
  bad.image_time = stamp(15);
  check_stop(bad);
  bad = frame;
  bad.image_time = stamp(13);
  check_stop(bad);
  bad = frame;
  bad.yaw = std::numeric_limits<double>::quiet_NaN();
  check_stop(bad);
  bad = frame;
  bad.feedback_fresh = false;
  check_stop(bad);
  expect(!pipeline.tracker().target(), "stale feedback clears old model");
  pipeline.step(frame);
  bad = frame;
  bad.auto_mode = false;
  check_stop(bad);
  expect(!pipeline.tracker().target(), "mode exit clears old model");
}

void rejection_test()
{
  auto_aim::OutpostTrackConfig config;
  config.filter.max_nis = 0.001;
  auto_aim::OutpostTracker tracker(config);
  const Eigen::Vector4d good{2.7235, 0, 0.3, 0};
  expect(tracker.update({good}, stamp(0)), "initialize gated tracker");
  Eigen::Vector4d outlier = good;
  outlier[1] += 0.3;  // passes Euclidean/angle gates but violates statistical innovation gate
  expect(!tracker.update({outlier}, stamp(0.02)), "NIS rejects geometrically nearby wrong armor");
  expect(tracker.updates() == 1 && tracker.seen_mask() == 1,
         "rejected measurement does not increase accepted updates or plate mask");
  expect(tracker.update({good, outlier}, stamp(0.04)), "good candidate wins association over outlier");
  const int count = tracker.updates();
  Eigen::Vector4d nan = good;
  nan[3] = std::numeric_limits<double>::quiet_NaN();
  expect(!tracker.update({nan, {200, 0, 0.3, 0}}, stamp(0.06)), "NaN and huge range rejected");
  expect(tracker.updates() == count, "invalid inputs not counted");
  tracker.expire(stamp(0.6));
  expect(!tracker.target(), "long loss expires target");
  expect(tracker.update({good}, stamp(0.62)) && !tracker.ready(), "reacquisition warms up again");

  auto_aim::Task3Settings settings;
  settings.aiming.yaw_offset = settings.aiming.pitch_offset = 0;
  auto_aim::Task3Pipeline pipeline(settings, true);
  auto_aim::Task3Frame frame;
  frame.image_valid = frame.pose_valid = frame.auto_mode = frame.feedback_fresh = true;
  frame.bullet_speed = 0;
  for (int i = 0; i < 5; ++i) {
    frame.image_time = stamp(1 + i * 0.02);
    frame.now = stamp(1.01 + i * 0.02);
    frame.observations = {good};
    const auto result = pipeline.step(frame);
    if (i >= 2) expect(result.control && !result.aim.ballistic && !result.fire,
                       "invalid bullet speed allows configured non-ballistic follow only");
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) return 2;
  const auto settings = auto_aim::load_task3_settings(argv[1]);
  expect(!settings.middle_height_verified, "distributed YAML blocks fire until height calibration");
  expect(!settings.model_geometry_verified, "distributed YAML keeps physical geometry unverified");
  expect(std::abs(settings.aiming.pitch_limit - CV_PI / 9) < 1e-12, "YAML degrees converted to rad");
  expect(settings.tracking.filter.max_nis == 25, "Task3 NIS parameter actually loaded");
  pipeline_test(settings, false, false, true);
  pipeline_test(settings, true, true, true);
  pipeline_test(settings, true, false, false);
  pipeline_test(settings, true, false, true);
  pipeline_test(settings, true, false, true, false);
  rejection_test();
  auto invalid = settings;
  invalid.tracking.filter.max_nis = std::numeric_limits<double>::quiet_NaN();
  bool rejected = false;
  try { auto_aim::validate_task3_settings(invalid); }
  catch (const std::invalid_argument &) { rejected = true; }
  expect(rejected, "invalid NIS config rejected before hardware construction");
  auto yaml = YAML::LoadFile(argv[1]);
  yaml["task3"]["pose_timeout_ms"] = "wrong-type";
  const char * path = "task3_invalid_config.yaml";
  { std::ofstream stream(path); stream << yaml; }
  rejected = false;
  try { auto_aim::load_task3_settings(path); }
  catch (const YAML::Exception &) { rejected = true; }
  std::remove(path);
  expect(rejected, "invalid YAML types rejected before hardware construction");
  std::cout << "PASS task3 acceptance: " << checks << " checks\n";
}
