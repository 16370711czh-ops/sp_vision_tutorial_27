#ifndef AUTO_AIM__TASK3_PIPELINE_HPP
#define AUTO_AIM__TASK3_PIPELINE_HPP

#include "outpost_tracker.hpp"
#include "src/task_3_fire.hpp"

namespace auto_aim
{
struct Task3Settings
{
  OutpostTrackConfig tracking;
  Task2AimConfig aiming;
  OutpostFireConfig firing;
  double feedback_timeout = 0.1;
  double camera_latency = 0;  // s; pose query = received image timestamp - this offset
  double max_reprojection_error_px = 5;
  int pose_timeout_ms = 20;
  // Manual calibration status for logging only; these flags do not gate fire.
  bool middle_height_verified = false;
  bool model_geometry_verified = false;
  bool follow_without_ballistics = true;
};

Task3Settings load_task3_settings(const std::string & path);
void validate_task3_settings(const Task3Settings & settings);

struct Task3Frame
{
  std::chrono::steady_clock::time_point image_time{}, now{};
  std::vector<Eigen::Vector4d> observations;
  bool image_valid = false, pose_valid = false, auto_mode = false, feedback_fresh = false;
  double yaw = 0, pitch = 0, bullet_speed = 0;
};

struct Task3Result
{
  Task2AimCommand aim;
  bool observed = false, control = false, fire = false, estimate_ready = false;
};

class Task3Pipeline
{
public:
  explicit Task3Pipeline(
    const Task3Settings & settings, bool enable_fire = false, bool dry_run = false);
  Task3Result step(const Task3Frame & frame);
  const OutpostTracker & tracker() const;

private:
  Task3Settings settings_;
  OutpostTracker tracker_;
  Task2Aimer aimer_;
  bool enable_fire_, dry_run_;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__TASK3_PIPELINE_HPP
