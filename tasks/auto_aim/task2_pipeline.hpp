#ifndef AUTO_AIM__TASK2_PIPELINE_HPP
#define AUTO_AIM__TASK2_PIPELINE_HPP

#include "src/task_2_fire.hpp"
#include "task2_aimer.hpp"
#include "task2_tracker.hpp"

namespace auto_aim
{
struct Task2Settings
{
  Task2TrackConfig tracking;
  Task2AimConfig aiming;
  double yaw_tolerance = 0.8 * 3.14159265358979323846 / 180;
  double pitch_tolerance = 0.8 * 3.14159265358979323846 / 180;
  double fire_facing_angle = 35 * 3.14159265358979323846 / 180;
  double feedback_timeout = 0.1;
  double max_reprojection_error_px = 5;
  double camera_latency = 0;  // 有符号相对时延：查询姿态时刻 = 图像接收时刻 - 此值
  int pose_timeout_ms = 20;
  int shot_limit = 0;
  bool follow_without_ballistics = true;
};

Task2Settings load_task2_settings(const std::string & path);
void validate_task2_settings(const Task2Settings & settings);

struct Task2Frame
{
  std::chrono::steady_clock::time_point image_time{}, now{};
  std::vector<Task2Observation> observations;
  bool image_valid = false;
  bool pose_valid = false;
  bool auto_mode = false;
  bool feedback_fresh = false;
  double yaw = 0, pitch = 0, bullet_speed = 0;
};

struct Task2Result
{
  Task2AimCommand aim;
  bool observed = false, control = false, fire = false, estimate_ready = false;
};

class Task2Pipeline
{
public:
  explicit Task2Pipeline(
    const Task2Settings & settings, bool enable_fire = false, bool dry_run = false);
  Task2Result step(const Task2Frame & frame);
  const Task2Tracker & tracker() const;
  int shot_requests() const;

private:
  Task2Settings settings_;
  Task2Tracker tracker_;
  Task2Aimer aimer_;
  SingleShot shooter_;
  bool enable_fire_, dry_run_;
  std::optional<std::chrono::steady_clock::time_point> start_;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__TASK2_PIPELINE_HPP
