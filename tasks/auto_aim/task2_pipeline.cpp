#include "task2_pipeline.hpp"

#include <cmath>
#include <stdexcept>

#include "tools/math_tools.hpp"

namespace auto_aim
{
void validate_task2_settings(const Task2Settings & settings)
{
  const auto & a = settings.aiming;
  const auto & f = settings.tracking.filter;
  const std::vector<double> positive{
    a.pitch_limit,
    a.max_frame_age,
    a.max_facing_angle,
    a.min_bullet_speed,
    a.max_bullet_speed,
    settings.yaw_tolerance,
    settings.pitch_tolerance,
    settings.fire_facing_angle,
    settings.feedback_timeout,
    settings.max_reprojection_error_px,
    f.radius_variance,
    f.radius_diff_variance,
    f.height_diff_variance,
    f.max_nis,
    f.max_prediction_dt};
  for (auto value : positive) {
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("Invalid positive Task2 parameter.");
  }
  for (auto value :
       {a.fire_delay, f.acceleration_variance, f.angular_acceleration_variance,
        f.z_acceleration_variance}) {
    if (!std::isfinite(value) || value < 0)
      throw std::invalid_argument("Invalid nonnegative Task2 parameter.");
  }
  if (
    !std::isfinite(a.yaw_offset) || !std::isfinite(a.pitch_offset) || a.pitch_limit >= CV_PI / 2 ||
    a.max_facing_angle >= CV_PI / 2 || settings.fire_facing_angle > a.max_facing_angle ||
    a.max_bullet_speed <= a.min_bullet_speed || settings.shot_limit < 0 ||
    settings.pose_timeout_ms < 0 || settings.pose_timeout_ms > 100 ||
    !std::isfinite(settings.camera_latency) || std::abs(settings.camera_latency) > 0.2 ||
    settings.feedback_timeout > 0.1 || a.max_frame_age + a.fire_delay + 1 > f.max_prediction_dt)
    throw std::invalid_argument("Invalid Task2 parameter range.");
  Task2Tracker check(settings.tracking);
}

Task2Pipeline::Task2Pipeline(const Task2Settings & settings, bool enable_fire, bool dry_run)
: settings_(settings),
  tracker_(settings.tracking),
  aimer_(settings.aiming),
  shooter_(settings.shot_limit),
  enable_fire_(enable_fire),
  dry_run_(dry_run)
{
  validate_task2_settings(settings);
}

Task2Result Task2Pipeline::step(const Task2Frame & frame)
{
  if (!start_) start_ = frame.now;
  Task2Result result;
  tracker_.expire(frame.now);
  if (!frame.auto_mode || !frame.feedback_fresh) {
    tracker_.reset();
    return result;
  }
  auto age = tools::delta_time(frame.now, frame.image_time);
  if (
    !frame.image_valid || !frame.pose_valid || age < 0 || age > settings_.aiming.max_frame_age ||
    !std::isfinite(frame.yaw) || !std::isfinite(frame.pitch))
    return result;
  result.observed = tracker_.update(frame.observations, frame.image_time);
  if (!result.observed || !tracker_.target()) return result;
  result.estimate_ready = tracker_.ready();
  result.aim = aimer_.aim(*tracker_.target(), age, frame.bullet_speed, frame.yaw);
  if (!result.aim.valid && settings_.follow_without_ballistics)
    result.aim = aimer_.follow(*tracker_.target(), age, frame.yaw);
  result.control = !dry_run_ && result.aim.valid && tracker_.updates() >= 3;
  bool ready =
    enable_fire_ && result.control && result.estimate_ready && result.aim.ballistic &&
    std::abs(std::remainder(result.aim.yaw - frame.yaw, 2 * CV_PI)) <= settings_.yaw_tolerance &&
    std::abs(result.aim.pitch - frame.pitch) <= settings_.pitch_tolerance &&
    result.aim.facing_angle <= settings_.fire_facing_angle;
  result.fire = shooter_.request(ready, tools::delta_time(frame.now, *start_));
  return result;
}

const Task2Tracker & Task2Pipeline::tracker() const { return tracker_; }
int Task2Pipeline::shot_requests() const { return shooter_.shots(); }
}  // namespace auto_aim
