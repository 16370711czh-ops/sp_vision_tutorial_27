#include "task3_pipeline.hpp"

#include <cmath>
#include <stdexcept>

#include "tools/math_tools.hpp"

namespace auto_aim
{
void validate_task3_settings(const Task3Settings & settings)
{
  const auto & a = settings.aiming;
  const auto & f = settings.firing;
  for (auto value :
       {a.pitch_limit, a.max_frame_age, a.max_facing_angle, a.min_bullet_speed,
        a.max_bullet_speed, f.yaw_tolerance, f.pitch_tolerance, f.facing_angle,
        settings.feedback_timeout, settings.max_reprojection_error_px}) {
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("Invalid positive Task3 setting.");
  }
  if (
    !std::isfinite(a.yaw_offset) || !std::isfinite(a.pitch_offset) ||
    !std::isfinite(a.fire_delay) || a.fire_delay < 0 || a.pitch_limit >= CV_PI / 2 ||
    a.max_facing_angle >= CV_PI / 2 || f.facing_angle > a.max_facing_angle ||
    a.max_bullet_speed <= a.min_bullet_speed || settings.feedback_timeout > 0.1 ||
    settings.pose_timeout_ms < 0 || settings.pose_timeout_ms > 100 ||
    !std::isfinite(settings.camera_latency) || std::abs(settings.camera_latency) > 0.2 ||
    a.max_frame_age + a.fire_delay + 1 > settings.tracking.filter.max_prediction_dt)
    throw std::invalid_argument("Invalid Task3 setting range.");
  OutpostTracker check(settings.tracking);
}

Task3Pipeline::Task3Pipeline(const Task3Settings & settings, bool enable_fire, bool dry_run)
: settings_(settings),
  tracker_(settings.tracking),
  aimer_(settings.aiming),
  enable_fire_(enable_fire),
  dry_run_(dry_run)
{
  validate_task3_settings(settings);
}

Task3Result Task3Pipeline::step(const Task3Frame & frame)
{
  Task3Result result;
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
  // Continuous permission: the controller provides the specified 12Hz cadence.
  // No Task2 SingleShot limiter, no packet-count to shot-count assumption.
  result.fire = outpost_fire_ready(
    enable_fire_ && settings_.middle_height_verified, result.control, result.estimate_ready,
    result.aim, frame.yaw, frame.pitch, settings_.firing);
  return result;
}

const OutpostTracker & Task3Pipeline::tracker() const { return tracker_; }
}  // namespace auto_aim
