#ifndef TASK_1__FOLLOW_HPP
#define TASK_1__FOLLOW_HPP

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "task_1_angles.hpp"
#include "tools/yaml.hpp"

namespace auto_aim
{
struct FollowSettings
{
  double yaw_offset = 0;
  double pitch_offset = 0;
  double pitch_limit = 20.0 * 3.14159265358979323846 / 180;
  double max_frame_age = 0.15;
  double feedback_timeout = 0.1;
  double camera_to_pose_offset = 0;
  int pose_timeout_ms = 20;
  int command_timeout_ms = 0;
};

inline FollowSettings load_follow_settings(const YAML::Node & yaml)
{
  constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180;
  FollowSettings s;
  s.yaw_offset = tools::read_or<double>(yaml, "task1_yaw_offset", 0.0) * DEG_TO_RAD;
  s.pitch_offset = tools::read_or<double>(yaml, "task1_pitch_offset", 0.0) * DEG_TO_RAD;
  s.pitch_limit = tools::read_or<double>(yaml, "task1_pitch_limit", 20.0) * DEG_TO_RAD;
  s.max_frame_age = tools::read_or<double>(yaml, "task1_max_frame_age", 0.15);
  s.feedback_timeout = tools::read_or<double>(yaml, "task1_feedback_timeout", 0.1);
  s.camera_to_pose_offset =
    tools::read_or<double>(yaml, "task1_camera_to_pose_offset_ms", 0.0) / 1000;
  s.pose_timeout_ms = tools::read_or<int>(yaml, "task1_pose_timeout_ms", 20);
  s.command_timeout_ms = tools::read_or<int>(yaml, "command_timeout_ms", 0);
  if (
    !std::isfinite(s.yaw_offset) || !std::isfinite(s.pitch_offset) ||
    !std::isfinite(s.pitch_limit) || s.pitch_limit <= 0 || s.pitch_limit >= 90 * DEG_TO_RAD ||
    !std::isfinite(s.max_frame_age) || s.max_frame_age <= 0 || s.max_frame_age > 1 ||
    !std::isfinite(s.feedback_timeout) || s.feedback_timeout <= 0 || s.feedback_timeout > 0.1 ||
    !std::isfinite(s.camera_to_pose_offset) || std::abs(s.camera_to_pose_offset) > 1 ||
    s.pose_timeout_ms < 0 || s.pose_timeout_ms > 100 || s.command_timeout_ms < 0 ||
    s.command_timeout_ms > 1000) {
    throw std::invalid_argument("Invalid Task1 angle/timing settings.");
  }
  return s;
}

struct FollowContext
{
  bool auto_mode = false;
  bool feedback_valid = false;
  bool dry_run = false;
  double frame_age = -1;
  double feedback_age = -1;
};

struct FollowCommand
{
  bool control = false;
  float yaw = 0;
  float pitch = 0;
};

// 与主程序共用的无状态控制门限：本帧无解、过期、退出挡位均不沿用旧命令。
inline FollowCommand follow_command(
  const FollowAngles & aim, const FollowContext & context, const FollowSettings & settings)
{
  auto float_limit = static_cast<double>(std::numeric_limits<float>::max());
  if (
    !aim.valid || !context.auto_mode || !context.feedback_valid || context.dry_run ||
    !std::isfinite(context.frame_age) || context.frame_age < 0 ||
    context.frame_age > settings.max_frame_age || !std::isfinite(context.feedback_age) ||
    context.feedback_age < 0 || context.feedback_age > settings.feedback_timeout ||
    !std::isfinite(aim.yaw) || !std::isfinite(aim.pitch) || std::abs(aim.yaw) > float_limit ||
    std::abs(aim.pitch) > settings.pitch_limit) {
    return {};
  }
  return {true, static_cast<float>(aim.yaw), static_cast<float>(aim.pitch)};
}
}  // namespace auto_aim

#endif  // TASK_1__FOLLOW_HPP
