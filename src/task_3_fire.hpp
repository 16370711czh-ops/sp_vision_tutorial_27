#ifndef TASK_3_FIRE_HPP
#define TASK_3_FIRE_HPP

#include <cmath>

#include "tasks/auto_aim/task2_aimer.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
struct OutpostFireConfig
{
  double yaw_tolerance = 0.5 * 3.14159265358979323846 / 180;
  double pitch_tolerance = 0.5 * 3.14159265358979323846 / 180;
  double facing_angle = 30 * 3.14159265358979323846 / 180;
};

inline bool outpost_fire_ready(
  bool enabled, bool control, bool estimate_ready, const Task2AimCommand & command,
  double current_yaw, double current_pitch, const OutpostFireConfig & config)
{
  // 连发模式：持续输出允许开火的电平，12Hz由考核电控配置。
  return enabled && control && estimate_ready && command.valid && command.ballistic &&
         std::isfinite(command.yaw) && std::isfinite(command.pitch) &&
         std::isfinite(command.facing_angle) && command.facing_angle >= 0 && std::isfinite(current_yaw) &&
         std::isfinite(current_pitch) &&
         std::abs(tools::limit_rad(command.yaw - current_yaw)) <= config.yaw_tolerance &&
         std::abs(command.pitch - current_pitch) <= config.pitch_tolerance &&
         command.facing_angle <= config.facing_angle;
}
}  // namespace auto_aim

#endif  // TASK_3_FIRE_HPP
