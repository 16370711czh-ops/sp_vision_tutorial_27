#ifndef TASK_1__ANGLES_HPP
#define TASK_1__ANGLES_HPP

#include <cmath>

namespace auto_aim
{
struct FollowAngles
{
  bool valid = false;
  double yaw = 0;
  double pitch = 0;
};

// 输入：世界坐标(m)、当前云台yaw(rad)、两个角度补偿(rad)、pitch限位(rad)。
// 输出：云台的绝对角度指令(rad)。只计算视线方向，不做弹道补偿。
inline FollowAngles follow_angles(
  double x, double y, double z, double current_yaw, double yaw_offset, double pitch_offset,
  double pitch_limit)
{
  constexpr double PI = 3.14159265358979323846;
  if (
    !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(current_yaw) ||
    !std::isfinite(yaw_offset) || !std::isfinite(pitch_offset) || !std::isfinite(pitch_limit) ||
    pitch_limit <= 0 || pitch_limit >= PI / 2) {
    return {};
  }

  auto horizontal_distance = std::hypot(x, y);
  if (!std::isfinite(horizontal_distance) || horizontal_distance < 1e-6) return {};

  auto target_yaw = std::atan2(y, x) + yaw_offset;
  // 选择距离当前yaw最近的等价角，避免跨过正负pi时产生一整圈的指令差。
  auto yaw_error = std::remainder(target_yaw - current_yaw, 2 * PI);
  if (!std::isfinite(yaw_error) || !std::isfinite(current_yaw + yaw_error)) return {};

  // 世界坐标z向上，视线仰角向上为正；沿用SP控制约定：抬头的pitch为负。
  auto pitch = -std::atan2(z, horizontal_distance) + pitch_offset;
  if (pitch > pitch_limit) pitch = pitch_limit;
  if (pitch < -pitch_limit) pitch = -pitch_limit;

  return {true, current_yaw + yaw_error, pitch};
}
}  // namespace auto_aim

#endif  // TASK_1__ANGLES_HPP
