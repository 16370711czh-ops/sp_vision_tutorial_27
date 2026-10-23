#include "task2_aimer.hpp"

#include <cmath>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
Task2Aimer::Task2Aimer(const Task2AimConfig & config) : config_(config) {}

Task2AimCommand Task2Aimer::aim(
  const Target & target, double frame_age, double bullet_speed, double current_yaw) const
{
  if (
    !std::isfinite(frame_age) || frame_age < 0 || frame_age > config_.max_frame_age ||
    !std::isfinite(bullet_speed) || bullet_speed < config_.min_bullet_speed ||
    bullet_speed > config_.max_bullet_speed || !std::isfinite(current_yaw) || target.diverged()) {
    return {};
  }

  double fly_time = 0;
  for (int i = 0; i < 10; i++) {
    // 每轮都从实时滤波结果复制，绝不将真实target推进到弹丸到达的时刻。
    auto future = target;
    auto lead_time = frame_age + config_.fire_delay + fly_time;
    future.predict(lead_time);
    auto armors = future.armor_xyza_list();

    int best_id = -1;
    double best_angle = config_.max_facing_angle;
    for (int id = 0; id < static_cast<int>(armors.size()); id++) {
      const auto & armor = armors[id];
      auto direction = std::atan2(armor[1], armor[0]);
      auto angle = std::abs(tools::limit_rad(armor[3] - direction));
      if (armor.allFinite() && angle < best_angle) {
        best_angle = angle;
        best_id = id;
      }
    }
    if (best_id < 0) return {};

    const auto & armor = armors[best_id];
    auto distance = std::hypot(armor[0], armor[1]);
    if (!std::isfinite(distance) || distance < 1e-3) return {};

    // 复用课程仓库的无空气阻力弹道：距离、高度 -> 仰角和飞行时间。
    tools::Trajectory trajectory(bullet_speed, distance, armor[2], 1);
    if (
      trajectory.unsolvable || !std::isfinite(trajectory.pitch) ||
      !std::isfinite(trajectory.fly_time) || trajectory.fly_time <= 0 ||
      trajectory.fly_time > 1.0) {
      return {};
    }

    auto pitch = -(trajectory.pitch + config_.pitch_offset);
    if (std::abs(pitch) > config_.pitch_limit) return {};

    if (std::abs(trajectory.fly_time - fly_time) < 1e-4) {
      auto target_yaw = std::atan2(armor[1], armor[0]) + config_.yaw_offset;
      auto yaw = current_yaw + std::remainder(target_yaw - current_yaw, 2 * CV_PI);
      return {true,      yaw,        pitch,   trajectory.fly_time,
              lead_time, best_angle, best_id, armor.head<3>(),
              true};
    }
    fly_time = trajectory.fly_time;
  }

  // 候选板切换附近若迭代不收敛，本帧不下发击打指令。
  return {};
}

Task2AimCommand Task2Aimer::follow(
  const Target & target, double frame_age, double current_yaw) const
{
  if (
    !std::isfinite(frame_age) || frame_age < 0 || frame_age > config_.max_frame_age ||
    !std::isfinite(current_yaw) || target.diverged())
    return {};
  auto future = target;
  future.predict(frame_age);
  auto armors = future.armor_xyza_list();
  int best = -1;
  double facing = config_.max_facing_angle;
  for (int id = 0; id < static_cast<int>(armors.size()); ++id) {
    auto angle =
      std::abs(std::remainder(armors[id][3] - std::atan2(armors[id][1], armors[id][0]), 2 * CV_PI));
    if (angle < facing) {
      facing = angle;
      best = id;
    }
  }
  if (best < 0) return {};
  const auto & armor = armors[best];
  auto distance = std::hypot(armor[0], armor[1]);
  auto pitch = -(std::atan2(armor[2], distance) + config_.pitch_offset);
  if (distance < 1e-3 || !std::isfinite(pitch) || std::abs(pitch) > config_.pitch_limit) return {};
  auto yaw =
    current_yaw +
    std::remainder(std::atan2(armor[1], armor[0]) + config_.yaw_offset - current_yaw, 2 * CV_PI);
  return {true, yaw, pitch, 0, frame_age, facing, best, armor.head<3>(), false};
}
}  // namespace auto_aim
