#ifndef AUTO_AIM__TASK2_AIMER_HPP
#define AUTO_AIM__TASK2_AIMER_HPP

#include "target.hpp"

namespace auto_aim
{
struct Task2AimConfig
{
  double yaw_offset = 0;     // rad
  double pitch_offset = 0;   // 仰角补偿，rad；在取负前相加，沿用SP的Aimer约定
  double fire_delay = 0.03;  // s，需要实机标定
  double pitch_limit = 20 * 3.14159265358979323846 / 180;
  double max_facing_angle = 60 * 3.14159265358979323846 / 180;
  double max_frame_age = 0.3;
  double min_bullet_speed = 14;
  double max_bullet_speed = 35;
};

struct Task2AimCommand
{
  bool valid = false;
  double yaw = 0;
  double pitch = 0;
  double fly_time = 0;
  double lead_time = 0;
  double facing_angle = 0;
  int armor_id = -1;
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  bool ballistic = false;
};

class Task2Aimer
{
public:
  explicit Task2Aimer(const Task2AimConfig & config = Task2AimConfig());

  Task2AimCommand aim(
    const Target & target, double frame_age, double bullet_speed, double current_yaw) const;
  Task2AimCommand follow(const Target & target, double frame_age, double current_yaw) const;

private:
  Task2AimConfig config_;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__TASK2_AIMER_HPP
