#ifndef AUTO_AIM__TASK2_TRACKER_HPP
#define AUTO_AIM__TASK2_TRACKER_HPP

#include "target.hpp"

namespace auto_aim
{
struct Task2Observation
{
  Eigen::Vector4d xyza;
  ArmorName name = ArmorName::not_armor;
  ArmorType type = ArmorType::small;
  double selection_score = 0;
};

struct Task2TrackConfig
{
  TargetConfig filter;
  int armor_num = 4;
  double initial_radius = 0.2;
  double lost_time = 0.3;
  double association_distance = 0.6;
  double association_angle = 1.0471975511965976;
  double max_observation_facing = 1.3962634015954636;
  int confirm_frames = 3;
  int min_updates = 20;
  double warmup_time = 0.5;
  double max_omega_variance = 0.25;
  double min_spin_speed = 2;
  double max_spin_speed = 12;
};

class Task2Tracker
{
public:
  using Time = std::chrono::steady_clock::time_point;
  explicit Task2Tracker(const Task2TrackConfig & config = Task2TrackConfig());
  bool update(const std::vector<Task2Observation> & observations, Time t);
  void expire(Time now);
  void reset();
  bool ready() const;
  const std::optional<Target> & target() const;
  int updates() const;
  unsigned seen_mask() const;
  unsigned track_id() const;
  double last_nis() const;

private:
  Task2TrackConfig config_;
  std::optional<Target> target_;
  std::optional<Task2Observation> pending_;
  Time last_input_{}, last_seen_{}, track_start_{}, pending_time_{};
  bool has_input_time_ = false;
  int updates_ = 0, confirmations_ = 0;
  unsigned seen_mask_ = 0, track_id_ = 0;
  double last_nis_ = 0;
  bool valid(const Task2Observation & observation) const;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__TASK2_TRACKER_HPP
