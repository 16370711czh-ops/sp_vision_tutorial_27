#ifndef AUTO_AIM__OUTPOST_TRACKER_HPP
#define AUTO_AIM__OUTPOST_TRACKER_HPP

#include <optional>

#include "target.hpp"

namespace auto_aim
{
struct OutpostTrackConfig
{
  TargetConfig filter;
  double radius = 0.2765;
  // 世界坐标系原点在云台轴交点，须按实际中部板高度配置，单位m。
  double middle_z_min = 0.1;
  double middle_z_max = 0.5;
  double lost_time = 0.4;
  double association_distance = 0.35;
  double association_angle = 3.14159265358979323846 / 3;
  double warmup_time = 2.0;
  int min_updates = 50;
  double max_omega_variance = 0.04;
  double min_spin_speed = 1.8;
  double max_spin_speed = 3.2;
};

class OutpostTracker
{
public:
  explicit OutpostTracker(const OutpostTrackConfig & config = OutpostTrackConfig());

  // 输入已经经过outpost类别/颜色筛选和PnP的[x,y,z,yaw]，每帧至多更新一次。
  bool update(
    const std::vector<Eigen::Vector4d> & observations, std::chrono::steady_clock::time_point t);
  void expire(std::chrono::steady_clock::time_point now);
  void reset();
  bool accepts_height(double z) const;
  bool ready() const;
  const std::optional<Target> & target() const;
  int updates() const;
  unsigned seen_mask() const;
  double last_nis() const;

private:
  OutpostTrackConfig config_;
  std::optional<Target> target_;
  std::chrono::steady_clock::time_point last_predict_, last_seen_, track_start_;
  int updates_ = 0;
  unsigned seen_mask_ = 0;
  double last_nis_ = 0;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__OUTPOST_TRACKER_HPP
