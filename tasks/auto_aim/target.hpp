#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{
struct TargetConfig
{
  double radius_variance = 0.01;       // r的初始方差，m^2
  double radius_diff_variance = 0.01;  // l的初始方差，m^2
  double height_diff_variance = 0.01;  // h的初始方差，m^2
  double acceleration_variance = 0.5;
  double angular_acceleration_variance = 4.0;
  double z_acceleration_variance = 0.1;
  double max_nis = 18.5;  // 运行门限，须结合真实残差检查
  double max_prediction_dt = 2.0;
  // 前哨站中心固定，三块中部装甲板共用半径，l/h分别表示id=1/2的高度差。
  double outpost_radius_variance = 1e-6;
  double outpost_center_noise = 1e-5;  // m^2/s，允许微小标定/姿态扰动
  double outpost_angular_acceleration_variance = 0.2;
};

class Target
{
public:
  ArmorName name = ArmorName::not_armor;
  ArmorType armor_type = ArmorType::small;
  ArmorPriority priority = ArmorPriority::first;
  bool jumped = false;
  int last_id = 0;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius = 0.2,
    int armor_num = 4, const TargetConfig & config = TargetConfig());

  // 同一套滤波逻辑也接受已解算的[x,y,z,yaw]，便于回放和离线验证。
  Target(
    const Eigen::Vector4d & xyza, ArmorName name, ArmorType type,
    std::chrono::steady_clock::time_point t, double radius = 0.2, int armor_num = 4,
    const TargetConfig & config = TargetConfig());

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);
  void update(const Eigen::Vector4d & xyza);
  bool try_update(const Eigen::Vector4d & xyza, int id);
  double innovation_distance(const Eigen::Vector4d & xyza, int id) const;
  static Eigen::Matrix4d measurement_covariance(const Eigen::Vector4d & xyza);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  // 与EKF更新共用的观测模型，供回放和雅各比数值校验使用。
  Eigen::Vector4d observation_model(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd observation_jacobian(const Eigen::VectorXd & x, int id) const;

  bool diverged() const;

  void update_convergency();
  bool is_converged() const;
  bool convergened();

  bool isinit = false;

  bool checkinit();

private:
  TargetConfig config_;
  int armor_num_ = 0;
  int switch_count_ = 0;
  int update_count_ = 0;

  bool is_switch_ = false, is_converged_ = false;
  bool rlh_initialized_ = false;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  bool apply_update(const Eigen::Vector4d & xyza, int id, double max_nis);
  bool update_ypda(const Eigen::Vector4d & xyza, int id, double max_nis);
  bool is_layered_outpost() const;

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
