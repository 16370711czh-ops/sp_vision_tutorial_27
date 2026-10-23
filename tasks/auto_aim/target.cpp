#include "target.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  const TargetConfig & config)
: Target(
    Eigen::Vector4d(
      armor.xyz_in_world[0], armor.xyz_in_world[1], armor.xyz_in_world[2], armor.ypr_in_world[0]),
    armor.name, armor.type, t, radius, armor_num, config)
{
}

Target::Target(
  const Eigen::Vector4d & xyza, ArmorName name, ArmorType type,
  std::chrono::steady_clock::time_point t, double radius, int armor_num,
  const TargetConfig & config)
: name(name),
  armor_type(type),
  priority(ArmorPriority::first),
  jumped(false),
  last_id(0),
  isinit(true),
  config_(config),
  armor_num_(armor_num > 0 ? armor_num : (name == ArmorName::outpost ? 3 : 4)),
  switch_count_(0),
  update_count_(0),
  is_switch_(false),
  is_converged_(false),
  t_(t)
{
  if (
    !xyza.allFinite() || !std::isfinite(xyza.head<3>().norm()) || xyza.head<3>().norm() < 0.1 ||
    xyza.head<3>().norm() > 100 || (armor_num_ != 2 && armor_num_ != 3 && armor_num_ != 4) ||
    !std::isfinite(config_.max_nis) || config_.max_nis <= 0 ||
    !std::isfinite(config_.max_prediction_dt) || config_.max_prediction_dt <= 0 ||
    !std::isfinite(radius))
    throw std::invalid_argument("[Target] Invalid initial pose/config.");
  auto r = radius > 0.0 ? radius : (name == ArmorName::outpost ? 0.2765 : 0.2);
  for (auto variance :
       {config_.radius_variance, config_.radius_diff_variance, config_.height_diff_variance,
        config_.outpost_radius_variance}) {
    if (!std::isfinite(variance) || variance <= 0)
      throw std::invalid_argument("[Target] Invalid geometry covariance.");
  }
  for (auto variance :
       {config_.acceleration_variance, config_.angular_acceleration_variance,
        config_.z_acceleration_variance, config_.outpost_center_noise,
        config_.outpost_angular_acceleration_variance}) {
    if (!std::isfinite(variance) || variance < 0)
      throw std::invalid_argument("[Target] Invalid process noise.");
  }
  if (r <= 0.05 || r >= 0.5) throw std::invalid_argument("[Target] Invalid initial radius.");

  // 旋转中心的坐标
  auto center_x = xyza[0] + r * std::cos(xyza[3]);
  auto center_y = xyza[1] + r * std::sin(xyza[3]);
  auto center_z = xyza[2];

  // x vx y vy z vz a w r l h
  // a: angle
  // w: angular velocity
  // tips for outpost:
  // l: dh1 for outpost / r2 - r1 for vehicles
  // h: dh2 for outpost / z2 - z1 for vehicles
  Eigen::VectorXd x0{
    {center_x, 0, center_y, 0, center_z, 0, std::remainder(xyza[3], 2 * CV_PI), 0, r, 0, 0}};
  Eigen::VectorXd P0_dig_car{
    {1, 64, 1, 64, 1, 64, 0.4, 100, config_.radius_variance, config_.radius_diff_variance,
     config_.height_diff_variance}};

  // 前哨站共用用户提供的半径先验（须实物/CAD核对），从0估计转速。
  Eigen::VectorXd P0_dig_outpost{
    {0.1, 0.01, 0.1, 0.01, 0.1, 0.01, 0.4, 100, config_.outpost_radius_variance,
     config_.radius_diff_variance, config_.height_diff_variance}};
  Eigen::Matrix<double, 11, 11> P0 =
    (is_layered_outpost() ? P0_dig_outpost : P0_dig_car).asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = std::remainder(c[6], 2 * CV_PI);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  if (!std::isfinite(dt) || dt < 0 || dt > config_.max_prediction_dt)
    throw std::invalid_argument("[Target] Invalid dt.");
  // 状态转移矩阵
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  if (is_layered_outpost()) {
    // 固定中心模型：保留11维接口，但不让闲置的vx/vy/vz推进中心。
    F(0, 1) = F(2, 3) = F(4, 5) = 0;
  }

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  double v1, v2;

  // 任务二：固定位置的转靶，中心运动较小，但允许角速度变化。
  v1 = config_.acceleration_variance;
  v2 = config_.angular_acceleration_variance;
  double v1_z = config_.z_acceleration_variance;

  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // 预测过程噪声偏差的方差
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1_z, b * v1_z,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1_z, c * v1_z,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on

  if (is_layered_outpost()) {
    Q.setZero();
    Q(0, 0) = Q(2, 2) = Q(4, 4) = config_.outpost_center_noise * dt;
    auto angular_noise = config_.outpost_angular_acceleration_variance;
    Q(6, 6) = a * angular_noise;
    Q(6, 7) = Q(7, 6) = b * angular_noise;
    Q(7, 7) = c * angular_noise;
  }

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  ekf_.predict(F, Q, f);
}

void Target::update(const Armor & armor)
{
  update(
    Eigen::Vector4d(
      armor.xyz_in_world[0], armor.xyz_in_world[1], armor.xyz_in_world[2], armor.ypr_in_world[0]));
}

void Target::update(const Eigen::Vector4d & observation)
{
  if (!observation.allFinite() || observation.head<3>().norm() < 1e-6) {
    throw std::invalid_argument("[Target] Invalid observation.");
  }
  auto best_id = -1;
  auto best_score = std::numeric_limits<double>::infinity();
  for (int id = 0; id < armor_num_; ++id) {
    auto score = innovation_distance(observation, id);
    if (score < best_score) {
      best_score = score;
      best_id = id;
    }
  }
  // 保留旧void接口无NIS门限的语义，避免给Task3旧调用增加隐式拒绝行为。
  if (best_id >= 0) apply_update(observation, best_id, std::numeric_limits<double>::infinity());
}

bool Target::try_update(const Eigen::Vector4d & observation, int id)
{
  return apply_update(observation, id, config_.max_nis);
}

bool Target::apply_update(const Eigen::Vector4d & observation, int id, double max_nis)
{
  if (
    !observation.allFinite() || observation.head<3>().norm() < 0.1 || id < 0 || id >= armor_num_ ||
    diverged())
    return false;
  if (!update_ypda(observation, id, max_nis)) return false;
  is_switch_ = id != last_id;
  if (is_switch_) {
    ++switch_count_;
    rlh_initialized_ = true;
  }
  if (id != 0) jumped = true;
  last_id = id;
  ++update_count_;
  update_convergency();
  return true;
}

double Target::innovation_distance(const Eigen::Vector4d & xyza, int id) const
{
  if (!xyza.allFinite() || xyza.head<3>().norm() < 0.1 || id < 0 || id >= armor_num_ || diverged())
    return std::numeric_limits<double>::infinity();
  const Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head<3>());
  Eigen::Vector4d residual =
    Eigen::Vector4d(ypd[0], ypd[1], ypd[2], xyza[3]) - observation_model(ekf_.x, id);
  for (int row : {0, 1, 3}) residual[row] = std::remainder(residual[row], 2 * CV_PI);
  auto H = observation_jacobian(ekf_.x, id);
  Eigen::Matrix4d S = H * ekf_.P * H.transpose() + measurement_covariance(xyza);
  auto decomposition = S.ldlt();
  if (decomposition.info() != Eigen::Success || !decomposition.isPositive())
    return std::numeric_limits<double>::infinity();
  auto score = residual.dot(decomposition.solve(residual));
  return std::isfinite(score) && score >= 0 ? score : std::numeric_limits<double>::infinity();
}

Eigen::Matrix4d Target::measurement_covariance(const Eigen::Vector4d & xyza)
{
  if (!xyza.allFinite() || xyza.head<3>().norm() < 0.1)
    throw std::invalid_argument("[Target] Invalid noise-model input.");
  // 随距自适应量测噪声
  const Eigen::Vector3d observed_ypd = tools::xyz2ypd(xyza.head<3>());
  const double center_yaw = std::atan2(xyza[1], xyza[0]);
  const double delta_angle = std::remainder(xyza[3] - center_yaw, 2 * CV_PI);
  const double distance = observed_ypd[2];
  const double distance_sq = distance * distance;

  const double r_cam_yaw = 5e-4;
  const double r_cam_pitch = 5e-4;
  const double r_distance = (1e-2 + 5e-3 * distance_sq) * (std::log(std::abs(delta_angle) + 1) + 1);
  const double r_armor_yaw = 5e-2 + 1e-2 * distance_sq + 2e-1 / distance;

  return Eigen::Vector4d(r_cam_yaw, r_cam_pitch, r_distance, r_armor_yaw).asDiagonal();
}

bool Target::update_ypda(const Eigen::Vector4d & xyza, int id, double max_nis)
{
  // H始终是实际h的导数；不再清零转角列或几何列。
  auto H = observation_jacobian(ekf_.x, id);
  auto R = measurement_covariance(xyza);
  const Eigen::Vector3d observed_ypd = tools::xyz2ypd(xyza.head<3>());

  // 定义非线性转换函数h: x -> z
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d { return observation_model(x, id); };

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = std::remainder(c[0], 2 * CV_PI);
    c[1] = std::remainder(c[1], 2 * CV_PI);
    c[3] = std::remainder(c[3], 2 * CV_PI);
    return c;
  };

  Eigen::Vector4d z{observed_ypd[0], observed_ypd[1], observed_ypd[2], xyza[3]};

  return ekf_.try_update(z, H, R, h, z_subtract, max_nis);
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  if (!isinit || ekf_.x.size() != 11) return {};
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}

bool Target::diverged() const
{
  if (ekf_.x.size() != 11 || ekf_.P.rows() != 11 || ekf_.P.cols() != 11) return true;
  if (!ekf_.x.allFinite() || !ekf_.P.allFinite() || ekf_.P.diagonal().minCoeff() < -1e-8)
    return true;
  if (
    Eigen::Vector3d(ekf_.x[0], ekf_.x[2], ekf_.x[4]).norm() > 100 ||
    Eigen::Vector3d(ekf_.x[1], ekf_.x[3], ekf_.x[5]).norm() > 20)
    return true;
  if (std::abs(ekf_.x[7]) > 30 || std::abs(ekf_.x[10]) > 0.5) return true;
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;

  if (is_layered_outpost()) {
    // l/h是相对id=0的高度差，可正可负，不再用r+l判定半径。
    return !(
      ekf_.x[8] > 0.22 && ekf_.x[8] < 0.33 && std::abs(ekf_.x[9]) < 0.25 &&
      std::abs(ekf_.x[10]) < 0.25 && std::abs(ekf_.x[9] - ekf_.x[10]) < 0.25 &&
      std::abs(ekf_.x[7]) < 6);
  }

  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

void Target::update_convergency() { is_converged_ = !diverged(); }

bool Target::is_converged() const { return is_converged_; }

bool Target::convergened()
{
  update_convergency();
  return is_converged();
}

bool Target::is_layered_outpost() const { return name == ArmorName::outpost && armor_num_ == 3; }

// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);
  auto use_outpost_h = is_layered_outpost();

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

  if (use_outpost_h) {
    auto dh = id == 1 ? x[9] : (id == 2 ? x[10] : 0.0);
    armor_z = x[4] + dh;
  }

  return {armor_x, armor_y, armor_z};
}

Eigen::Vector4d Target::observation_model(const Eigen::VectorXd & x, int id) const
{
  if (x.size() != 11 || !x.allFinite() || id < 0 || id >= armor_num_)
    throw std::invalid_argument("[Target] Invalid model input.");
  const Eigen::Vector3d xyz = h_armor_xyz(x, id);
  const Eigen::Vector3d ypd = tools::xyz2ypd(xyz);
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  return {ypd[0], ypd[1], ypd[2], angle};
}

Eigen::MatrixXd Target::observation_jacobian(const Eigen::VectorXd & x, int id) const
{
  if (x.size() != 11 || !x.allFinite() || id < 0 || id >= armor_num_)
    throw std::invalid_argument("[Target] Invalid model input.");
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);
  auto use_outpost_h = is_layered_outpost();

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = use_l_h ? 1.0 : 0.0;

  auto dz_dl = (use_outpost_h && id == 1) ? 1.0 : 0.0;
  if (use_outpost_h) dz_dh = id == 2 ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0, dz_dl, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim
