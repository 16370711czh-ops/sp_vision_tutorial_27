#include "extended_kalman_filter.hpp"

#include <cmath>
#include <numeric>
#include <stdexcept>

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  if (
    x0.size() == 0 || P0.rows() != x0.size() || P0.cols() != x0.size() || !x0.allFinite() ||
    !P0.allFinite() || (P0 - P0.transpose()).norm() > 1e-8 || !P0.ldlt().isPositive())
    throw std::invalid_argument("[EKF] Invalid initial state/covariance.");
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nees"] = 0.0;
  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  if (
    F.rows() != x.size() || F.cols() != x.size() || Q.rows() != x.size() || Q.cols() != x.size() ||
    !F.allFinite() || !Q.allFinite() || (Q - Q.transpose()).norm() > 1e-8) {
    throw std::invalid_argument("[EKF] Invalid prediction dimensions or values.");
  }
  // Q可以秩亏；允许舍入量级的负特征值，不要求严格正定。
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> noise(Q, Eigen::EigenvaluesOnly);
  if (noise.info() != Eigen::Success || noise.eigenvalues().minCoeff() < -1e-10)
    throw std::invalid_argument("[EKF] Process covariance is not positive semidefinite.");
  Eigen::VectorXd next_x = f(x);
  Eigen::MatrixXd next_p = F * P * F.transpose() + Q;
  if (next_x.size() != x.size() || !next_x.allFinite() || !next_p.allFinite()) {
    throw std::runtime_error("[EKF] Non-finite prediction.");
  }
  x = next_x;
  P = (next_p + next_p.transpose()) * 0.5;
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  if (!try_update(z, H, R, h, z_subtract)) {
    throw std::runtime_error("[EKF] Invalid update; state preserved.");
  }
  return x;
}

bool ExtendedKalmanFilter::try_update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract,
  double max_nis)
{
  data["update_accepted"] = 0;
  if (
    x.size() == 0 || H.cols() != x.size() || H.rows() != z.size() || R.rows() != z.size() ||
    R.cols() != z.size() || P.rows() != x.size() || P.cols() != x.size() || !x.allFinite() ||
    !P.allFinite() || !z.allFinite() || !H.allFinite() || !R.allFinite() || std::isnan(max_nis) ||
    max_nis <= 0)
    return false;
  Eigen::VectorXd predicted = h(x);
  if (predicted.size() != z.size() || !predicted.allFinite()) return false;
  Eigen::VectorXd residual = z_subtract(z, predicted);
  if (residual.size() != z.size() || !residual.allFinite()) return false;
  if ((R - R.transpose()).norm() > 1e-8 || !R.ldlt().isPositive()) return false;
  Eigen::MatrixXd S = H * P * H.transpose() + R;
  auto decomposition = S.ldlt();
  if (
    decomposition.info() != Eigen::Success || !decomposition.isPositive() ||
    decomposition.vectorD().minCoeff() <= 1e-12)
    return false;
  auto nis = residual.dot(decomposition.solve(residual));
  if (!std::isfinite(nis) || nis < -1e-8) return false;
  last_nis = std::max(0.0, nis);
  data["nis"] = last_nis;
  // 四维观测的95%阈值；不将其解释为真实估计误差。
  const bool nis_failed = last_nis > 9.487729036781154;
  data["nis_fail"] = nis_failed ? 1 : 0;
  recent_nis_failures.push_back(nis_failed ? 1 : 0);
  while (recent_nis_failures.size() > std::max<size_t>(1, window_size))
    recent_nis_failures.pop_front();
  data["recent_nis_failures"] = static_cast<double>(std::accumulate(
                                  recent_nis_failures.begin(), recent_nis_failures.end(), 0)) /
                                recent_nis_failures.size();
  if (last_nis > max_nis) return false;

  Eigen::MatrixXd K = decomposition.solve((P * H.transpose()).transpose()).transpose();
  Eigen::VectorXd next_x = x_add(x, K * residual);
  Eigen::MatrixXd IKH = I - K * H;
  Eigen::MatrixXd next_p = IKH * P * IKH.transpose() + K * R * K.transpose();
  next_p = ((next_p + next_p.transpose()) * 0.5).eval();
  if (
    next_x.size() != x.size() || !next_x.allFinite() || !next_p.allFinite() ||
    !next_p.ldlt().isPositive())
    return false;

  // 该量是校正幅度的Mahalanobis量，不是需要真实状态才能计算的NEES。
  Eigen::VectorXd correction = K * residual;  // 切空间校正，避免角度wrap产生虚假的2π跳变
  auto posterior = next_p.ldlt();
  bool diagnostic_ok = posterior.info() == Eigen::Success && posterior.vectorD().minCoeff() > 1e-12;
  data["correction_mahalanobis"] = diagnostic_ok ? correction.dot(posterior.solve(correction)) : 0;
  data["correction_diagnostic_valid"] = diagnostic_ok ? 1 : 0;
  data["nees"] = 0;  // 兼容旧键，不能声称已获得真值NEES。
  data["nees_valid"] = 0;
  data["nees_fail"] = 0;
  if (residual.size() == 4) {
    data["residual_yaw"] = residual[0];
    data["residual_pitch"] = residual[1];
    data["residual_distance"] = residual[2];
    data["residual_angle"] = residual[3];
  }
  x = next_x;
  P = next_p;
  data["update_accepted"] = 1;
  return true;
}

}  // namespace tools
