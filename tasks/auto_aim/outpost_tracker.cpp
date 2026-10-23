#include "outpost_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "tools/math_tools.hpp"

namespace auto_aim
{
OutpostTracker::OutpostTracker(const OutpostTrackConfig & config) : config_(config)
{
  const std::vector<double> positive{
    config_.radius,
    config_.lost_time,
    config_.association_distance,
    config_.association_angle,
    config_.max_omega_variance,
    config_.min_spin_speed,
    config_.max_spin_speed,
    config_.filter.outpost_radius_variance,
    config_.filter.radius_diff_variance,
    config_.filter.height_diff_variance,
    config_.filter.max_nis,
    config_.filter.max_prediction_dt};
  for (auto value : positive) {
    if (!std::isfinite(value) || value <= 0) {
      throw std::invalid_argument("[OutpostTracker] Invalid positive setting.");
    }
  }
  if (
    !std::isfinite(config_.middle_z_min) || !std::isfinite(config_.middle_z_max) ||
    config_.middle_z_min >= config_.middle_z_max || !std::isfinite(config_.warmup_time) ||
    config_.warmup_time < 0 || config_.min_updates < 3 || config_.radius <= 0.22 ||
    config_.radius >= 0.33 || config_.association_angle > 3.14159265358979323846 / 2 ||
    config_.max_spin_speed <= config_.min_spin_speed || config_.max_spin_speed >= 6 ||
    !std::isfinite(config_.filter.outpost_center_noise) ||
    config_.filter.outpost_center_noise < 0 ||
    !std::isfinite(config_.filter.outpost_angular_acceleration_variance) ||
    config_.filter.outpost_angular_acceleration_variance < 0) {
    throw std::invalid_argument("[OutpostTracker] Invalid outpost settings.");
  }
}

bool OutpostTracker::update(
  const std::vector<Eigen::Vector4d> & observations, std::chrono::steady_clock::time_point t)
{
  expire(t);
  if (target_) {
    auto dt = tools::delta_time(t, last_predict_);
    // 拒绝重复或倒退的相机时间戳，避免同一图像被多次融合。
    if (dt <= 0) return false;
    if (dt > config_.filter.max_prediction_dt) reset();
  }
  if (target_) {
    target_->predict(t);
    last_predict_ = t;
    if (target_->diverged()) reset();
  }

  const Eigen::Vector4d * best = nullptr;
  int best_id = 0;
  auto best_score = std::numeric_limits<double>::infinity();
  for (const auto & observation : observations) {
    if (
      !observation.allFinite() || !accepts_height(observation[2]) ||
      observation.head<3>().norm() < 0.1 || observation.head<3>().norm() > 100) {
      continue;
    }
    // 初次锁定正对相机的中部板，把它作为id=0，绝不按高度强行固定板号。
    double score =
      std::abs(tools::limit_rad(observation[3] - std::atan2(observation[1], observation[0])));
    if (score > config_.association_angle) continue;
    int matched_id = 0;
    if (target_) {
      score = std::numeric_limits<double>::infinity();
      const auto predicted_armors = target_->armor_xyza_list();
      for (int id = 0; id < 3; ++id) {
        const auto & predicted = predicted_armors[id];
        auto distance = (observation.head<3>() - predicted.head<3>()).norm();
        auto angle = std::abs(tools::limit_rad(observation[3] - predicted[3]));
        if (distance < config_.association_distance && angle < config_.association_angle) {
          const auto nis = target_->innovation_distance(observation, id);
          if (std::isfinite(nis) && nis <= config_.filter.max_nis && nis < score) {
            score = nis;
            matched_id = id;
          }
        }
      }
    }
    if (score < best_score) {
      best_score = score;
      best = &observation;
      best_id = matched_id;
    }
  }
  if (!best) return false;

  const bool initializing = !target_;
  if (!target_) {
    target_.emplace(
      *best, ArmorName::outpost, ArmorType::special, t, config_.radius, 3, config_.filter);
    track_start_ = last_predict_ = t;
    updates_ = 1;
    seen_mask_ = 1;
  } else {
    // Fuse exactly the pair accepted by association; count only accepted EKF updates.
    if (!target_->try_update(*best, best_id)) return false;
    ++updates_;
    seen_mask_ |= 1u << target_->last_id;
  }
  if (target_->diverged()) {
    reset();
    return false;
  }
  last_nis_ = initializing ? 0 : best_score;
  last_seen_ = t;
  return true;
}

void OutpostTracker::expire(std::chrono::steady_clock::time_point now)
{
  if (target_ && tools::delta_time(now, last_seen_) > config_.lost_time) reset();
}

void OutpostTracker::reset()
{
  target_.reset();
  updates_ = 0;
  seen_mask_ = 0;
  last_nis_ = 0;
}

bool OutpostTracker::accepts_height(double z) const
{
  return std::isfinite(z) && z >= config_.middle_z_min && z <= config_.middle_z_max;
}

bool OutpostTracker::ready() const
{
  if (!target_ || target_->diverged()) return false;
  auto omega = std::abs(target_->ekf_x()[7]);
  auto variance = target_->ekf().P(7, 7);
  return updates_ >= config_.min_updates && seen_mask_ == 7 &&
         tools::delta_time(last_seen_, track_start_) >= config_.warmup_time && variance >= 0 &&
         variance <= config_.max_omega_variance && omega >= config_.min_spin_speed &&
         omega <= config_.max_spin_speed;
}

const std::optional<Target> & OutpostTracker::target() const { return target_; }
int OutpostTracker::updates() const { return updates_; }
unsigned OutpostTracker::seen_mask() const { return seen_mask_; }
double OutpostTracker::last_nis() const { return last_nis_; }
}  // namespace auto_aim
