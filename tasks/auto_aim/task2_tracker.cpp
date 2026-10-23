#include "task2_tracker.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "tools/math_tools.hpp"

namespace auto_aim
{
Task2Tracker::Task2Tracker(const Task2TrackConfig & config) : config_(config)
{
  const std::vector<double> positive{config.initial_radius,         config.lost_time,
                                     config.association_distance,   config.association_angle,
                                     config.max_observation_facing, config.max_omega_variance,
                                     config.min_spin_speed,         config.max_spin_speed};
  for (auto value : positive) {
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("Invalid Task2 tracker config.");
  }
  if (
    (config.armor_num != 2 && config.armor_num != 4) || config.confirm_frames < 1 ||
    config.min_updates < 2 || config.initial_radius <= 0.05 || config.initial_radius >= 0.5 ||
    !std::isfinite(config.warmup_time) || config.warmup_time < 0 ||
    config.association_angle >= 1.5707963267948966 ||
    config.max_observation_facing >= 1.5707963267948966 ||
    config.max_spin_speed <= config.min_spin_speed)
    throw std::invalid_argument("Invalid Task2 tracker range.");
}

bool Task2Tracker::valid(const Task2Observation & observation) const
{
  const auto & p = observation.xyza;
  if (
    !p.allFinite() || !std::isfinite(observation.selection_score) || p.head<3>().norm() < 0.1 ||
    p.head<3>().norm() > 20 || observation.name < ArmorName::one ||
    observation.name > ArmorName::sentry ||
    (observation.type != ArmorType::small && observation.type != ArmorType::big))
    return false;
  return std::abs(std::remainder(p[3] - std::atan2(p[1], p[0]), 2 * CV_PI)) <=
         config_.max_observation_facing;
}

bool Task2Tracker::update(const std::vector<Task2Observation> & observations, Time t)
{
  if (has_input_time_ && t <= last_input_) return false;
  expire(t);
  last_input_ = t;
  has_input_time_ = true;
  if (target_) target_->predict(t);
  if (target_ && target_->diverged()) reset();

  const Task2Observation * best = nullptr;
  int best_id = -1;
  double best_score = std::numeric_limits<double>::infinity();
  for (const auto & observation : observations) {
    if (!valid(observation)) continue;
    if (!target_) {
      if (observation.selection_score < best_score) {
        best = &observation;
        best_score = observation.selection_score;
      }
      continue;
    }
    if (observation.name != target_->name || observation.type != target_->armor_type) continue;
    auto predicted = target_->armor_xyza_list();
    for (int id = 0; id < static_cast<int>(predicted.size()); ++id) {
      if (
        (observation.xyza.head<3>() - predicted[id].head<3>()).norm() >
          config_.association_distance ||
        std::abs(std::remainder(observation.xyza[3] - predicted[id][3], 2 * CV_PI)) >
          config_.association_angle)
        continue;
      auto score = target_->innovation_distance(observation.xyza, id);
      if (score < best_score) {
        best = &observation;
        best_id = id;
        best_score = score;
      }
    }
  }
  if (!best) return false;
  if (!target_) {
    bool same =
      pending_ && pending_->name == best->name && pending_->type == best->type &&
      (pending_->xyza.head<3>() - best->xyza.head<3>()).norm() < config_.association_distance &&
      std::abs(std::remainder(pending_->xyza[3] - best->xyza[3], 2 * CV_PI)) <
        config_.association_angle;
    confirmations_ = same ? confirmations_ + 1 : 1;
    pending_ = *best;
    pending_time_ = t;
    if (confirmations_ < config_.confirm_frames) return false;
    target_.emplace(
      best->xyza, best->name, best->type, t, config_.initial_radius, config_.armor_num,
      config_.filter);
    pending_.reset();
    updates_ = 1;
    seen_mask_ = 1;
    last_nis_ = 0;
    track_start_ = last_seen_ = t;
    ++track_id_;
    return true;
  }
  // 使用同一个(observation,id)做门限判断和融合，避免主程序与Target各选一次板。
  if (!target_->try_update(best->xyza, best_id)) return false;
  if (target_->diverged()) {
    reset();
    return false;
  }
  last_nis_ = target_->ekf().last_nis;
  ++updates_;
  seen_mask_ |= 1u << best_id;
  last_seen_ = t;
  return true;
}

void Task2Tracker::expire(Time now)
{
  if (
    (target_ && tools::delta_time(now, last_seen_) > config_.lost_time) ||
    (pending_ && tools::delta_time(now, pending_time_) > config_.lost_time))
    reset();
}

void Task2Tracker::reset()
{
  target_.reset();
  pending_.reset();
  updates_ = confirmations_ = 0;
  seen_mask_ = 0;
  last_nis_ = 0;
}

bool Task2Tracker::ready() const
{
  if (
    !target_ || target_->diverged() || updates_ < config_.min_updates ||
    tools::delta_time(last_seen_, track_start_) < config_.warmup_time ||
    (seen_mask_ & (seen_mask_ - 1)) == 0)
    return false;
  auto omega = std::abs(target_->ekf_x()[7]);
  auto variance = target_->ekf().P(7, 7);
  return variance >= 0 && variance <= config_.max_omega_variance &&
         omega >= config_.min_spin_speed && omega <= config_.max_spin_speed;
}

const std::optional<Target> & Task2Tracker::target() const { return target_; }
int Task2Tracker::updates() const { return updates_; }
unsigned Task2Tracker::seen_mask() const { return seen_mask_; }
unsigned Task2Tracker::track_id() const { return track_id_; }
double Task2Tracker::last_nis() const { return last_nis_; }
}  // namespace auto_aim
