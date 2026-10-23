#ifndef TOOLS__QUATERNION_HISTORY_HPP
#define TOOLS__QUATERNION_HISTORY_HPP

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <iterator>
#include <mutex>

namespace tools
{
class QuaternionHistory
{
public:
  using Clock = std::chrono::steady_clock;

  bool push(const Eigen::Quaterniond & q, Clock::time_point t)
  {
    if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-6) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!samples_.empty() && t <= samples_.back().t) return false;
    samples_.push_back({q.normalized(), t});
    while (samples_.size() > 256) samples_.pop_front();
    changed_.notify_all();
    return true;
  }

  bool wait_for_data(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return changed_.wait_for(lock, timeout, [this] { return !samples_.empty(); });
  }

  bool sample(
    Clock::time_point t, Eigen::Quaterniond & q, std::chrono::milliseconds timeout,
    double max_gap = 0.05)
  {
    if (timeout.count() < 0 || !std::isfinite(max_gap) || max_gap <= 0) return false;
    std::unique_lock<std::mutex> lock(mutex_);
    auto generation = generation_;
    if (!changed_.wait_for(lock, timeout, [&] {
          return generation != generation_ || (!samples_.empty() && samples_.back().t >= t);
        }))
      return false;
    if (generation != generation_ || samples_.empty() || t < samples_.front().t) return false;
    auto upper = std::lower_bound(
      samples_.begin(), samples_.end(), t,
      [](const Sample & a, Clock::time_point stamp) { return a.t < stamp; });
    if (upper == samples_.end()) return false;
    if (upper->t == t) {
      q = upper->q;
      return true;
    }
    if (upper == samples_.begin()) return false;
    auto lower = std::prev(upper);
    auto span = std::chrono::duration<double>(upper->t - lower->t).count();
    if (span <= 0 || span > max_gap) return false;
    auto fraction = std::chrono::duration<double>(t - lower->t).count() / span;
    q = lower->q.slerp(fraction, upper->q).normalized();
    return q.coeffs().allFinite();
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.clear();
    ++generation_;
    changed_.notify_all();
  }

private:
  struct Sample
  {
    Eigen::Quaterniond q;
    Clock::time_point t;
  };
  std::deque<Sample> samples_;
  unsigned generation_ = 0;
  std::mutex mutex_;
  std::condition_variable changed_;
};
}  // namespace tools

#endif  // TOOLS__QUATERNION_HISTORY_HPP
