#ifndef TASK_2__FIRE_HPP
#define TASK_2__FIRE_HPP

#include <cmath>

namespace auto_aim
{
class SingleShot
{
public:
  // shot_limit=0表示不由程序限制总数。正数为本次进程的请求总上限。
  explicit SingleShot(int shot_limit = 0) : shot_limit_(shot_limit) {}

  bool request(bool ready, double time)
  {
    // 两次fire=true之间至少有一轮false；时间节流保证不超过5Hz。
    if (last_was_fire_) {
      last_was_fire_ = false;
      return false;
    }
    if (
      !ready || !std::isfinite(time) || time < 0 || (shot_limit_ > 0 && shots_ >= shot_limit_) ||
      (shots_ > 0 && time - last_fire_time_ < 0.2 - 1e-9)) {
      return false;
    }
    last_fire_time_ = time;
    last_was_fire_ = true;
    ++shots_;
    return true;
  }

  int shots() const { return shots_; }

private:
  int shot_limit_;
  int shots_ = 0;
  double last_fire_time_ = 0;
  bool last_was_fire_ = false;
};
}  // namespace auto_aim

#endif  // TASK_2__FIRE_HPP
