#pragma once

#include <chrono>
#include <cmath>
#include <unordered_map>

#include "util.hpp"

namespace util {
class Timer {
 private:
  typedef std::chrono::time_point<std::chrono::system_clock> TimePoint;
  typedef std::chrono::milliseconds MS;
  typedef std::chrono::seconds Sec;

  Timer const* parent_timer_;
  bool time_limit_enabled_;
  TimePoint time_limit_;
  TimePoint started_;
  int effective_time_limit_;

 public:
  Timer(int time_limit_sec = 0, Timer const* parent_timer = nullptr)
      : parent_timer_(parent_timer), started_(std::chrono::system_clock::now()), effective_time_limit_(-1) {
    set_time_limit(time_limit_sec);
  }

  /**
   * @brief Sets the time limit to the given duration.
   *
   * @param time_limit_sec time limit
   * @return int Effective time limit; -1 if no time limit, duration in seconds otherwise.
   *             The value may be shorter than the given value if the parent timer ends earlier.
   */
  int set_time_limit(int time_limit_sec) {
    auto parent_remain = parent_timer_ ? parent_timer_->get_remain_sec() : INFINITY;
    auto time_limit = time_limit_sec > 0 ? static_cast<double>(time_limit_sec) : INFINITY;

    time_limit_enabled_ = !std::isinf(parent_remain) || !std::isinf(time_limit);
    if (!time_limit_enabled_) return -1;  // no time limit

    effective_time_limit_ = static_cast<int>(std::floor(std::min(time_limit, parent_remain)));
    time_limit_ = started_ + Sec(effective_time_limit_);

    return effective_time_limit_;
  }

  int get_effective_time_limit() const { return effective_time_limit_; }

  double get_remain_sec() const {
    if (!time_limit_enabled_) return INFINITY;  // unbounded

    auto now = std::chrono::system_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<MS>(time_limit_ - now).count();
    return std::max(0.0, static_cast<double>(elapsed_ms) / 1000.0);
  }

  double stop() const {
    auto now = std::chrono::system_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<MS>(now - started_).count();
    return static_cast<double>(elapsed_ms) / 1000.0;
  }

  bool is_time_over() const {
    if (!time_limit_enabled_) return false;
    return std::chrono::system_clock::now() >= time_limit_;
  }
};
}  // namespace util
