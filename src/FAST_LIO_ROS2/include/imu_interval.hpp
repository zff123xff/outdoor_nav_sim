#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <string>

namespace fast_lio {

// The caller owns the queue lock. Future measurements remain in the queue.
template<class Queue, class Stamp>
std::deque<typename Queue::value_type> take_imu_until(Queue &queue, double end, Stamp stamp) {
  std::deque<typename Queue::value_type> result;
  while (!queue.empty() && stamp(queue.front()) <= end) {
    result.push_back(queue.front());
    queue.pop_front();
  }
  return result;
}

struct ImuCoverage {
  bool valid = false;
  double max_gap = 0.;
  std::string reason;
};

// Includes the saved boundary sample and the final extrapolation to scan end.
template<class Samples, class Stamp>
ImuCoverage check_imu_coverage(const Samples &samples, double previous_sample,
                              double state_stamp, double end, double limit, Stamp stamp) {
  ImuCoverage result;
  if (samples.empty()) { result.reason = "empty_imu"; return result; }
  if (!std::isfinite(previous_sample) || !std::isfinite(state_stamp) ||
      !std::isfinite(end) || !std::isfinite(limit) || limit <= 0. || end <= state_stamp) {
    result.reason = "invalid_state_interval"; return result;
  }
  double previous = previous_sample;
  for (const auto &sample : samples) {
    const double time = stamp(sample);
    if (!std::isfinite(time) || time <= previous || time > end) {
      result.reason = "invalid_imu_order"; return result;
    }
    result.max_gap = std::max(result.max_gap, time-previous);
    previous = time;
  }
  result.max_gap = std::max(result.max_gap, end-previous);
  if (result.max_gap > limit+1e-9) { result.reason = "imu_gap"; return result; }
  result.valid = true;
  result.reason = "ready";
  return result;
}

}  // namespace fast_lio
