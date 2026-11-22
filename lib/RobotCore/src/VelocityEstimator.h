#pragma once

#include <stddef.h>
#include <stdint.h>

// Estimates speed from an encoder count using a sliding time window.
//
// At low speed a wheel produces only a few counts per 10 ms control period, so
// differencing consecutive samples is very noisy. Differencing against the
// sample N periods ago averages over a longer window while still using exact
// timestamps, so jitter in the control loop does not bias the result.
template <size_t N>
class VelocityEstimator {
  static_assert(N >= 2, "window needs at least two samples");

 public:
  // Fill the whole window with one sample (call at start-up and after any
  // discontinuity such as an encoder reset).
  void reset(int64_t count, int64_t timeUs) {
    for (size_t i = 0; i < N; ++i) samples_[i] = {count, timeUs};
    head_ = 0;
    velocity_ = 0.0f;
  }

  // Returns counts per second.
  float update(int64_t count, int64_t timeUs) {
    head_ = (head_ + 1) % N;
    samples_[head_] = {count, timeUs};
    const Sample& oldest = samples_[(head_ + 1) % N];
    const int64_t dtUs = timeUs - oldest.timeUs;
    if (dtUs > 0) {
      velocity_ = static_cast<float>(count - oldest.count) * 1.0e6f / static_cast<float>(dtUs);
    }
    return velocity_;
  }

  float velocity() const { return velocity_; }

 private:
  struct Sample {
    int64_t count;
    int64_t timeUs;
  };
  Sample samples_[N] = {};
  size_t head_ = 0;
  float velocity_ = 0.0f;
};
