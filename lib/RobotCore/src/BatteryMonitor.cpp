#include "BatteryMonitor.h"

#include <math.h>

namespace {
// Resting LiPo voltage vs. state of charge (per cell).
struct SocPoint {
  float volts;
  int percent;
};
constexpr SocPoint kLipoCurve[] = {
    {3.30f, 0},  {3.60f, 5},  {3.69f, 10}, {3.73f, 20}, {3.77f, 30}, {3.80f, 40},
    {3.84f, 50}, {3.87f, 60}, {3.95f, 70}, {4.02f, 80}, {4.11f, 90}, {4.20f, 100},
};
constexpr int kCurvePoints = sizeof(kLipoCurve) / sizeof(kLipoCurve[0]);
}  // namespace

BatteryMonitor::BatteryMonitor(const BatteryConfig& config) : config_(config) {
  if (config_.cells > 0) cells_ = config_.cells;
}

void BatteryMonitor::update(float volts, float amps, float dtS) {
  // A bad sample is skipped, not treated as "battery gone" (which would reset
  // the low/critical timers).
  if (!isfinite(volts) || !(dtS > 0.0f) || !isfinite(dtS)) return;
  if (!isfinite(amps)) amps = 0.0f;

  if (volts < config_.absentBelowV) {
    // Battery gone (or monitor unplugged). Keep the latched level and the
    // cell count: if the same pack comes back it is still just as flat.
    present_ = false;
    presentS_ = 0.0f;
    detectSum_ = 0.0f;
    detectCount_ = 0;
    belowLowS_ = belowCriticalS_ = 0.0f;
    amps_ = 0.0f;
    return;
  }

  if (!present_) {
    present_ = true;
    volts_ = volts;  // start the filter at the first reading, not at 0
    amps_ = amps;
  } else {
    const float alpha = dtS / (config_.filterTauS + dtS);
    volts_ += alpha * (volts - volts_);
    amps_ += alpha * (amps - amps_);
  }
  presentS_ += dtS;
  if (amps > 0.0f) mahUsed_ += amps * dtS * (1000.0f / 3600.0f);

  if (cells_ == 0) {
    // Average the first readings (before the robot drives, so no sag), then
    // pick the cell count that puts the pack at <= 4.25 V per cell.
    detectSum_ += volts;
    ++detectCount_;
    if (presentS_ < config_.detectSettleS) return;
    const float avg = detectSum_ / static_cast<float>(detectCount_);
    int n = static_cast<int>(ceilf(avg / 4.25f));
    if (n < config_.minCells) n = config_.minCells;
    if (n > config_.maxCells) n = config_.maxCells;
    cells_ = n;
  }

  const float perCell = volts_ / static_cast<float>(cells_);
  belowLowS_ = perCell < config_.cellLowV ? belowLowS_ + dtS : 0.0f;
  belowCriticalS_ = perCell < config_.cellCriticalV ? belowCriticalS_ + dtS : 0.0f;

  if (belowCriticalS_ >= config_.criticalDelayS) {
    latched_ = BatteryLevel::Critical;
  } else if (belowLowS_ >= config_.lowDelayS && latched_ == BatteryLevel::Ok) {
    latched_ = BatteryLevel::Low;
  }
}

BatteryLevel BatteryMonitor::level() const { return present_ ? latched_ : BatteryLevel::Absent; }

float BatteryMonitor::cellVolts() const {
  return (present_ && cells_ > 0) ? volts_ / static_cast<float>(cells_) : 0.0f;
}

int BatteryMonitor::percent() const {
  const float v = cellVolts();
  return v > 0.0f ? lipoPercent(v) : -1;
}

int BatteryMonitor::lipoPercent(float cellVolts) {
  if (!(cellVolts > kLipoCurve[0].volts)) return 0;
  if (cellVolts >= kLipoCurve[kCurvePoints - 1].volts) return 100;
  for (int i = 1; i < kCurvePoints; ++i) {
    if (cellVolts < kLipoCurve[i].volts) {
      const SocPoint& a = kLipoCurve[i - 1];
      const SocPoint& b = kLipoCurve[i];
      const float t = (cellVolts - a.volts) / (b.volts - a.volts);
      return static_cast<int>(lroundf(static_cast<float>(a.percent) + t * static_cast<float>(b.percent - a.percent)));
    }
  }
  return 100;
}
