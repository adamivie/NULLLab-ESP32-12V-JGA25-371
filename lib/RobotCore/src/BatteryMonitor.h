#pragma once

#include <stdint.h>

// Battery state from a voltage (and current) measurement - pure logic, no
// hardware, so it can be unit-tested on a PC.
//
// Protects a LiPo from being run flat:
//
//   ABSENT    no battery voltage (USB only, or the monitor is not wired)
//   OK
//   LOW       per-cell voltage below cellLowV for lowDelayS -> warn, cap speed
//   CRITICAL  per-cell voltage below cellCriticalV for criticalDelayS
//             -> stop and refuse to arm
//
// Motors pull the voltage down while they accelerate ("sag"), and it recovers
// once they stop. So the decision uses a low-pass filtered voltage AND requires
// it to stay below the threshold for a while; a hard launch cannot trip it.
// For the same reason LOW and CRITICAL latch until the battery is replaced
// (power cycle): a flat pack that rests for a minute reads healthy again, and
// letting it drive on would damage it.
enum class BatteryLevel : uint8_t { Absent, Ok, Low, Critical };

struct BatteryConfig {
  int cells = 0;                 // series cell count; 0 = detect from the first readings
  float cellLowV = 3.50f;        // per-cell, under load
  float cellCriticalV = 3.30f;   // per-cell, under load
  float lowDelayS = 5.0f;        // filtered voltage must stay low this long
  float criticalDelayS = 3.0f;
  float filterTauS = 1.5f;       // low-pass time constant for voltage and current
  float absentBelowV = 3.0f;     // pack voltage below this = no battery
  float detectSettleS = 0.5f;    // cell detection waits this long after connect
  int minCells = 2;              // the board needs >= 6 V
  int maxCells = 4;              // and tolerates <= 16 V
};

class BatteryMonitor {
 public:
  explicit BatteryMonitor(const BatteryConfig& config = BatteryConfig());

  // Feed one measurement (pack volts, amps) taken dtS seconds after the last.
  void update(float volts, float amps, float dtS);

  BatteryLevel level() const;
  bool present() const { return present_; }
  int cells() const { return cells_; }           // 0 until known
  float volts() const { return volts_; }         // filtered pack voltage
  float amps() const { return amps_; }           // filtered current
  float cellVolts() const;                       // filtered per-cell voltage, 0 if unknown
  float mahUsed() const { return mahUsed_; }     // since power-up
  int percent() const;                           // rough state of charge, -1 if unknown

  // Rough LiPo state of charge (0-100) for one cell's voltage. Under load this
  // reads low; it is a guide, not a fuel gauge.
  static int lipoPercent(float cellVolts);

 private:
  BatteryConfig config_;
  bool present_ = false;
  int cells_ = 0;
  float volts_ = 0.0f;
  float amps_ = 0.0f;
  float mahUsed_ = 0.0f;
  float presentS_ = 0.0f;     // time since the battery appeared
  float detectSum_ = 0.0f;    // raw voltage sum for cell detection
  int detectCount_ = 0;
  float belowLowS_ = 0.0f;
  float belowCriticalS_ = 0.0f;
  BatteryLevel latched_ = BatteryLevel::Ok;  // worst level seen (Ok/Low/Critical)
};
