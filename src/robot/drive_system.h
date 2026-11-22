#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>

#include "../common/wheel_set.h"
#include "SlewRateLimiter.h"
#include "StallDetector.h"
#include "VelocityEstimator.h"
#include "WheelSpeedController.h"
#include "robot_config.h"

// Owns the drivetrain and runs the 100 Hz speed loop in its own FreeRTOS task
// (core 1, away from Wi-Fi on core 0). All public methods are thread-safe and
// may be called from the web server task.
//
// State machine:
//
//   DISARMED --arm()--> ARMED --softStop()/link lost--> STOPPING --> DISARMED
//       ^                 |                                 |
//       |                 +--emergencyStop()----------------+--> DISARMED (brake now)
//       |                 +--wheel stalls--> FAULT
//       +----disarm()/softStop()---------------------------------------+
//
// Motors are actively braked whenever the robot is not ARMED or STOPPING.
enum class DriveState : uint8_t { Disarmed, Armed, Stopping, Fault };
enum class ArmResult : uint8_t { Ok, NotCentred, NoLink, Faulted, HardwareError };

struct WheelTelemetry {
  float targetMps;
  float speedMps;
  float duty;
};

struct DriveStatus {
  DriveState state;
  int faultWheel;  // index of the stalled wheel, -1 if none
  bool linkOk;     // joystick messages arriving
  float speedLimit;
  WheelTelemetry wheels[config::kWheelCount];
  uint32_t loopOverruns;  // control periods that ran late (> 1.5x period)
  uint32_t maxLoopUs;     // worst-case control computation time
  bool hardwareOk;
};

class DriveSystem {
 public:
  DriveSystem();

  // Initialises the hardware and starts the control task.
  bool begin();

  // Raw joystick, both in [-1, 1]; turn > 0 = right, throttle > 0 = forward.
  // Also acts as the heartbeat: the robot stops if these stop arriving.
  void setJoystick(float turn, float throttle);
  void setSpeedLimit(float limit);  // [kMinSpeedLimit, 1]

  ArmResult arm();
  void softStop();       // ramp to zero, then disarm; also clears a fault
  void emergencyStop();  // brake immediately and disarm

  DriveStatus status() const;

 private:
  static void taskEntry(void* self);
  void runLoop();
  void controlStep(float dtS, int64_t nowUs, uint32_t nowMs);
  bool stickCentredLocked() const;

  WheelSet wheels_;
  WheelSpeedController controllers_[config::kWheelCount];
  VelocityEstimator<6> estimators_[config::kWheelCount];
  StallDetector stallDetectors_[config::kWheelCount];
  SlewRateLimiter rampLeft_;
  SlewRateLimiter rampRight_;
  TaskHandle_t task_ = nullptr;
  bool hardwareOk_ = false;

  // Shared between tasks; guarded by lock_.
  mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
  float stickTurn_ = 0.0f;
  float stickThrottle_ = 0.0f;
  uint32_t lastCommandMs_ = 0;
  bool everCommanded_ = false;
  float speedLimit_ = config::kDefaultSpeedLimit;
  DriveState state_ = DriveState::Disarmed;
  bool brakeNow_ = false;  // set by emergencyStop(), consumed by the loop
  DriveStatus status_ = {};
};
