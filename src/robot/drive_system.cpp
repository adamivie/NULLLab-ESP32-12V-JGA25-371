#include "drive_system.h"

#include <Arduino.h>
#include <esp_timer.h>
#include <math.h>

#include "ControlProtocol.h"
#include "SkidSteer.h"

namespace {
constexpr int kN = config::kWheelCount;
constexpr float kStoppedSpeed = 0.02f;  // normalised; "wheel has stopped"

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

DriveSystem::DriveSystem()
    : stallDetectors_{{config::kStallDuty, config::kStallSpeed, config::kStallTimeS},
                      {config::kStallDuty, config::kStallSpeed, config::kStallTimeS},
                      {config::kStallDuty, config::kStallSpeed, config::kStallTimeS},
                      {config::kStallDuty, config::kStallSpeed, config::kStallTimeS}},
      rampLeft_(config::kAccelPerS, config::kDecelPerS),
      rampRight_(config::kAccelPerS, config::kDecelPerS) {
  for (auto& c : controllers_) c.setGains(config::wheelGains());
  status_.state = DriveState::Disarmed;
  status_.faultWheel = -1;
  status_.speedLimit = config::kDefaultSpeedLimit;
}

bool DriveSystem::begin() {
  hardwareOk_ = wheels_.begin();
  const int64_t now = esp_timer_get_time();
  for (int i = 0; i < kN; ++i) estimators_[i].reset(wheels_.encoder(i).count(), now);
  status_.hardwareOk = hardwareOk_;

  // Core 1 (Arduino side). Priority above loop() (1) and the web server task
  // (10); the loop needs well under 1 ms per period so it cannot starve them.
  // Wi-Fi and AsyncTCP are pinned to core 0.
  constexpr UBaseType_t kControlTaskPriority = 12;
  const BaseType_t created = xTaskCreatePinnedToCore(taskEntry, "drive", 4096, this, kControlTaskPriority, &task_, 1);
  return hardwareOk_ && created == pdPASS;
}

void DriveSystem::setJoystick(float turn, float throttle) {
  if (!isfinite(turn) || !isfinite(throttle)) return;
  const uint32_t now = millis();
  portENTER_CRITICAL(&lock_);
  stickTurn_ = clampf(turn, -1.0f, 1.0f);
  stickThrottle_ = clampf(throttle, -1.0f, 1.0f);
  lastCommandMs_ = now;
  everCommanded_ = true;
  portEXIT_CRITICAL(&lock_);
}

void DriveSystem::setSpeedLimit(float limit) {
  if (!isfinite(limit)) return;
  portENTER_CRITICAL(&lock_);
  speedLimit_ = clampf(limit, kMinSpeedLimit, 1.0f);
  portEXIT_CRITICAL(&lock_);
}

bool DriveSystem::stickCentredLocked() const {
  return shapeAxis(stickTurn_, config::kStickDeadzone, 0.0f) == 0.0f &&
         shapeAxis(stickThrottle_, config::kStickDeadzone, 0.0f) == 0.0f;
}

ArmResult DriveSystem::arm() {
  const uint32_t now = millis();
  ArmResult result = ArmResult::Ok;
  portENTER_CRITICAL(&lock_);
  if (!hardwareOk_) {
    result = ArmResult::HardwareError;  // never drive half-initialised hardware
  } else if (state_ == DriveState::Fault) {
    result = ArmResult::Faulted;
  } else if (!everCommanded_ || now - lastCommandMs_ > config::kCommandTimeoutMs) {
    result = ArmResult::NoLink;
  } else if (!stickCentredLocked()) {
    result = ArmResult::NotCentred;  // never lurch off when arming
  } else if (state_ != DriveState::Armed) {
    state_ = DriveState::Armed;
    brakeNow_ = false;
  }
  portEXIT_CRITICAL(&lock_);
  return result;
}

void DriveSystem::softStop() {
  portENTER_CRITICAL(&lock_);
  if (state_ == DriveState::Armed) state_ = DriveState::Stopping;
  else if (state_ == DriveState::Fault) state_ = DriveState::Disarmed;  // acknowledge fault
  portEXIT_CRITICAL(&lock_);
}

void DriveSystem::emergencyStop() {
  portENTER_CRITICAL(&lock_);
  if (state_ != DriveState::Fault) state_ = DriveState::Disarmed;
  brakeNow_ = true;
  portEXIT_CRITICAL(&lock_);
  // Only the control task touches the motors (no cross-task hardware
  // access); it applies the brake within one 10 ms period.
}

DriveStatus DriveSystem::status() const {
  portENTER_CRITICAL(&lock_);
  const DriveStatus copy = status_;
  portEXIT_CRITICAL(&lock_);
  return copy;
}

void DriveSystem::taskEntry(void* self) { static_cast<DriveSystem*>(self)->runLoop(); }

void DriveSystem::runLoop() {
  const TickType_t period = pdMS_TO_TICKS(config::kControlPeriodMs);
  TickType_t wake = xTaskGetTickCount();
  int64_t lastUs = esp_timer_get_time();
  for (;;) {
    vTaskDelayUntil(&wake, period);
    const int64_t startUs = esp_timer_get_time();
    // Use the real elapsed time; clamp so a long stall (e.g. flash write)
    // cannot produce a huge integrator step.
    const float dtS = clampf(static_cast<float>(startUs - lastUs) * 1e-6f, 0.001f, 0.05f);
    const bool overrun = (startUs - lastUs) > static_cast<int64_t>(config::kControlPeriodMs) * 1500;
    lastUs = startUs;

    controlStep(dtS, startUs, millis());

    const uint32_t computeUs = static_cast<uint32_t>(esp_timer_get_time() - startUs);
    portENTER_CRITICAL(&lock_);
    if (overrun) ++status_.loopOverruns;
    if (computeUs > status_.maxLoopUs) status_.maxLoopUs = computeUs;
    portEXIT_CRITICAL(&lock_);
  }
}

void DriveSystem::controlStep(float dtS, int64_t nowUs, uint32_t nowMs) {
  // --- snapshot shared inputs ------------------------------------------------
  portENTER_CRITICAL(&lock_);
  const uint32_t sinceCommandMs = nowMs - lastCommandMs_;
  const bool linkOk = everCommanded_ && sinceCommandMs <= config::kCommandTimeoutMs;
  if (!everCommanded_ || sinceCommandMs > config::kLinkLostDisarmMs) {
    if (state_ == DriveState::Armed) state_ = DriveState::Stopping;
  }
  DriveState state = state_;
  const float turn = stickTurn_;
  const float throttle = stickThrottle_;
  const float limit = speedLimit_;
  const bool brakeNow = brakeNow_;
  brakeNow_ = false;
  portEXIT_CRITICAL(&lock_);

  // --- desired side speeds (normalised) --------------------------------------
  SideSpeeds desired = {0.0f, 0.0f};
  if (state == DriveState::Armed && linkOk) {
    const float t = shapeAxis(turn, config::kStickDeadzone, config::kStickExpo);
    const float th = shapeAxis(throttle, config::kStickDeadzone, config::kStickExpo);
    desired = mixArcade(th, t, config::kTurnGain);
    desired.left *= limit;
    desired.right *= limit;
  }

  const bool driving = state == DriveState::Armed || state == DriveState::Stopping;
  if (!driving || brakeNow) {
    rampLeft_.reset();
    rampRight_.reset();
  }
  const float left = rampLeft_.update(desired.left, dtS);
  const float right = rampRight_.update(desired.right, dtS);

  // --- per-wheel speed control -------------------------------------------------
  WheelTelemetry telemetry[kN];
  int stalledWheel = -1;
  bool allStopped = true;
  for (int i = 0; i < kN; ++i) {
    const float measured =
        estimators_[i].update(wheels_.encoder(i).count(), nowUs) / config::kMaxWheelSpeedCps;
    const float target = config::kWheels[i].isLeft ? left : right;
    if (fabsf(measured) > kStoppedSpeed) allStopped = false;

    float duty = 0.0f;
    if (driving && !brakeNow) {
      duty = controllers_[i].update(target, measured, dtS);
      if (stallDetectors_[i].update(duty, measured, dtS) && stalledWheel < 0) stalledWheel = i;
    } else {
      controllers_[i].reset();
      stallDetectors_[i].reset();
    }
    wheels_.motor(i).setDuty(duty);  // 0 = brake
    telemetry[i] = {target * config::kMaxWheelSpeedMps, measured * config::kMaxWheelSpeedMps, duty};
  }

  if (stalledWheel >= 0) {
    wheels_.brakeAll();
    rampLeft_.reset();
    rampRight_.reset();
    for (auto& c : controllers_) c.reset();
    for (auto& s : stallDetectors_) s.reset();
  }

  // --- state transitions & telemetry ------------------------------------------
  portENTER_CRITICAL(&lock_);
  if (stalledWheel >= 0 && (state_ == DriveState::Armed || state_ == DriveState::Stopping)) {
    state_ = DriveState::Fault;
    status_.faultWheel = stalledWheel;
  } else if (state_ == DriveState::Stopping && rampLeft_.value() == 0.0f && rampRight_.value() == 0.0f &&
             allStopped) {
    state_ = DriveState::Disarmed;
  }
  if (state_ != DriveState::Fault) status_.faultWheel = -1;
  status_.state = state_;
  status_.linkOk = linkOk;
  status_.speedLimit = limit;
  status_.hardwareOk = hardwareOk_;
  for (int i = 0; i < kN; ++i) status_.wheels[i] = telemetry[i];
  portEXIT_CRITICAL(&lock_);
}
