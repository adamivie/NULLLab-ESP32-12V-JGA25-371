// Host-side unit tests for lib/RobotCore (no hardware, no Arduino).
// Run with:  ./host_tests/run.sh
#include <initializer_list>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ControlProtocol.h"
#include "SkidSteer.h"
#include "SlewRateLimiter.h"
#include "StallDetector.h"
#include "VelocityEstimator.h"
#include "WheelSpeedController.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                    \
  do {                                                                 \
    ++g_checks;                                                        \
    if (!(cond)) {                                                     \
      ++g_failures;                                                    \
      printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
    }                                                                  \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                              \
  do {                                                                                     \
    ++g_checks;                                                                            \
    const double _a = (a), _b = (b);                                                       \
    if (!(fabs(_a - _b) <= (tol))) {                                                       \
      ++g_failures;                                                                        \
      printf("  FAIL %s:%d: %s = %g, expected %g (+/- %g)\n", __FILE__, __LINE__, #a, _a, _b, \
             (double)(tol));                                                               \
    }                                                                                      \
  } while (0)

// ---------------------------------------------------------------------------
static void testSlewRateLimiter() {
  puts("SlewRateLimiter");
  SlewRateLimiter s(1.0f, 2.0f);  // accel 1/s, decel 2/s

  for (int i = 0; i < 50; ++i) s.update(1.0f, 0.01f);
  CHECK_NEAR(s.value(), 0.5f, 1e-4);  // 0.5 s at 1/s
  for (int i = 0; i < 100; ++i) s.update(1.0f, 0.01f);
  CHECK_NEAR(s.value(), 1.0f, 1e-6);  // reaches but never passes target

  for (int i = 0; i < 25; ++i) s.update(0.0f, 0.01f);
  CHECK_NEAR(s.value(), 0.5f, 1e-4);  // decelerates at 2/s

  // Reversal: must pass through exactly zero before going negative.
  s.reset(0.3f);
  bool sawZero = false;
  float previous = s.value();
  for (int i = 0; i < 200; ++i) {
    const float v = s.update(-1.0f, 0.01f);
    if (v == 0.0f) sawZero = true;
    if (previous > 0.0f) CHECK(v >= 0.0f);  // never jumps straight from + to -
    previous = v;
  }
  CHECK(sawZero);
  CHECK_NEAR(s.value(), -1.0f, 1e-6);

  // Zero or negative dt changes nothing.
  s.reset(0.2f);
  CHECK_NEAR(s.update(1.0f, 0.0f), 0.2f, 1e-9);
  CHECK_NEAR(s.update(1.0f, -1.0f), 0.2f, 1e-9);
}

// ---------------------------------------------------------------------------
static void testVelocityEstimator() {
  puts("VelocityEstimator");
  VelocityEstimator<6> est;
  est.reset(1000, 0);
  // 2000 counts/s with 10 ms samples and +/-1 ms timing jitter.
  int64_t t = 0;
  for (int i = 1; i <= 50; ++i) {
    t = i * 10000 + ((i % 2) ? 1000 : -1000);
    est.update(1000 + (t * 2000) / 1000000, t);
  }
  CHECK_NEAR(est.velocity(), 2000.0f, 2000.0f * 0.02f);

  // Direction reversal is reported with the correct sign.
  const int64_t base = 1000 + (t * 2000) / 1000000;
  for (int i = 1; i <= 50; ++i) est.update(base - i * 30, t + i * 10000);
  CHECK_NEAR(est.velocity(), -3000.0f, 1.0f);

  // Standing still reads exactly zero.
  est.reset(42, 0);
  for (int i = 1; i <= 10; ++i) est.update(42, i * 10000);
  CHECK_NEAR(est.velocity(), 0.0f, 1e-9);
}

// ---------------------------------------------------------------------------
// Brushed-DC-motor + encoder model in normalised units, close to what the
// firmware sees on the robot:
//  - speed follows the effective duty with a first-order lag
//  - static friction and an external load eat part of the duty
//  - position is integrated in whole encoder counts (quantisation)
//  - speed is measured by the same VelocityEstimator the firmware uses
//  - the duty computed in one control period is applied in the next
struct MotorPlant {
  float tau = 0.08f;          // mechanical time constant, s
  float friction = 0.05f;     // duty needed to break away
  float load = 0.0f;          // extra duty consumed by load (carpet, slope)
  float gain = 1.0f;          // 1.0 = full battery; lower = sagging battery
  float maxCps = 4000.0f;     // counts/s at speed 1.0
  float speed = 0.0f;         // true normalised speed
  double position = 0.0;      // true position, counts

  void step(float duty, float dt) {
    const float drive = gain * duty;
    const float losses = friction + load;
    float effective;
    if (duty == 0.0f || fabsf(drive) <= losses) effective = 0.0f;  // brake / stiction
    else effective = drive > 0 ? drive - losses : drive + losses;
    speed += (effective - speed) * (dt / tau);
    position += speed * maxCps * dt;
  }
};

struct ClosedLoop {
  MotorPlant plant;
  WheelSpeedController ctrl;
  VelocityEstimator<6> estimator;
  float pendingDuty = 0.0f;
  int64_t timeUs = 0;

  ClosedLoop(const WheelGains& g, const MotorPlant& p) : plant(p), ctrl(g) { estimator.reset(0, 0); }

  void step(float target) {
    const float dt = 0.01f;
    timeUs += 10000;
    const float measured =
        estimator.update(static_cast<int64_t>(floor(plant.position)), timeUs) / plant.maxCps;
    const float duty = ctrl.update(target, measured, dt);
    plant.step(pendingDuty, dt);
    pendingDuty = duty;
  }
};

struct StepResult {
  float finalSpeed;
  float overshoot;    // fraction of target
  float settleTimeS;  // time after which speed stays within 5% of target
};

static StepResult simulateStep(const WheelGains& gains, const MotorPlant& plant, float target, float seconds) {
  ClosedLoop loop(gains, plant);
  const int steps = (int)(seconds / 0.01f);
  StepResult r = {0, 0, -1};
  float peak = 0;
  for (int i = 0; i < steps; ++i) {
    loop.step(target);
    const float s = loop.plant.speed;
    peak = fmaxf(peak, target > 0 ? s : -s);
    const bool inBand = fabsf(s - target) <= 0.05f * fabsf(target) + 0.003f;
    if (inBand && r.settleTimeS < 0) r.settleTimeS = (i + 1) * 0.01f;
    if (!inBand) r.settleTimeS = -1;
  }
  r.finalSpeed = loop.plant.speed;
  r.overshoot = (peak - fabsf(target)) / fabsf(target);
  return r;
}

static void testWheelSpeedController() {
  puts("WheelSpeedController");
  const WheelGains gains;  // firmware defaults

  // Nominal: reaches target, bounded overshoot, settles in about a second.
  const MotorPlant nominal;
  for (float target : {0.1f, 0.4f, 0.8f}) {
    const StepResult r = simulateStep(gains, nominal, target, 3.0f);
    CHECK_NEAR(r.finalSpeed, target, 0.03f * target + 0.003f);
    CHECK(r.overshoot < 0.25f);
    CHECK(r.settleTimeS > 0 && r.settleTimeS < 1.2f);
  }

  // Heavy load + sagging battery: the integrator must make up the difference.
  MotorPlant loaded;
  loaded.load = 0.15f;
  loaded.gain = 0.8f;
  StepResult r = simulateStep(gains, loaded, 0.4f, 4.0f);
  CHECK_NEAR(r.finalSpeed, 0.4f, 0.015f);
  CHECK(r.settleTimeS > 0 && r.settleTimeS < 2.0f);

  // Reverse direction works symmetrically.
  r = simulateStep(gains, nominal, -0.5f, 3.0f);
  CHECK_NEAR(r.finalSpeed, -0.5f, 0.015f);

  // Ramped target (what the robot actually does via SlewRateLimiter): with a
  // 1/s ramp, overshoot should be small.
  {
    ClosedLoop loop(gains, nominal);
    SlewRateLimiter ramp(1.0f, 2.0f);
    float peak = 0;
    for (int i = 0; i < 300; ++i) {
      loop.step(ramp.update(0.6f, 0.01f));
      peak = fmaxf(peak, loop.plant.speed);
    }
    CHECK(peak < 0.6f * 1.08f);
    CHECK_NEAR(loop.plant.speed, 0.6f, 0.02f);
  }

  // Unreachable target: output saturates but the integrator must stay within
  // its limit, so the wheel recovers promptly once the target is reachable.
  {
    MotorPlant weak;
    weak.gain = 0.75f;  // tired battery: tops out around 0.7
    ClosedLoop loop(gains, weak);
    for (int i = 0; i < 500; ++i) loop.step(1.0f);
    CHECK(fabsf(loop.ctrl.integral()) <= gains.iLimit + 1e-6f);
    for (int i = 0; i < 200; ++i) loop.step(0.4f);
    CHECK_NEAR(loop.plant.speed, 0.4f, 0.03f);
  }

  // Stop target -> exactly zero output (brake) and cleared integrator.
  {
    WheelSpeedController ctrl(gains);
    for (int i = 0; i < 50; ++i) ctrl.update(0.5f, 0.45f, 0.01f);
    CHECK(ctrl.integral() != 0.0f);
    CHECK(ctrl.update(0.0f, 0.3f, 0.01f) == 0.0f);
    CHECK(ctrl.integral() == 0.0f);
  }

  // Never drives backwards to slow down (gentle on gearbox).
  {
    WheelSpeedController ctrl(gains);
    CHECK(ctrl.update(0.2f, 0.9f, 0.01f) >= 0.0f);
    CHECK(ctrl.update(-0.2f, -0.9f, 0.01f) <= 0.0f);
  }

  // Garbage in -> safe output.
  {
    WheelSpeedController ctrl(gains);
    CHECK(ctrl.update(NAN, 0.0f, 0.01f) == 0.0f);
    CHECK(ctrl.update(0.5f, INFINITY, 0.01f) == 0.0f);
    const float d = ctrl.update(5.0f, 0.0f, 0.01f);
    CHECK(d <= 1.0f && d >= -1.0f);
  }
}

// ---------------------------------------------------------------------------
static void testStallDetector() {
  puts("StallDetector");
  StallDetector s(0.5f, 0.05f, 1.0f);
  for (int i = 0; i < 99; ++i) s.update(0.8f, 0.0f, 0.01f);
  CHECK(!s.stalled());
  s.update(0.8f, 0.0f, 0.011f);
  CHECK(s.stalled());

  s.reset();
  for (int i = 0; i < 200; ++i) s.update(0.8f, 0.5f, 0.01f);  // moving fine
  CHECK(!s.stalled());
  for (int i = 0; i < 200; ++i) s.update(0.3f, 0.0f, 0.01f);  // low effort, not a stall
  CHECK(!s.stalled());
  // A brief blip of motion restarts the timer.
  for (int i = 0; i < 90; ++i) s.update(-0.9f, 0.0f, 0.01f);
  s.update(-0.9f, -0.2f, 0.01f);
  for (int i = 0; i < 90; ++i) s.update(-0.9f, 0.0f, 0.01f);
  CHECK(!s.stalled());
}

// ---------------------------------------------------------------------------
static void testSkidSteer() {
  puts("SkidSteer");
  CHECK(shapeAxis(0.05f, 0.08f, 0.3f) == 0.0f);
  CHECK(shapeAxis(-0.05f, 0.08f, 0.3f) == 0.0f);
  CHECK_NEAR(shapeAxis(1.0f, 0.08f, 0.3f), 1.0f, 1e-6);
  CHECK_NEAR(shapeAxis(-1.0f, 0.08f, 0.3f), -1.0f, 1e-6);
  CHECK_NEAR(shapeAxis(0.0801f, 0.08f, 0.0f), 0.0f, 1e-3);  // continuous at edge
  CHECK_NEAR(shapeAxis(2.0f, 0.0f, 0.0f), 1.0f, 1e-6);       // clamped
  CHECK(shapeAxis(NAN, 0.0f, 0.0f) == 0.0f);
  // Monotonic.
  float prev = -2;
  for (int i = -100; i <= 100; ++i) {
    const float v = shapeAxis(i / 100.0f, 0.08f, 0.4f);
    CHECK(v >= prev);
    prev = v;
  }

  SideSpeeds s = mixArcade(1.0f, 0.0f, 0.7f);
  CHECK_NEAR(s.left, 1.0f, 1e-6);
  CHECK_NEAR(s.right, 1.0f, 1e-6);
  s = mixArcade(0.0f, 1.0f, 0.7f);  // spin right in place
  CHECK_NEAR(s.left, 0.7f, 1e-6);
  CHECK_NEAR(s.right, -0.7f, 1e-6);
  s = mixArcade(1.0f, 1.0f, 0.7f);  // saturates: scaled, ratio kept
  CHECK_NEAR(s.left, 1.0f, 1e-6);
  CHECK_NEAR(s.right, 0.3f / 1.7f, 1e-6);
  s = mixArcade(-0.5f, -0.5f, 1.0f);
  CHECK_NEAR(s.left, -1.0f, 1e-6);
  CHECK_NEAR(s.right, 0.0f, 1e-6);
  s = mixArcade(NAN, INFINITY, 1.0f);
  CHECK(s.left == 0.0f && s.right == 0.0f);
}

// ---------------------------------------------------------------------------
static Message parse(const char* s) { return parseMessage(s, strlen(s)); }

static void testControlProtocol() {
  puts("ControlProtocol");
  Message m = parse("D 0.25 -0.5");
  CHECK(m.type == MessageType::Drive);
  CHECK_NEAR(m.a, 0.25f, 1e-6);
  CHECK_NEAR(m.b, -0.5f, 1e-6);
  CHECK(parse("D -1 1").type == MessageType::Drive);

  CHECK(parse("A 1").type == MessageType::Arm && parse("A 1").a == 1.0f);
  CHECK(parse("A 0").type == MessageType::Arm && parse("A 0").a == 0.0f);
  CHECK(parse("L 0.6").type == MessageType::SpeedLimit);
  CHECK(parse("S").type == MessageType::Stop);
  CHECK(parse("C").type == MessageType::Claim);

  const char* bad[] = {
      "",          "D",          "D 0.1",     "D 0.1 0.2 0.3", "D 1.5 0",   "D 0 -1.01",
      "D nan 0",   "D 0 inf",    "D  0.1 0.2", "D 0.1  0.2",   "D0.1 0.2",  "D 0.1 0.2x",
      "A 2",       "A 0.5",      "A",          "L 0",           "L 1.1",     "L 0.05",
      "S 1",       "C x",        "X",          "d 0 0",         "D 0.1 0.2 ",
  };
  for (const char* b : bad) {
    const bool rejected = parse(b).type == MessageType::Invalid;
    if (!rejected) printf("  accepted bad message: '%s'\n", b);
    CHECK(rejected);
  }
  // Over-long and embedded-NUL input.
  char longMsg[100];
  memset(longMsg, '1', sizeof(longMsg));
  longMsg[0] = 'L';
  longMsg[1] = ' ';
  CHECK(parseMessage(longMsg, sizeof(longMsg)).type == MessageType::Invalid);
  const char withNul[] = {'S', '\0', 'x'};
  CHECK(parseMessage(withNul, 3).type == MessageType::Invalid);
  CHECK(parseMessage(nullptr, 3).type == MessageType::Invalid);
}

int main() {
  testSlewRateLimiter();
  testVelocityEstimator();
  testWheelSpeedController();
  testStallDetector();
  testSkidSteer();
  testControlProtocol();
  printf("\n%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
