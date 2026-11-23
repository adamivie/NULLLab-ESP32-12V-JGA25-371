// Host-side unit tests for lib/RobotCore (no hardware, no Arduino).
// Run with:  ./host_tests/run.sh
#include <initializer_list>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "BatteryMonitor.h"
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

// ---------------------------------------------------------------------------
// Feed a constant reading for `seconds` at 50 Hz.
static void feed(BatteryMonitor& b, float volts, float amps, float seconds) {
  for (int i = 0, n = (int)lroundf(seconds / 0.02f); i < n; ++i) b.update(volts, amps, 0.02f);
}

static void testBatteryMonitor() {
  puts("BatteryMonitor");

  // No battery -> Absent, nothing known.
  {
    BatteryMonitor b;
    feed(b, 0.05f, 0.0f, 2.0f);
    CHECK(b.level() == BatteryLevel::Absent);
    CHECK(!b.present());
    CHECK(b.cells() == 0);
    CHECK(b.percent() == -1);
  }

  // Cell-count detection over the useful range of each pack.
  {
    struct Case { float v; int cells; };
    const Case cases[] = {{8.40f, 2}, {7.0f, 2}, {12.6f, 3}, {11.1f, 3}, {9.9f, 3}, {9.0f, 3},
                          {16.8f, 4}, {14.8f, 4}, {13.2f, 4}};
    for (const Case& c : cases) {
      BatteryMonitor b;
      feed(b, c.v, 0.0f, 1.0f);
      if (b.cells() != c.cells) printf("  %.1f V -> %d cells, expected %d\n", c.v, b.cells(), c.cells);
      CHECK(b.cells() == c.cells);
    }
    BatteryMonitor early;
    feed(early, 12.0f, 0.0f, 0.2f);
    CHECK(early.cells() == 0);  // still settling
    CHECK(early.level() == BatteryLevel::Ok);
    BatteryConfig fixed;
    fixed.cells = 3;
    BatteryMonitor f(fixed);
    CHECK(f.cells() == 3);
    feed(f, 8.4f, 0.0f, 1.0f);   // a configured count is never overridden
    CHECK(f.cells() == 3);
  }

  // Filter starts at the first reading, then follows.
  {
    BatteryMonitor b;
    b.update(12.0f, 1.0f, 0.02f);
    CHECK_NEAR(b.volts(), 12.0f, 1e-5);
    feed(b, 11.0f, 1.0f, 10.0f);   // >> tau
    CHECK_NEAR(b.volts(), 11.0f, 0.01f);
  }

  // A full pack: hard launches sag the voltage briefly - must stay OK.
  {
    BatteryMonitor b;
    feed(b, 12.4f, 0.3f, 1.0f);
    for (int k = 0; k < 20; ++k) {
      feed(b, 9.3f, 8.0f, 0.4f);   // 3.1 V/cell for 0.4 s
      feed(b, 12.2f, 1.0f, 2.0f);
    }
    CHECK(b.level() == BatteryLevel::Ok);
  }

  // Simulated 3S LiPo draining under load (internal resistance 0.15 ohm):
  // LOW must come before CRITICAL, both latch, and resting voltage recovering
  // afterwards must not clear them.
  {
    BatteryMonitor b;
    feed(b, 12.5f, 0.2f, 1.0f);
    CHECK(b.cells() == 3);
    float restV = 12.5f, t = 0.0f, tLow = -1.0f, tCrit = -1.0f;
    while (t < 2000.0f && tCrit < 0.0f) {
      const float amps = 3.0f + 2.0f * sinf(t);          // varying driving load
      b.update(restV - 0.15f * amps, amps, 0.02f);
      restV -= 0.02f * 0.002f;                           // 2 mV/s drain
      t += 0.02f;
      if (tLow < 0.0f && b.level() == BatteryLevel::Low) tLow = t;
      if (b.level() == BatteryLevel::Critical) tCrit = t;
    }
    CHECK(tLow > 0.0f);
    CHECK(tCrit > tLow);
    feed(b, 11.4f, 0.1f, 30.0f);                         // rested: 3.8 V/cell
    CHECK(b.level() == BatteryLevel::Critical);          // still latched
    feed(b, 0.0f, 0.0f, 1.0f);
    CHECK(b.level() == BatteryLevel::Absent);
    feed(b, 11.4f, 0.1f, 1.0f);                          // same pack reconnected
    CHECK(b.level() == BatteryLevel::Critical);
  }

  // LOW needs to persist for lowDelayS (filtered).
  {
    BatteryMonitor b;
    feed(b, 11.0f, 1.0f, 1.0f);                          // 3.67 V/cell
    feed(b, 10.35f, 1.0f, 3.0f);                         // 3.45: filtered dips, < 5 s
    feed(b, 11.0f, 1.0f, 3.0f);
    CHECK(b.level() == BatteryLevel::Ok);
    feed(b, 10.35f, 1.0f, 12.0f);
    CHECK(b.level() == BatteryLevel::Low);
    feed(b, 11.5f, 1.0f, 10.0f);
    CHECK(b.level() == BatteryLevel::Low);               // latched
  }

  // Charge used: 2 A for 30 min = 1000 mAh; negative current is ignored.
  {
    BatteryMonitor b;
    feed(b, 12.0f, 2.0f, 1800.0f);
    CHECK_NEAR(b.mahUsed(), 1000.0f, 1.0f);
    feed(b, 12.0f, -1.0f, 60.0f);
    CHECK_NEAR(b.mahUsed(), 1000.0f, 1.0f);
  }

  // State of charge curve: endpoints, monotonic, sane mid-point.
  CHECK(BatteryMonitor::lipoPercent(3.0f) == 0);
  CHECK(BatteryMonitor::lipoPercent(4.25f) == 100);
  CHECK(BatteryMonitor::lipoPercent(NAN) == 0);
  CHECK(BatteryMonitor::lipoPercent(3.84f) == 50);
  int previous = -1;
  for (float v = 3.2f; v <= 4.3f; v += 0.005f) {
    const int p = BatteryMonitor::lipoPercent(v);
    CHECK(p >= previous);
    previous = p;
  }

  // Garbage input never produces NaN state.
  {
    BatteryMonitor b;
    feed(b, 12.0f, 1.0f, 1.0f);
    b.update(NAN, INFINITY, 0.02f);
    b.update(12.0f, 1.0f, NAN);
    b.update(12.0f, 1.0f, -1.0f);
    CHECK(isfinite(b.volts()) && isfinite(b.amps()) && isfinite(b.mahUsed()));
    CHECK(b.present());  // a NaN sample is not "battery unplugged"
    CHECK_NEAR(b.volts(), 12.0f, 1e-4);
  }
}

int main() {
  testSlewRateLimiter();
  testVelocityEstimator();
  testWheelSpeedController();
  testStallDetector();
  testSkidSteer();
  testControlProtocol();
  testBatteryMonitor();
  printf("\n%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
