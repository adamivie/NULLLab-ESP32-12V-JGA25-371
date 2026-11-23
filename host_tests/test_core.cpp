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

  // Ping.
  m = parse("P 42");
  CHECK(m.type == MessageType::Ping);
  CHECK_NEAR(m.a, 42.0f, 1e-6f);
  for (const char* b : {"P", "P -1", "P 1.5", "P 16777216", "P 1 2"}) CHECK(parse(b).type == MessageType::Invalid);

  // Driving feel.
  m = parse("T 0.35 0.65");
  CHECK(m.type == MessageType::Feel);
  CHECK_NEAR(m.a, 0.35f, 1e-6f);
  CHECK_NEAR(m.b, 0.65f, 1e-6f);
  for (const char* b : {"T", "T 0.5", "T 1.1 0.5", "T -0.1 0.5", "T 0.5 0.1", "T 0.5 1.5", "T 0.5 0.5 1"}) {
    CHECK(parse(b).type == MessageType::Invalid);
  }

  // Light effect settings: mode rgb1 rgb2 palette speed brightness movingOnly.
  m = parse("E 4 16711680 255 6 100 255 1");
  CHECK(m.type == MessageType::Leds);
  CHECK(m.leds.mode == LedMode::SpinFade && m.leds.color1 == 0xFF0000 && m.leds.color2 == 0x0000FF);
  CHECK(m.leds.palette == LedPalette::Custom && m.leds.speed == 100 && m.leds.brightness == 255 && m.leds.movingOnly);
  for (const char* b : {"E", "E 4 0 0 0 50 40", "E 9 0 0 0 50 40 1", "E 4 16777216 0 0 50 40 1", "E 4 0 0 7 50 40 1",
                        "E 4 0 0 0 0 40 1", "E 4 0 0 0 101 40 1", "E 4 0 0 0 50 0 1", "E 4 0 0 0 50 256 1",
                        "E 4 0 0 0 50 40 2", "E 4.5 0 0 0 50 40 1", "E -1 0 0 0 50 40 1", "E 4 0 0 0 50 40 1 1"}) {
    const bool rejected = parse(b).type == MessageType::Invalid;
    if (!rejected) printf("  accepted bad message: '%s'\n", b);
    CHECK(rejected);
  }
}

static bool sameColor(LedRgb c, uint32_t hex) {
  return c.r == ((hex >> 16) & 0xFF) && c.g == ((hex >> 8) & 0xFF) && c.b == (hex & 0xFF);
}

static void testLedEffects() {
  LedConfig cfg;
  cfg.color1 = 0xFF0000;
  cfg.color2 = 0x0000FF;
  cfg.speed = 50;
  LedMotion still, forward, reverse, right;
  forward.forward = 0.5f;
  forward.moving = true;
  reverse.forward = -0.5f;
  reverse.moving = true;
  right.turn = 0.5f;
  right.moving = true;
  LedRgb px[kLedRingSize];
  LedClock clock;

  // Settings survive a round trip through the wire/storage fields.
  uint32_t fields[kLedConfigFields];
  LedConfig back;
  cfg.mode = LedMode::PaletteSweep;
  ledConfigToFields(cfg, fields);
  CHECK(ledConfigFromFields(fields, &back));
  CHECK(back.mode == cfg.mode && back.color1 == cfg.color1 && back.color2 == cfg.color2 && back.speed == cfg.speed);

  // "Moving only": the clock is frozen at rest and runs while moving.
  cfg.mode = LedMode::Pulse;
  cfg.movingOnly = true;
  clock.advance(cfg, still, 0.5f);
  CHECK(clock.phase == 0.0f);
  clock.advance(cfg, forward, 0.5f);
  CHECK(clock.phase > 0.0f);
  // Reversing or turning left runs the animation the other way.
  LedClock a, b;
  a.phase = b.phase = 10.0f;
  a.advance(cfg, forward, 0.1f);
  b.advance(cfg, reverse, 0.1f);
  CHECK(a.phase > 10.0f && b.phase < 10.0f);
  // Without "moving only" it always runs.
  cfg.movingOnly = false;
  LedClock c;
  c.advance(cfg, still, 0.5f);
  CHECK(c.phase > 0.0f);

  // Solid: every LED is colour 1.
  cfg.mode = LedMode::Solid;
  renderLeds(cfg, still, clock, px);
  for (const LedRgb& p : px) CHECK(sameColor(p, 0xFF0000));

  // Pulse at rest (moving only) is a steady dim colour 1; never brighter than colour 1.
  cfg.mode = LedMode::Pulse;
  cfg.movingOnly = true;
  renderLeds(cfg, still, clock, px);
  CHECK(px[0].r > 0 && px[0].r < 255 && px[0].g == 0 && px[0].b == 0);

  // Blink alternates between the two colours.
  cfg.mode = LedMode::Blink;
  clock.phase = 0.25f;
  renderLeds(cfg, forward, clock, px);
  CHECK(sameColor(px[0], 0xFF0000));
  clock.phase = 0.75f;
  renderLeds(cfg, forward, clock, px);
  CHECK(sameColor(px[0], 0x0000FF));

  // Spin (blink): exactly one LED is colour 1 and it moves clockwise.
  cfg.mode = LedMode::SpinBlink;
  for (int step = 0; step < kLedRingSize; ++step) {
    clock.phase = (step + 0.5f) / kLedRingSize;
    renderLeds(cfg, right, clock, px);
    for (int i = 0; i < kLedRingSize; ++i) CHECK(sameColor(px[i], i == step ? 0xFF0000 : 0x0000FF));
  }

  // Spin (fade): the head is the brightest LED.
  cfg.mode = LedMode::SpinFade;
  cfg.color2 = 0x000000;
  clock.phase = 2.0f / kLedRingSize;  // head exactly on LED 2
  renderLeds(cfg, right, clock, px);
  CHECK(px[2].r == 255 && px[1].r < px[2].r && px[3].r < px[1].r);

  // Front/back: front LEDs colour 1, rear colour 2; the leading end is brighter.
  cfg.mode = LedMode::FrontBack;
  cfg.color2 = 0x0000FF;
  renderLeds(cfg, forward, clock, px);
  CHECK(px[0].r == 255 && px[1].r == 255 && px[0].b == 0);
  CHECK(px[2].b > 0 && px[2].b < 255 && px[3].r == 0);
  renderLeds(cfg, reverse, clock, px);
  CHECK(px[2].b == 255 && px[3].b == 255 && px[0].r < 255);

  // Rainbow: the four LEDs show different colours.
  cfg.mode = LedMode::Rainbow;
  clock.phase = 0.0f;
  renderLeds(cfg, forward, clock, px);
  CHECK(sameColor(px[0], 0xFF0000));
  CHECK(!(px[0].r == px[2].r && px[0].g == px[2].g && px[0].b == px[2].b));

  // Palette sweep stays inside the palette: "Fire" never shows blue.
  cfg.mode = LedMode::PaletteSweep;
  cfg.palette = LedPalette::Fire;
  for (int k = 0; k < 40; ++k) {
    clock.phase = k * 0.137f;
    renderLeds(cfg, forward, clock, px);
    for (const LedRgb& p : px) CHECK(p.b == 0 && p.r > 0);
  }
}

int main() {
  testSlewRateLimiter();
  testVelocityEstimator();
  testWheelSpeedController();
  testStallDetector();
  testSkidSteer();
  testControlProtocol();
  testLedEffects();
  printf("\n%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
