// =============================================================================
//  Bring-up & calibration firmware.
//
//  Flash with:   pio run -e bringup -t upload && pio device monitor
//  Then type `help`. Run the commands in this order on a new robot:
//
//    1. enc        turn each wheel by hand: does its encoder count?
//    2. dir        wizard: maps ports to wheels, fixes motor/encoder direction
//    3. cal        counts per wheel revolution (turn wheels by hand)
//    4. maxspeed   top speed at full duty            (wheels OFF the ground)
//    5. friction   minimum duty to start turning     (wheels OFF the ground)
//    6. step       closed-loop step test for tuning  (wheels OFF the ground)
//
//  Each step prints the exact lines to paste into include/robot_config.h.
//  Every test ramps the motors (never instant reversal) and stops on any key.
// =============================================================================
#include <Arduino.h>
#include <math.h>

#include "../common/battery_sensor.h"
#include "../common/wheel_set.h"
#include "VelocityEstimator.h"
#include "WheelSpeedController.h"
#include "robot_config.h"

namespace {

constexpr int kN = config::kWheelCount;
WheelSet wheels;
BatterySensor battery;
WheelGains gains = config::wheelGains();

// --- serial helpers ---------------------------------------------------------

void drainInput() {
  while (Serial.available()) Serial.read();
}

// True (and input drained) if the user pressed any key.
bool userAborted() {
  if (!Serial.available()) return false;
  delay(20);
  drainInput();
  return true;
}

// Blocking line read with echo; handles backspace and CR/LF/CRLF endings.
String readLine() {
  String line;
  for (;;) {
    while (!Serial.available()) delay(5);
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (c == '\r') {
        delay(5);
        if (Serial.peek() == '\n') Serial.read();
      }
      Serial.println();
      line.trim();
      return line;
    }
    if ((c == '\b' || c == 127) && line.length() > 0) {
      line.remove(line.length() - 1);
      Serial.print("\b \b");
    } else if (c >= 32 && c < 127 && line.length() < 120) {
      line += c;
      Serial.print(c);
    }
  }
}

String prompt(const char* question) {
  Serial.print(question);
  Serial.print(" ");
  return readLine();
}

bool confirm(const char* question) {
  for (;;) {
    String a = prompt(question);
    a.toLowerCase();
    if (a == "y" || a == "yes") return true;
    if (a == "n" || a == "no" || a == "q") return false;
    Serial.println("  please answer y or n");
  }
}

bool confirmWheelsUp() {
  Serial.println();
  Serial.println("!! The motors will run at high speed. Lift the robot so NO wheel touches anything.");
  return confirm("Wheels off the ground and clear? (y/n)");
}

// --- motion helpers ---------------------------------------------------------

// Linearly ramps the selected motors from their current duty to `to` over ms.
// Returns false if the user aborted (all motors are braked in that case).
bool rampTo(const bool* which, float to, uint32_t ms) {
  float from[kN];
  for (int i = 0; i < kN; ++i) from[i] = wheels.motor(i).duty();
  const uint32_t start = millis();
  for (;;) {
    const uint32_t elapsed = millis() - start;
    const float f = ms == 0 ? 1.0f : fminf(1.0f, static_cast<float>(elapsed) / static_cast<float>(ms));
    for (int i = 0; i < kN; ++i)
      if (which[i]) wheels.motor(i).setDuty(from[i] + (to - from[i]) * f);
    if (f >= 1.0f) return true;
    if (userAborted()) {
      wheels.brakeAll();
      Serial.println("\n[aborted]");
      return false;
    }
    delay(5);
  }
}

bool rampOne(int index, float to, uint32_t ms) {
  bool which[kN] = {};
  which[index] = true;
  return rampTo(which, to, ms);
}

// Waits ms while watching for abort.
bool waitMs(uint32_t ms) {
  const uint32_t start = millis();
  while (millis() - start < ms) {
    if (userAborted()) {
      wheels.brakeAll();
      Serial.println("\n[aborted]");
      return false;
    }
    delay(5);
  }
  return true;
}

void countsSnapshot(int64_t* out) {
  for (int i = 0; i < kN; ++i) out[i] = wheels.encoder(i).count();
}

// --- commands ---------------------------------------------------------------

void cmdHelp() {
  Serial.println(R"(
Commands (any key aborts a running test):
  enc                     live encoder counts - turn wheels by hand
  spin <1-4> <pct> [ms]   open-loop drive one motor, pct -100..100 (default 1000 ms)
  dir                     guided wizard: which port drives which wheel, directions
  cal                     counts per wheel revolution (turn wheels by hand)
  maxspeed                full-duty top speed           [wheels off ground]
  friction                breakaway duty -> kS          [wheels off ground]
  step <0-4> <pct> [ms]   closed-loop step response, 0 = all wheels [wheels off ground]
  gains [kp ki kf ks tau] show / set controller gains used by `step`
  batt                    live PM02 battery voltage / current readout
  batt cal <volts>        calibrate the voltage scale against a multimeter reading
  stop                    brake all motors
  help                    this text
Suggested order on a new robot: enc, dir, cal, maxspeed, friction, step)");
}

void cmdBatt(const String& args) {
  if (!battery.enabled()) {
    Serial.println("Battery monitor is disabled (kBatteryMonitorEnabled in robot_config.h).");
    return;
  }
  if (args.startsWith("cal")) {
    const float measured = args.substring(3).toFloat();
    if (!(measured > 3.0f && measured < 30.0f)) {
      Serial.println("usage: batt cal <volts>   e.g. batt cal 12.34  (battery voltage measured with a multimeter)");
      return;
    }
    // Average 2 s of readings with the motors idle.
    uint64_t sum = 0;
    int n = 0;
    for (uint32_t start = millis(); millis() - start < 2000; ++n) {
      battery.sample(millis());
      sum += battery.rawVoltageMv();
      delay(20);
    }
    const float pinV = static_cast<float>(sum) / static_cast<float>(n) * 0.001f;
    if (pinV < 0.1f) {
      Serial.printf("!! Voltage pin GPIO %u reads %.0f mV - check the PM02 VOLTAGE wire and GND.\n",
                    config::kBatteryVoltagePin, pinV * 1000.0f);
      return;
    }
    const float scale = measured / pinV;
    Serial.printf("Pin %.1f mV, meter %.2f V -> scale %.3f (Holybro nominal 18.182, now %.3f)\n", pinV * 1000.0f,
                  measured, scale, config::kBatteryVoltsPerVolt);
    if (fabsf(scale / 18.182f - 1.0f) > 0.15f)
      Serial.println("!! More than 15% off nominal - check the wiring (pin 4 = VOLTAGE) before trusting this.");
    Serial.println("\n==== Paste into include/robot_config.h ====");
    Serial.printf("constexpr float kBatteryVoltsPerVolt = %.3ff;\n", scale);
    return;
  }

  Serial.printf("PM02 on GPIO %u (voltage) / %u (current). Press any key to stop.\n", config::kBatteryVoltagePin,
                config::kBatteryCurrentPin);
  Serial.println("Compare the pack voltage with a multimeter; then run `batt cal <volts>` if it is off.");
  uint32_t lastPrint = 0;
  while (!userAborted()) {
    const uint32_t now = millis();
    battery.sample(now);
    if (now - lastPrint >= 250) {
      lastPrint = now;
      const BatteryMonitor& m = battery.monitor();
      const char* level = "absent";
      if (m.level() == BatteryLevel::Ok) level = "ok";
      else if (m.level() == BatteryLevel::Low) level = "LOW";
      else if (m.level() == BatteryLevel::Critical) level = "CRITICAL";
      Serial.printf("pins %4u / %4u mV  ->  %6.2f V  %6.2f A   filtered %6.2f V  %dS %4.2f V/cell  %3d%%  %s\n",
                    static_cast<unsigned>(battery.rawVoltageMv()), static_cast<unsigned>(battery.rawCurrentMv()),
                    battery.instantVolts(), battery.instantAmps(), m.volts(), m.cells(), m.cellVolts(), m.percent(),
                    level);
    }
    delay(20);
  }
}

void cmdEnc() {
  Serial.println("Turn the wheels by hand. Counts should change smoothly in BOTH directions.");
  Serial.println("Press any key to stop.");
  wheels.coastAll();
  while (!userAborted()) {
    for (int i = 0; i < kN; ++i)
      Serial.printf("M%d:%9lld  ", i + 1, static_cast<long long>(wheels.encoder(i).count()));
    Serial.println();
    delay(200);
  }
  wheels.brakeAll();
}

void cmdSpin(const String& args) {
  int port = 0;
  float pct = 0;
  int ms = 1000;
  const int n = sscanf(args.c_str(), "%d %f %d", &port, &pct, &ms);
  if (n < 2 || port < 1 || port > kN || pct < -100 || pct > 100) {
    Serial.println("usage: spin <1-4> <pct -100..100> [ms]");
    return;
  }
  const int idx = port - 1;
  ms = constrain(ms, 100, 10000);
  Serial.printf("%s at %.0f%% for %d ms (configured direction applied)\n", wheels.hw(idx).name, pct, ms);

  VelocityEstimator<6> est;
  est.reset(wheels.encoder(idx).count(), esp_timer_get_time());
  const int64_t startCount = wheels.encoder(idx).count();
  if (!rampOne(idx, pct / 100.0f, 250)) return;
  const uint32_t start = millis();
  uint32_t lastPrint = 0;
  while (millis() - start < static_cast<uint32_t>(ms)) {
    est.update(wheels.encoder(idx).count(), esp_timer_get_time());
    if (millis() - lastPrint >= 100) {
      lastPrint = millis();
      Serial.printf("  count %8lld  speed %7.0f counts/s\n", static_cast<long long>(wheels.encoder(idx).count()),
                    est.velocity());
    }
    if (!waitMs(10)) return;
  }
  if (!rampOne(idx, 0.0f, 250)) return;
  wheels.brakeAll();
  Serial.printf("Total: %lld counts\n", static_cast<long long>(wheels.encoder(idx).count() - startCount));
}

// Direction / mapping wizard --------------------------------------------------
struct DirResult {
  int position = -1;  // index into kPositionCodes
  bool motorInvert = false;
  bool encoderInvert = false;
  bool encoderOk = false;
};

const char* const kPositionCodes[4] = {"fl", "rl", "fr", "rr"};
const char* const kPositionNames[4] = {"front-left", "rear-left", "front-right", "rear-right"};

void cmdDir() {
  Serial.println(R"(
Direction wizard. For each motor port one wheel turns slowly for about a
second. Watch WHICH wheel turns and WHICH WAY. Keep the wheels free (lifting
the robot is easiest) - the motors only run at low power.)");
  if (!confirm("Ready? (y/n)")) return;

  wheels.applyConfiguredInversion(false);  // raw hardware directions
  DirResult results[kN];
  bool aborted = false;

  for (int i = 0; i < kN && !aborted; ++i) {
    Serial.printf("\n--- Port M%d (motor GPIO %u/%u, encoder GPIO %u/%u) ---\n", i + 1, wheels.hw(i).motorIn1,
                  wheels.hw(i).motorIn2, wheels.hw(i).encoderA, wheels.hw(i).encoderB);
    int64_t before[kN], after[kN];
    countsSnapshot(before);
    if (!rampOne(i, 0.35f, 300) || !waitMs(900) || !rampOne(i, 0.0f, 300) || !waitMs(400)) {
      aborted = true;
      break;
    }
    wheels.brakeAll();
    countsSnapshot(after);

    const int64_t own = after[i] - before[i];
    Serial.printf("Encoder M%d counted %lld\n", i + 1, static_cast<long long>(own));
    for (int j = 0; j < kN; ++j) {
      const int64_t other = after[j] - before[j];
      if (j != i && llabs(other) > 100) {
        Serial.printf("!! The encoder wired as M%d counted %lld while M%d was driven.\n"
                      "   The encoder cables of M%d and M%d are probably swapped.\n",
                      j + 1, static_cast<long long>(other), i + 1, i + 1, j + 1);
      }
    }

    DirResult& r = results[i];
    for (;;) {
      String a = prompt("Which wheel turned? (fl / rl / fr / rr, or none)");
      a.toLowerCase();
      if (a == "none") break;
      for (int p = 0; p < 4; ++p)
        if (a == kPositionCodes[p]) r.position = p;
      if (r.position >= 0) break;
      Serial.println("  type fl, rl, fr, rr or none");
    }
    if (r.position < 0) {
      Serial.printf("!! Nothing turned: check the motor plug on port M%d, the DC power jack and the\n"
                    "   power switch%s.\n",
                    i + 1, i >= 2 ? ", and that the Motor/IO DIP switch is on the MOTOR side" : "");
      continue;
    }
    const bool forward = confirm("Did it turn the way that drives the robot FORWARD? (y/n)");
    r.motorInvert = !forward;
    r.encoderOk = llabs(own) >= 50;
    if (!r.encoderOk) {
      Serial.printf("!! Encoder M%d barely counted. Check: blue->3.3V, black->GND, yellow->GPIO %u,\n"
                    "   green->GPIO %u. Then run `enc` and turn that wheel by hand.\n",
                    i + 1, wheels.hw(i).encoderA, wheels.hw(i).encoderB);
    } else {
      // A raw +duty produced `own` counts; forward motion must count up.
      const int64_t forwardCounts = forward ? own : -own;
      r.encoderInvert = forwardCounts < 0;
    }
  }

  wheels.brakeAll();
  wheels.applyConfiguredInversion(true);
  if (aborted) return;

  bool problems = false;
  for (int p = 0; p < 4; ++p) {
    int uses = 0;
    for (const DirResult& r : results)
      if (r.position == p) ++uses;
    if (uses > 1) {
      Serial.printf("!! %s was chosen for %d ports.\n", kPositionNames[p], uses);
      problems = true;
    }
  }
  int leftCount = 0;
  for (const DirResult& r : results) {
    if (r.position < 0 || !r.encoderOk) problems = true;
    if (r.position == 0 || r.position == 1) ++leftCount;
  }
  if (leftCount != 2) problems = true;

  Serial.println("\n==== Paste into include/robot_config.h, replacing the kWheels block ====");
  Serial.println("constexpr WheelHw kWheels[kWheelCount] = {");
  Serial.println("    //  name             IN1 IN2  encA encB  invMotor invEnc  isLeft");
  for (int i = 0; i < kN; ++i) {
    const config::WheelHw& w = wheels.hw(i);
    const DirResult& r = results[i];
    char name[32];
    snprintf(name, sizeof(name), "\"M%d %s\",", i + 1, r.position >= 0 ? kPositionNames[r.position] : "UNKNOWN");
    Serial.printf("    {%-18s %2u, %2u,   %2u,  %2u,   %-6s,  %-6s, %s},\n", name, w.motorIn1, w.motorIn2, w.encoderA,
                  w.encoderB, r.motorInvert ? "true" : "false", r.encoderInvert ? "true" : "false",
                  (r.position == 0 || r.position == 1) ? "true" : "false");
  }
  Serial.println("};");
  Serial.println("=======================================================================");
  if (problems) Serial.println("!! There were problems above (need 2 left + 2 right wheels, all encoders counting).");
  Serial.println("After editing, re-flash this bring-up firmware so the next tests use the new directions.");
}

void cmdCal() {
  Serial.println(R"(
Counts-per-revolution calibration.
Put a tape mark on each tyre. Turn EVERY wheel exactly 10 full turns in the
direction that drives the robot FORWARD (one wheel at a time is fine; the
counters are independent). Then press Enter. Type q + Enter to cancel.)");
  wheels.coastAll();
  for (int i = 0; i < kN; ++i) wheels.encoder(i).reset();
  const String answer = readLine();
  wheels.brakeAll();
  if (answer == "q") return;

  constexpr float kTurns = 10.0f;
  float sum = 0, lowest = 1e9f, highest = 0;
  bool signProblem = false;
  for (int i = 0; i < kN; ++i) {
    const float cpr = static_cast<float>(wheels.encoder(i).count()) / kTurns;
    Serial.printf("  %-16s %8.1f counts/rev  (gear ratio ~ %.2f)\n", wheels.hw(i).name, cpr, fabsf(cpr) / 44.0f);
    if (cpr < 0) signProblem = true;
    sum += fabsf(cpr);
    lowest = fminf(lowest, fabsf(cpr));
    highest = fmaxf(highest, fabsf(cpr));
  }
  const float avg = sum / kN;
  if (avg < 1.0f) {
    Serial.println("!! No counts at all - check encoder power and wiring with `enc`.");
    return;
  }
  if (signProblem) Serial.println("!! A wheel counted NEGATIVE going forward: run `dir` first (invertEncoder).");
  if (lowest < 0.5f * avg) {
    Serial.println("!! One wheel counted far less - was it turned 10 times? Is its encoder OK?");
  } else if ((highest - lowest) / avg > 0.03f) {
    Serial.println("!  Wheels differ by more than 3% - probably a partial turn. Repeat for a cleaner number.");
  }
  Serial.printf("\nPaste into robot_config.h:\n  constexpr float kCountsPerWheelRev = %.1ff;\n", avg);
}

void cmdMaxSpeed() {
  if (!confirmWheelsUp()) return;
  bool all[kN];
  for (bool& b : all) b = true;

  Serial.println("Ramping all motors to 100%...");
  if (!rampTo(all, 1.0f, 800) || !waitMs(1000)) return;
  int64_t c0[kN], c1[kN];
  countsSnapshot(c0);
  const int64_t t0 = esp_timer_get_time();
  if (!waitMs(1000)) return;
  countsSnapshot(c1);
  const int64_t t1 = esp_timer_get_time();
  if (!rampTo(all, 0.0f, 800)) return;
  wheels.brakeAll();

  const float seconds = static_cast<float>(t1 - t0) / 1e6f;
  float slowest = 1e9f;
  for (int i = 0; i < kN; ++i) {
    const float cps = static_cast<float>(c1[i] - c0[i]) / seconds;
    const float rpm = cps / config::kCountsPerWheelRev * 60.0f;
    Serial.printf("  %-16s %7.0f counts/s = %5.0f wheel rpm = %.2f m/s\n", wheels.hw(i).name, cps, rpm,
                  cps * config::kMetersPerCount);
    if (cps <= 0) Serial.println("    !! not positive: run `dir` (directions) or check this encoder");
    slowest = fminf(slowest, cps);
  }
  if (slowest > 0) {
    const float suggestion = floorf(slowest * 0.92f / 10.0f) * 10.0f;
    Serial.printf("\nPaste into robot_config.h (8%% below the slowest wheel so every wheel can reach it):\n"
                  "  constexpr float kMaxWheelSpeedCps = %.0ff;\n",
                  suggestion);
    Serial.println("(Measured without load on the battery's present charge.)");
  }
}

void cmdFriction() {
  if (!confirmWheelsUp()) return;
  float breakaway[kN];
  for (int i = 0; i < kN; ++i) {
    breakaway[i] = NAN;
    VelocityEstimator<6> est;
    est.reset(wheels.encoder(i).count(), esp_timer_get_time());
    const float threshold = 0.02f * config::kMaxWheelSpeedCps;
    int movingSamples = 0;
    for (float duty = 0.0f; duty <= 0.40f; duty += 0.0025f) {
      wheels.motor(i).setDuty(duty);
      for (int k = 0; k < 4; ++k) {  // 40 ms per duty step
        if (!waitMs(10)) return;
        est.update(wheels.encoder(i).count(), esp_timer_get_time());
      }
      movingSamples = est.velocity() > threshold ? movingSamples + 1 : 0;
      if (movingSamples >= 2) {
        breakaway[i] = duty;
        break;
      }
    }
    if (!rampOne(i, 0.0f, 200)) return;
    wheels.motor(i).brake();
    if (isnan(breakaway[i])) {
      Serial.printf("  %-16s did not start below 40%% duty! (direction/encoder setup, or a jam)\n",
                    wheels.hw(i).name);
    } else {
      Serial.printf("  %-16s starts turning at %.1f%% duty\n", wheels.hw(i).name, breakaway[i] * 100.0f);
    }
    if (!waitMs(300)) return;
  }
  float lowest = 1.0f;
  for (float b : breakaway)
    if (!isnan(b)) lowest = fminf(lowest, b);
  if (lowest < 1.0f) {
    Serial.printf("\nPaste into robot_config.h wheelGains():\n  g.kS = %.3ff;\n", lowest * 0.8f);
    Serial.println("(80% of the easiest-turning wheel, so kS alone never moves a wheel.)");
  }
}

void printGains() {
  Serial.printf("kP=%.3f kI=%.3f kF=%.3f kS=%.3f refTau=%.3f iLimit=%.2f\n", gains.kP, gains.kI, gains.kF, gains.kS,
                gains.refTauS, gains.iLimit);
  Serial.printf("robot_config.h:  g.kP = %.3ff; g.kI = %.3ff; g.kF = %.3ff; g.kS = %.3ff; g.refTauS = %.3ff;\n",
                gains.kP, gains.kI, gains.kF, gains.kS, gains.refTauS);
}

void cmdGains(const String& args) {
  if (args.length() > 0) {
    float v[5] = {gains.kP, gains.kI, gains.kF, gains.kS, gains.refTauS};
    const int n = sscanf(args.c_str(), "%f %f %f %f %f", &v[0], &v[1], &v[2], &v[3], &v[4]);
    bool valid = n >= 1;
    for (float x : v) valid = valid && isfinite(x) && x >= 0.0f && x <= 50.0f;
    if (!valid) {
      Serial.println("usage: gains <kp> [ki] [kf] [ks] [refTau]   (all >= 0)");
      return;
    }
    gains.kP = v[0];
    gains.kI = v[1];
    gains.kF = v[2];
    gains.kS = v[3];
    gains.refTauS = v[4];
  }
  printGains();
}

void cmdStep(const String& args) {
  int which = -1;
  float pct = 0;
  int ms = 1500;
  const int n = sscanf(args.c_str(), "%d %f %d", &which, &pct, &ms);
  if (n < 2 || which < 0 || which > kN || pct < -100 || pct > 100 || pct == 0) {
    Serial.println("usage: step <0-4> <pct of max speed, -100..100> [ms]   (0 = all wheels)");
    return;
  }
  ms = constrain(ms, 600, 5000);
  if (!confirmWheelsUp()) return;
  printGains();

  const float target = pct / 100.0f;
  WheelSpeedController ctrl[kN] = {WheelSpeedController(gains), WheelSpeedController(gains),
                                   WheelSpeedController(gains), WheelSpeedController(gains)};
  VelocityEstimator<6> est[kN];
  const int64_t tStart = esp_timer_get_time();
  for (int i = 0; i < kN; ++i) est[i].reset(wheels.encoder(i).count(), tStart);

  Serial.println("t_ms,target,reference,speed_M1,speed_M2,speed_M3,speed_M4,duty_M1,duty_M2,duty_M3,duty_M4");
  const int totalSteps = (ms + 700) / 10;  // the step, then 0.7 s of stopping
  float measured[kN] = {};
  float duty[kN] = {};
  const int probe = which == 0 ? 0 : which - 1;  // wheel used for the summary
  float peak = 0, sum = 0, sumSq = 0;
  int settleIndex = -1, steadyCount = 0;
  bool aborted = false;

  TickType_t wake = xTaskGetTickCount();
  for (int k = 0; k < totalSteps; ++k) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
    const int64_t now = esp_timer_get_time();
    const bool stepping = k * 10 < ms;
    const float tgt = stepping ? target : 0.0f;
    for (int i = 0; i < kN; ++i) {
      measured[i] = est[i].update(wheels.encoder(i).count(), now) / config::kMaxWheelSpeedCps;
      const bool active = which == 0 || which - 1 == i;
      duty[i] = active ? ctrl[i].update(tgt, measured[i], 0.01f) : 0.0f;
      wheels.motor(i).setDuty(duty[i]);
    }
    Serial.printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", k * 10, tgt, ctrl[probe].reference(),
                  measured[0], measured[1], measured[2], measured[3], duty[0], duty[1], duty[2], duty[3]);
    if (stepping) {
      const float m = target > 0 ? measured[probe] : -measured[probe];
      const float t = fabsf(target);
      peak = fmaxf(peak, m);
      const bool inBand = fabsf(m - t) <= 0.05f * t;
      if (inBand && settleIndex < 0) settleIndex = k;
      if (!inBand) settleIndex = -1;
      if (k * 10 >= ms - 500) {  // last 0.5 s of the step = steady state
        sum += m;
        sumSq += m * m;
        ++steadyCount;
      }
    }
    if (userAborted()) {
      Serial.println("[aborted]");
      aborted = true;
      break;
    }
  }
  wheels.brakeAll();
  if (aborted || steadyCount == 0) return;

  const float t = fabsf(target);
  const float mean = sum / steadyCount;
  const float sd = sqrtf(fmaxf(0.0f, sumSq / steadyCount - mean * mean));
  char settle[24];
  if (settleIndex >= 0) snprintf(settle, sizeof(settle), "%d ms", settleIndex * 10);
  else snprintf(settle, sizeof(settle), "never");
  Serial.printf("\nM%d summary: overshoot %.1f%%, settle(5%%) %s, steady error %+.1f%%, ripple(sd) %.1f%%\n",
                probe + 1, fmaxf(0.0f, (peak - t) / t * 100.0f), settle, (mean - t) / t * 100.0f, sd / t * 100.0f);
  Serial.println("Tuning: overshoot/oscillation -> lower kI (then kP); slow to settle -> raise kI;");
  Serial.println("        jittery speed -> lower kP. Paste the CSV into a spreadsheet to plot it.");
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  const int space = line.indexOf(' ');
  String cmd = space < 0 ? line : line.substring(0, space);
  String args = space < 0 ? String() : line.substring(space + 1);
  cmd.toLowerCase();
  args.trim();

  if (cmd == "help" || cmd == "?") cmdHelp();
  else if (cmd == "enc") cmdEnc();
  else if (cmd == "spin") cmdSpin(args);
  else if (cmd == "dir") cmdDir();
  else if (cmd == "cal") cmdCal();
  else if (cmd == "maxspeed") cmdMaxSpeed();
  else if (cmd == "friction") cmdFriction();
  else if (cmd == "step") cmdStep(args);
  else if (cmd == "gains") cmdGains(args);
  else if (cmd == "batt") cmdBatt(args);
  else if (cmd == "stop") wheels.brakeAll();
  else Serial.printf("unknown command '%s' - type help\n", cmd.c_str());
  wheels.brakeAll();
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n\n=== Maker-ESP32 robot: BRING-UP firmware ===");
  Serial.println("Motors need 6-16 V on the DC jack with the power switch ON.");
  Serial.println("Ports M3/M4 also need the Motor/IO DIP switch on the MOTOR side.");
  if (!wheels.begin()) Serial.println("!! Hardware setup reported errors (see above).");
  battery.begin();
  Serial.printf("Config: %.1f counts/rev, max %.0f counts/s (%.2f m/s)\n", config::kCountsPerWheelRev,
                config::kMaxWheelSpeedCps, config::kMaxWheelSpeedMps);
  cmdHelp();
  Serial.print("\n> ");
}

void loop() {
  handleCommand(readLine());
  Serial.print("\n> ");
}
