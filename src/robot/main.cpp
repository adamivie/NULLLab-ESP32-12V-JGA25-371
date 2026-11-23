// =============================================================================
//  4WD robot firmware: NULLLAB Maker-ESP32 + 4x JGA25-371 encoder motors.
//
//  Flash:  pio run -e robot -t upload && pio device monitor
//  Drive:  join the robot's Wi-Fi (see serial log / robot_config.h) and open
//          http://192.168.4.1 on your phone. Press ARM, then use the joystick.
//
//  Run the bring-up firmware first (pio run -e bringup -t upload) to wire up
//  and calibrate the motors; see README.md.
// =============================================================================
#include <Arduino.h>

#include "../common/battery_sensor.h"
#include "drive_system.h"
#include "status_leds.h"
#include "web_control.h"

namespace {
DriveSystem drive;
BatterySensor battery;
WebControl web(drive, battery);
StatusLeds leds;
void logBatteryChanges(BatteryLevel level) {
  static BatteryLevel last = BatteryLevel::Absent;
  static bool first = true;
  if (!battery.enabled() || (!first && level == last)) return;
  first = false;
  last = level;
  const BatteryMonitor& m = battery.monitor();
  switch (level) {
    case BatteryLevel::Absent:
      Serial.println("[battery] no voltage reading (USB only, or PM02 not wired)");
      break;
    case BatteryLevel::Ok:
      Serial.printf("[battery] %.2f V\n", m.volts());
      break;
    case BatteryLevel::Low:
      Serial.printf("[battery] LOW: %.2f V (%.2f V/cell) - speed capped, recharge soon\n", m.volts(), m.cellVolts());
      break;
    case BatteryLevel::Critical:
      Serial.printf("[battery] CRITICAL: %.2f V (%.2f V/cell) - stopped; replace the battery\n", m.volts(),
                    m.cellVolts());
      break;
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n\n=== Maker-ESP32 robot ===");

  leds.begin();
  leds.showStarting();

  if (!drive.begin()) {
    Serial.println("!! Drive hardware failed to initialise - the robot will refuse to arm properly.");
    Serial.println("   Flash the bring-up firmware to diagnose (pio run -e bringup -t upload).");
  }
  Serial.printf("Top speed %.2f m/s, default limit %.0f%%\n", config::kMaxWheelSpeedMps,
                config::kDefaultSpeedLimit * 100.0f);

  battery.begin();
  if (battery.enabled()) {
    Serial.printf("Battery monitor (PM02) on GPIO %u/%u, %d cells, low %.2f V/cell, critical %.2f V/cell\n",
                  config::kBatteryVoltagePin, config::kBatteryCurrentPin, config::kBatteryCells, config::kCellLowV,
                  config::kCellCriticalV);
  }

  web.begin();
}

void loop() {
  const uint32_t now = millis();
  battery.sample(now);
  const BatteryLevel level = battery.monitor().level();
  drive.setBatteryLevel(level);
  logBatteryChanges(level);
  web.loop(now);
  leds.update(drive.status(), web.hasClients(), level, now);
  delay(20);
}
