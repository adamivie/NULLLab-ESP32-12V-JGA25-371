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

#include "drive_system.h"
#include "net_log.h"
#include "status_leds.h"
#include "web_control.h"

namespace {
DriveSystem drive;
StatusLeds leds;
WebControl web(drive, leds);
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  Log.println("\n\n=== Maker-ESP32 robot ===");

  leds.begin();
  leds.showStarting();

  if (!drive.begin()) {
    Log.println("!! Drive hardware failed to initialise - the robot will refuse to arm properly.");
    Log.println("   Flash the bring-up firmware to diagnose (pio run -e bringup -t upload).");
  }
  Log.printf("Top speed %.2f m/s, default limit %.0f%%\n", config::kMaxWheelSpeedMps,
                config::kDefaultSpeedLimit * 100.0f);

  web.begin();
}

void loop() {
  const uint32_t now = millis();
  web.loop(now);
  leds.update(drive.status(), web.hasClients(), now);
  delay(10);
}
