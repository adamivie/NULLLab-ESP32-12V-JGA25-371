#pragma once

#include <ESPAsyncWebServer.h>

#include <atomic>

#include "../common/battery_sensor.h"
#include "drive_system.h"

// Wi-Fi (own access point, or your home network if wifi_secrets.h exists),
// the web page and the WebSocket used to drive.
//
// Exactly one connected browser is the "driver" at a time: the first one to
// connect, or whoever presses "Take control". Others see live telemetry and
// can always hit STOP - the emergency stop is accepted from every client.
class WebControl {
 public:
  WebControl(DriveSystem& drive, const BatterySensor& battery);

  void begin();
  // Call from loop(): sends telemetry ~10x per second and housekeeping.
  void loop(uint32_t nowMs);

  bool hasClients() const { return ws_.count() > 0; }
  const String& address() const { return address_; }

 private:
  void startWifi();
  void onEvent(AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);
  void handleText(AsyncWebSocketClient* client, const char* text, size_t len);
  void sendHello(AsyncWebSocketClient* client);
  void sendTelemetry();

  DriveSystem& drive_;
  const BatterySensor& battery_;
  AsyncWebServer server_;
  AsyncWebSocket ws_;
  std::atomic<uint32_t> ownerId_{0};
  String address_;
  uint32_t lastTelemetryMs_ = 0;
  uint32_t lastCleanupMs_ = 0;
};
