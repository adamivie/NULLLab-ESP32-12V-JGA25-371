#pragma once

#include <ESPAsyncWebServer.h>

#include <atomic>

#include "drive_system.h"
#include "status_leds.h"

// Wi-Fi (own access point, or your home network if wifi_secrets.h exists),
// the web page and the WebSocket used to drive, plus the Wi-Fi log (telnet)
// and over-the-air firmware updates.
//
// Exactly one connected browser is the "driver" at a time: the first one to
// connect, or whoever presses "Take control". Others see live telemetry and
// can always hit STOP - the emergency stop is accepted from every client.
class WebControl {
 public:
  WebControl(DriveSystem& drive, StatusLeds& leds);

  void begin();
  // Call from loop(): sends telemetry ~10x per second and housekeeping.
  void loop(uint32_t nowMs);

  bool hasClients() const { return ws_.count() > 0; }
  const String& address() const { return address_; }

 private:
  void startWifi();
  void startOta();
  void loadSettings();
  void saveSettings();
  void onEvent(AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);
  void handleText(AsyncWebSocketClient* client, const char* text, size_t len);
  void sendHello(AsyncWebSocketClient* client);
  // Writes {"led":[...]} (the light effect settings) into buf; returns its length.
  int ledJson(char* buf, size_t size) const;
  void sendTelemetry();

  DriveSystem& drive_;
  StatusLeds& leds_;
  AsyncWebServer server_;
  AsyncWebSocket ws_;
  std::atomic<uint32_t> ownerId_{0};
  std::atomic<bool> updating_{false};
  String address_;
  uint32_t lastTelemetryMs_ = 0;
  uint32_t lastCleanupMs_ = 0;
  // Speed limit and driving feel are saved a moment after the last change.
  std::atomic<bool> settingsDirty_{false};
  std::atomic<uint32_t> settingsChangedMs_{0};
};
