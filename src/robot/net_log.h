#pragma once

#include <Arduino.h>
#include <WiFi.h>

// Log output that goes to the USB serial port AND to a telnet client, so the
// robot's log can be read over Wi-Fi:
//
//     telnet makerbot.local        (or: telnet <robot IP>, port 23)
//
// The last few kilobytes are kept, so a client that connects later still sees
// the boot messages. Use `Log.printf(...)` wherever you would use Serial.
// Safe to call from any task; only loop() touches the network.
class NetLog : public Print {
 public:
  void begin();  // call once Wi-Fi is up
  void loop();   // call from loop(): accepts a client and sends new text

  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t* data, size_t len) override;

 private:
  static constexpr size_t kBufferSize = 4096;
  static constexpr uint16_t kPort = 23;

  portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
  uint8_t buffer_[kBufferSize] = {};
  uint32_t written_ = 0;  // total bytes ever logged; buffer_ holds the newest kBufferSize
  uint32_t sent_ = 0;     // total bytes delivered to the current client
  WiFiServer server_{kPort};
  WiFiClient client_;
  bool started_ = false;
};

extern NetLog Log;
