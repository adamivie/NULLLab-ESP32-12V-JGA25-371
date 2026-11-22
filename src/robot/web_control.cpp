#include "web_control.h"

#include <ESPmDNS.h>
#include <WiFi.h>
#include <esp_mac.h>

#include "ControlProtocol.h"
#include "robot_config.h"
#include "web_page.h"

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif

namespace {
constexpr uint32_t kTelemetryPeriodMs = 100;

const char* stateName(DriveState s) {
  switch (s) {
    case DriveState::Armed:
      return "armed";
    case DriveState::Stopping:
      return "stopping";
    case DriveState::Fault:
      return "fault";
    case DriveState::Disarmed:
    default:
      return "disarmed";
  }
}
}  // namespace

WebControl::WebControl(DriveSystem& drive) : drive_(drive), server_(80), ws_("/ws") {}

void WebControl::startWifi() {
  WiFi.persistent(false);  // don't wear flash with credentials
  bool connected = false;

#if defined(WIFI_STA_SSID) && defined(WIFI_STA_PASSWORD)
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(config::kHostname);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASSWORD);
  Serial.printf("[wifi] joining \"%s\"", WIFI_STA_SSID);
  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < config::kStationConnectTimeoutMs) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  connected = WiFi.status() == WL_CONNECTED;
  if (connected) {
    address_ = WiFi.localIP().toString();
    Serial.printf("[wifi] connected, open http://%s or http://%s.local\n", address_.c_str(), config::kHostname);
  } else {
    Serial.println("[wifi] could not join - starting the robot's own network instead");
    WiFi.disconnect(true);
  }
#endif

  if (!connected) {
    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    char ssid[33];
    snprintf(ssid, sizeof(ssid), "%s%02X%02X", config::kApSsidPrefix, mac[4], mac[5]);
    WiFi.mode(WIFI_AP);
    WiFi.softAPsetHostname(config::kHostname);
    if (!WiFi.softAP(ssid, config::kApPassword, config::kApChannel)) {
      Serial.println("[wifi] !! failed to start access point (password must be 8-63 characters)");
    }
    address_ = WiFi.softAPIP().toString();
    Serial.printf("[wifi] network \"%s\", password \"%s\" -> open http://%s\n", ssid, config::kApPassword,
                  address_.c_str());
  }

  WiFi.setSleep(false);  // power-save adds 100+ ms latency spikes
  if (MDNS.begin(config::kHostname)) MDNS.addService("http", "tcp", 80);
}

void WebControl::begin() {
  startWifi();

  ws_.onEvent([this](AsyncWebSocket*, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data,
                     size_t len) { onEvent(client, type, arg, data, len); });
  server_.addHandler(&ws_);

  server_.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse* response = request->beginResponse(
        200, "text/html", reinterpret_cast<const uint8_t*>(kIndexHtml), sizeof(kIndexHtml) - 1);
    response->addHeader("Cache-Control", "no-store");
    request->send(response);
  });
  server_.onNotFound([](AsyncWebServerRequest* request) { request->redirect("/"); });
  server_.begin();
}

void WebControl::onEvent(AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT: {
      // A slow phone must never get the connection closed on it just because
      // telemetry queued up; dropping a telemetry frame is harmless.
      client->setCloseClientOnQueueFull(false);
      uint32_t expected = 0;
      ownerId_.compare_exchange_strong(expected, client->id());  // first client drives
      Serial.printf("[ws] client %u connected from %s\n", static_cast<unsigned>(client->id()), client->remoteIP().toString().c_str());
      sendHello(client);
      break;
    }
    case WS_EVT_DISCONNECT: {
      uint32_t expected = client->id();
      if (ownerId_.compare_exchange_strong(expected, 0)) {
        drive_.softStop();  // the driver left: bring the robot to a stop now
      }
      Serial.printf("[ws] client %u disconnected\n", static_cast<unsigned>(client->id()));
      break;
    }
    case WS_EVT_DATA: {
      const AwsFrameInfo* info = static_cast<const AwsFrameInfo*>(arg);
      // Only whole, unfragmented text frames - our messages are tiny.
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        handleText(client, reinterpret_cast<const char*>(data), len);
      }
      break;
    }
    default:
      break;
  }
}

void WebControl::handleText(AsyncWebSocketClient* client, const char* text, size_t len) {
  const Message msg = parseMessage(text, len);
  if (msg.type == MessageType::Invalid) return;

  // STOP works from any connected device.
  if (msg.type == MessageType::Stop) {
    drive_.emergencyStop();
    return;
  }
  if (msg.type == MessageType::Claim) {
    const uint32_t previous = ownerId_.exchange(client->id());
    if (previous != client->id()) drive_.softStop();  // new driver must arm deliberately
    return;
  }
  if (ownerId_.load() != client->id()) return;  // spectators cannot drive

  switch (msg.type) {
    case MessageType::Drive:
      drive_.setJoystick(msg.a, msg.b);
      break;
    case MessageType::SpeedLimit:
      drive_.setSpeedLimit(msg.a);
      break;
    case MessageType::Arm:
      if (msg.a == 0.0f) {
        drive_.softStop();
      } else {
        const char* reason = nullptr;
        switch (drive_.arm()) {
          case ArmResult::Ok:
            break;
          case ArmResult::NotCentred:
            reason = "Let go of the joystick to arm";
            break;
          case ArmResult::NoLink:
            reason = "Waiting for a steady connection - try again";
            break;
          case ArmResult::Faulted:
            reason = "Press RESET first";
            break;
          case ArmResult::HardwareError:
            reason = "Motor/encoder hardware failed to start - see serial log";
            break;
        }
        if (reason != nullptr) {
          char buf[96];
          snprintf(buf, sizeof(buf), "{\"msg\":\"%s\"}", reason);
          client->text(buf);
        }
      }
      break;
    default:
      break;
  }
}

void WebControl::sendHello(AsyncWebSocketClient* client) {
  char buf[256];
  int n = snprintf(buf, sizeof(buf), "{\"hello\":%u,\"maxMps\":%.2f,\"names\":[", static_cast<unsigned>(client->id()),
                   config::kMaxWheelSpeedMps);
  for (int i = 0; i < config::kWheelCount && n > 0 && n < static_cast<int>(sizeof(buf)); ++i) {
    n += snprintf(buf + n, sizeof(buf) - n, "%s\"%s\"", i ? "," : "", config::kWheels[i].name);
  }
  if (n > 0 && n < static_cast<int>(sizeof(buf)) - 2) {
    buf[n++] = ']';
    buf[n++] = '}';
    buf[n] = '\0';
    client->text(buf);
  }
}

void WebControl::sendTelemetry() {
  const DriveStatus s = drive_.status();
  char buf[384];
  int n = snprintf(buf, sizeof(buf),
                   "{\"st\":\"%s\",\"fw\":%d,\"link\":%d,\"lim\":%.2f,\"own\":%u,\"hw\":%d,\"rssi\":%d,"
                   "\"ovr\":%u,\"loopUs\":%u,\"w\":[",
                   stateName(s.state), s.faultWheel, s.linkOk ? 1 : 0, s.speedLimit, static_cast<unsigned>(ownerId_.load()),
                   s.hardwareOk ? 1 : 0, WiFi.getMode() == WIFI_STA ? static_cast<int>(WiFi.RSSI()) : 0,
                   static_cast<unsigned>(s.loopOverruns), static_cast<unsigned>(s.maxLoopUs));
  for (int i = 0; i < config::kWheelCount && n > 0 && n < static_cast<int>(sizeof(buf)); ++i) {
    n += snprintf(buf + n, sizeof(buf) - n, "%s[%.2f,%.2f,%.2f]", i ? "," : "", s.wheels[i].targetMps,
                  s.wheels[i].speedMps, s.wheels[i].duty);
  }
  if (n > 0 && n < static_cast<int>(sizeof(buf)) - 2) {
    buf[n++] = ']';
    buf[n++] = '}';
    buf[n] = '\0';
    ws_.textAll(buf, n);
  }
}

void WebControl::loop(uint32_t nowMs) {
  if (nowMs - lastTelemetryMs_ >= kTelemetryPeriodMs) {
    lastTelemetryMs_ = nowMs;
    if (ws_.count() > 0) sendTelemetry();
  }
  if (nowMs - lastCleanupMs_ >= 1000) {
    lastCleanupMs_ = nowMs;
    ws_.cleanupClients();
  }
}
