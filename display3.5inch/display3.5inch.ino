// Pick-to-light firmware — Edge gateway contract (warehouse/v1 +
// light/.../command). Protocol copy: contract/EDGE_LIGHT_PROTOCOL.md and
// contract/edge-light-examples.json
//
// Flash settings for this board are fairly specific
//
// Board: ESP32S3 Dev Module
// Flash Size: 16MB (128Mb)
// Partition Scheme: "8M with spiffs (3MB APP/1.5MB SPIFFS)"
// PSRAM: "OPI PSRAM"
// Upload Speed: "921600"

// Import needed libraries
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <DTF_ESP32Update.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <algorithm>
#include <esp_system.h>
#include <time.h>
#include "LedManager.h"
#include "EpdManager.h"

LedManager leds;
EpdManager epd;

// Factory provisioning (re-flash to change). See contract/EDGE_LIGHT_PROTOCOL.md
const char *WIFI_SSID = "Teton";
const char *WIFI_PASSWORD = "tetonfuel";
// Empty host = Wi-Fi gateway IP (warehouse edge default). Or hostname/IP for test broker.
const char *MQTT_BROKER_HOST = "mqttbroker.tetontechnology.com";
const uint16_t MQTT_BROKER_PORT = 1883;
// Edge deployment: leave username and password empty. MQTTX / test broker: set both.
const char *MQTT_USERNAME = "";
const char *MQTT_PASSWORD = "";
String P2L_MODEL = "elecrow-epaper-3.5";
unsigned long HEARTBEAT_PERIOD_MS = 300000; // 5 minutes for battery devices
String CURRENT_VERSION = "1.0.3";
String DTF_PRODUCT_ID = "e2e5ee12-254d-458c-883c-7f9db5215e0e";
String deviceId;
String sessionId;
uint32_t deviceSequence = 0;
bool clockSynced = false;
bool ntpStarted = false;
unsigned long offlineAfterMs;
unsigned long lastNtpAttemptMs = 0;
const unsigned long NTP_RETRY_MS = 30000;

// Flag for software update
bool needToUpdate = false;

// Pin for measuring battery voltage
const int BATTERY_PIN = 3;

// Divider ratio of the circuit
const float DIVIDER_RATIO = 4.5;

// Variable for displaying the voltage
float v = 0.0;

bool mqttWasConnected = false;
bool mqttBrokerConfigured = false;

// Timers to help with state transitions
unsigned long lastMqttAttemptMs = 0;
unsigned long lastWifiAttemptMs = 0;
const unsigned long WIFI_RETRY_MS = 10000;

// Timers for heartbeat
unsigned long lastHeartbeatMs = 0;

String HeartbeatTopic;
String CommandTopic;
String gatewayId;
String wifiID;

// Mqtt topics
const char *gatewayTopic = "warehouse/v1/gateway";
const char *commandTopic = nullptr;
const char *heartbeatTopic = nullptr;

// Setup WiFi Client
WiFiClient Wifi_net;
PubSubClient mqttClient(Wifi_net);

// Forward declarations
void remove();
float readBatteryVoltage();
void buildDeviceId();
void updateMqttBrokerFromNetwork();
void onTransportLost();
bool ensureFactoryWiFi();
bool publishHeartbeatEvent(const char *topic, const uint8_t *payload,
                           unsigned int length);

// callback helpers
volatile bool mqttMsgReady = false;
char mqttTopicBuff[256];
char mqttPayloadBuff[1024];

/////////////////////////////// Battery Voltage Reading //////////////////////////////////
float readBatteryVoltage() {
  long totalMv = 0;
  const int samples = 20;

  for (int i = 0; i < samples; i++) {
    totalMv += analogReadMilliVolts(BATTERY_PIN);
    delay(2);
  }

  float pinMv = totalMv / (float)samples;
  float pinVoltage = pinMv / 1000.0;
  float batteryVoltage = pinVoltage * DIVIDER_RATIO + .2;

  Serial.println(pinMv);
  Serial.println(pinVoltage, 6);
  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage, 6);
  Serial.println(" V");

  return batteryVoltage;
}

void buildDeviceId() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char idBuffer[17];
  snprintf(idBuffer, sizeof(idBuffer), "p2l-%02x%02x%02x%02x%02x%02x", mac[0],
           mac[1], mac[2], mac[3], mac[4], mac[5]);
  deviceId = String(idBuffer);
  wifiID = deviceId;
  CommandTopic = "light/" + deviceId + "/command";
  commandTopic = CommandTopic.c_str();
}

void updateMqttBrokerFromNetwork() {
  if (MQTT_BROKER_HOST == nullptr || MQTT_BROKER_HOST[0] == '\0') {
    IPAddress broker = WiFi.gatewayIP();
    if (broker == IPAddress(0, 0, 0, 0)) {
      Serial.println("MQTT broker address not available yet");
      return;
    }
    mqttClient.setServer(broker, MQTT_BROKER_PORT);
    Serial.print("MQTT broker (gateway): ");
    Serial.println(broker);
  } else {
    mqttClient.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    Serial.print("MQTT broker (host): ");
    Serial.println(MQTT_BROKER_HOST);
  }
  mqttBrokerConfigured = true;
}

static bool mqttConnectWithCredentials() {
  if (MQTT_USERNAME != nullptr && MQTT_USERNAME[0] != '\0') {
    return mqttClient.connect(wifiID.c_str(), MQTT_USERNAME, MQTT_PASSWORD);
  }
  return mqttClient.connect(wifiID.c_str());
}

void onTransportLost() {
  gatewayId = "";
  HeartbeatTopic = "";
  heartbeatTopic = nullptr;
  ntpStarted = false;
  clockSynced = false;
  lastHeartbeatMs = 0;
  epd.setConnectedScreenShown(false);
  mqttWasConnected = false;
}

bool ensureFactoryWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }
  if (millis() - lastWifiAttemptMs < WIFI_RETRY_MS) {
    return false;
  }
  lastWifiAttemptMs = millis();
  Serial.println("WiFi reconnect (factory credentials)...");
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  return false;
}

bool publishHeartbeatEvent(const char *topic, const uint8_t *payload,
                           unsigned int length) {
  // Contract QoS 1; PubSubClient publishes at QoS 0 (no API on stock 2.x).
  return mqttClient.publish(topic, payload, length, false);
}

// Wifi callback for digesting commands over Mqtt
void WiFi_callback(char *topic, byte *payload, unsigned int length) {
  // Copy topic
  strncpy(mqttTopicBuff, topic, sizeof(mqttTopicBuff));
  mqttTopicBuff[sizeof(mqttTopicBuff) - 1] = '\0';

  // Copy payload
  size_t n = (length < sizeof(mqttPayloadBuff) - 1)
                 ? length
                 : (sizeof(mqttPayloadBuff) - 1);
  memcpy(mqttPayloadBuff, payload, n);
  mqttPayloadBuff[n] = '\0';

  mqttMsgReady = true;
}

void handleMqttMessage() {
  if (!mqttMsgReady)
    return;
  mqttMsgReady = false;

  if (strcmp(mqttTopicBuff, gatewayTopic) == 0) {
    StaticJsonDocument<512> gatewayDoc;

    DeserializationError err = deserializeJson(gatewayDoc, mqttPayloadBuff);

    if (err) {
      Serial.print("Gateway JSON parse failed: ");
      Serial.println(err.c_str());
      return;
    }

    const char *id = gatewayDoc["gatewayId"] | "";

    if (id[0] == '\0') {
      Serial.println("Gateway announcement missing gatewayId");
      return;
    }

    gatewayId = id;

    HeartbeatTopic =
        "warehouse/v1/" + gatewayId + "/devices/" + deviceId + "/events";
    heartbeatTopic = HeartbeatTopic.c_str();
    lastHeartbeatMs = 0;

    Serial.print("Heartbeat Topic: ");
    Serial.println(HeartbeatTopic);

    Serial.print("Gateway ID received: ");
    Serial.println(gatewayId);
    
    return;
  }

  if (commandTopic == nullptr || strcmp(mqttTopicBuff, commandTopic) != 0) {
    return;
  }

  StaticJsonDocument<1024> doc;

  DeserializationError err = deserializeJson(doc, mqttPayloadBuff);

  if (err) {
    return;
  }

  JsonObject payload = doc["payload"].as<JsonObject>();
  if (payload.isNull()) {
    return;
  }

  const char *cmd = payload["cmd"] | "";
  if (cmd[0] == '\0') {
    return;
  }

  else if (strcmp(cmd, "update") == 0) {
    needToUpdate = true;
    return;
  }

  else if (strcmp(cmd, "remove") == 0) {
    remove();
    return;
  }

   else leds.handleLightPayload(payload);
}

void mqttConnectAttempt() {
  if (mqttClient.connected())
    return;

  if (!mqttBrokerConfigured) {
    updateMqttBrokerFromNetwork();
    if (!mqttBrokerConfigured)
      return;
  }

  if (millis() - lastMqttAttemptMs < 5000)
    return;

  lastMqttAttemptMs = millis();

  Serial.println("Attempting MQTT connect...");

  if (mqttConnectWithCredentials()) {
    mqttClient.subscribe(gatewayTopic, 1);
    if (commandTopic != nullptr) {
      mqttClient.subscribe(commandTopic, 0);
    }
    gatewayId = "";
    lastHeartbeatMs = 0;

    Serial.println("MQTT connected + subscribed");

  } else {
    Serial.print("MQTT connect failed, state=");
    Serial.println(mqttClient.state());
  }
}

void remove() {
  Serial.println("Remove command received");

  leds.stopLedEffects();
  leds.turnOff();

  mqttClient.disconnect();
  lastMqttAttemptMs = 0;
  lastHeartbeatMs = 0;

  generateSessionId();
  deviceSequence = 0;
  gatewayId = "";
  HeartbeatTopic = "";
  heartbeatTopic = nullptr;
  epd.setConnectedScreenShown(false);
  mqttWasConnected = false;
}

void generateSessionId() {
  uint32_t randomHigh = esp_random();
  uint32_t randomLow = esp_random();

  char buffer[17];
  snprintf(buffer, sizeof(buffer), "%08x%08x", randomHigh, randomLow);
  sessionId = String(buffer);
}

void heartbeat() {
  if (!clockSynced) {
    return;
  }

  if (gatewayId.length() == 0) {
    return;
  }

  if (heartbeatTopic == nullptr) {
    return;
  }

  if (!mqttClient.connected()) {
    return;
  }

  if (millis() - lastHeartbeatMs < HEARTBEAT_PERIOD_MS) {
    return;
  }

  lastHeartbeatMs = millis();

  deviceSequence++;

  v = readBatteryVoltage();
  Serial.println(v);

  epd.setCurrentRSSI(WiFi.RSSI());

  StaticJsonDocument<512> doc;

  char edgeEventId[200];
  formatEdgeEventId(edgeEventId, sizeof(edgeEventId));

  doc["version"] = 1;
  doc["type"] = "device_heartbeat";
  doc["edgeEventId"] = edgeEventId;
  doc["deviceId"] = deviceId.c_str();
  doc["deviceSequence"] = deviceSequence;
  doc["occurredAt"] = nowEpochMs();

  JsonObject payload = doc.createNestedObject("payload");

  payload["nodeType"] = "pick_to_light";
  payload["name"] = deviceId.c_str();
  payload["offlineAfterMs"] = offlineAfterMs;
  payload["sessionId"] = sessionId.c_str();
  payload["model"] = P2L_MODEL.c_str();
  payload["firmwareVersion"] = CURRENT_VERSION.c_str();

  String ssid = WiFi.SSID();
  String ip = WiFi.localIP().toString();

  if (ssid.length() > 0)
    payload["ssid"] = ssid;

  if (WiFi.status() == WL_CONNECTED && ip.length() > 0) {
    payload["ip"] = ip;
  }
  if (WiFi.status() == WL_CONNECTED) {
    payload["rssi"] = epd.getCurrentRSSI();
  }

  if (v > 0.05f) {
    payload["voltage"] = v;
  }

  char out[768];
  size_t n = serializeJson(doc, out, sizeof(out));

  Serial.println(out);

  bool ok = publishHeartbeatEvent(heartbeatTopic, (const uint8_t *)out,
                                  (unsigned int)n);
  Serial.print("Heartbeat publish: ");
  Serial.println(ok ? "ok" : "failed");
}

void formatEdgeEventId(char *out, size_t outLen) {
  snprintf(out, outLen, "%s:%s:%lu", deviceId.c_str(), sessionId.c_str(),
           (unsigned long)deviceSequence);
}

int64_t nowEpochMs() { return (int64_t)time(nullptr) * 1000LL; }

bool ensureClockSynced() {
  if (clockSynced) {
    return true;
  }

  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  if (!ntpStarted) {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    ntpStarted = true;
  }

  unsigned long startTime = millis();

  while (time(nullptr) <= 1700000000) {
    if (millis() - startTime >= 20000) {
      Serial.println("NTP sync timed out");
      return false;
    }

    delay(100);
  }

  clockSynced = true;

  Serial.print("NTP synced. Unix time: ");
  Serial.println((long)time(nullptr));

  return true;
}

// ----------------------------------------------------
// SETUP
// ----------------------------------------------------
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db);

  leds.init();
  epd.init();

  mqttClient.setCallback(WiFi_callback);
  mqttClient.setBufferSize(1024);

  epd.showBootScreenBase();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  buildDeviceId();

  Serial.print("Command topic: ");
  Serial.println(CommandTopic);
  generateSessionId();
  Serial.println(deviceId);
  Serial.println(sessionId);

  offlineAfterMs = max(15000UL, 3UL * HEARTBEAT_PERIOD_MS);

  uint32_t bootStartMs = millis();
  uint32_t lastDotMs = 0;
  uint8_t dotCount = 0;

  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastDotMs >= 400) {
      lastDotMs = millis();
      dotCount = (dotCount + 1) % 4;
      epd.updateBootDots(dotCount);
    }
    delay(10);
    if (millis() - bootStartMs > 60000) {
      bootStartMs = millis();
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }

  updateMqttBrokerFromNetwork();
}

// ----------------------------------------------------
// LOOP
// ----------------------------------------------------
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    onTransportLost();
    mqttBrokerConfigured = false;
    ensureFactoryWiFi();
    leds.ledTick();
    return;
  }

  if (!mqttBrokerConfigured) {
    updateMqttBrokerFromNetwork();
  }

  if (!clockSynced) {
    if (millis() - lastNtpAttemptMs >= NTP_RETRY_MS) {
      lastNtpAttemptMs = millis();
      ensureClockSynced();
    }
  }

  mqttClient.loop();
  handleMqttMessage();

  mqttConnectAttempt();

  if (mqttWasConnected && !mqttClient.connected()) {
    onTransportLost();
  }
  mqttWasConnected = mqttClient.connected();

  heartbeat();

  leds.ledTick();

  if (needToUpdate) {
    leds.fadeLed("magenta");
  } else if (!mqttClient.connected()) {
    leds.blinkLed("green", 333);
  } else if (!clockSynced || gatewayId.length() == 0) {
    leds.blinkLed("cyan", 333);
  } else {
    leds.fadeLed("green");
  }

  if (!epd.getConnectedScreenShown() && mqttClient.connected() && clockSynced &&
      gatewayId.length() > 0) {
    epd.setConnectedScreenShown(true);
    epd.showConnectedDashboard(WiFi.SSID().c_str(), WiFi.localIP(), WiFi.RSSI(), deviceId, CURRENT_VERSION, v);
  }

  if (needToUpdate) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Updating Firmware");

      epd.showUpdatingScreenBase();

      for (uint8_t i = 0; i < 4; i++) {
        epd.updateUpdatingDots(i % 4); // 0,1,2,3
        delay(300);
      }

      auto response = DTF_ESP32Update::getFirmwareUpdate(
          DTF_PRODUCT_ID.c_str(), CURRENT_VERSION.c_str());

      Serial.print("DTF Response: ");
      Serial.println((int)response);

      needToUpdate = false;
    } else {
      Serial.println("Software Update Pending. Need to connect to internet "
                     "before update.");
    }
  }
}