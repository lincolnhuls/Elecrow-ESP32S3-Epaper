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
#include "EPD.h"
#include "EPD_GUI.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <DTF_ESP32Update.h>
#include <Display_EPD_W21.h>
#include <FastLED.h>
#include <PubSubClient.h>
#include <QRCodeGenerator.h>
#include <WiFi.h>
#include <algorithm>
#include <esp_system.h>
#include <time.h>

// Factory provisioning (re-flash to change). See contract/EDGE_LIGHT_PROTOCOL.md
const char *WIFI_SSID = "Teton";
const char *WIFI_PASSWORD = "tetonfuel";
// Empty = use Wi-Fi gateway IP as MQTT broker (contract default)
const char *MQTT_BROKER_HOST = "";
const uint16_t MQTT_BROKER_PORT = 1883;
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

// RSSI values for comparing and updating screen if needed
int compareRSSI = 0;
int currentRSSI = 0;

// Pins for onboard led
#define R 4
#define G 5
#define B 6

// Variables for Connection Status Led
static bool ledState = false;
static unsigned long ledTimer = 0;
static int fadeValue = -1;
static bool fadingDown = false;

// Pin for measuring battery voltage
const int BATTERY_PIN = 3;

// Divider ratio of the circuit
const float DIVIDER_RATIO = 4.5;

// Variable for displaying the voltage
float v = 0.0;

// Pin for led and screen power
#define SCREEN 7
#define LED 14

#define MAX_SLOTS 25
#define MAX_RANGES 25

struct LedRange {
  uint16_t start;
  uint16_t count;
};

enum EffectType : uint8_t {
  EFFECT_NONE = 0,
  EFFECT_ON,
  EFFECT_BLINK,
  EFFECT_BREATHE,
  EFFECT_MASK_OFF
};

struct LedSlot {
  bool active = false;
  EffectType type = EFFECT_NONE;

  // Color and intensity caps
  uint8_t r = 255, g = 255, b = 255;
  uint8_t maxBrightness = 255;

  // Ranges owned by slot
  LedRange ranges[MAX_RANGES];
  uint8_t rangeCount = 0;

  uint32_t lastFrameMs = 0;

  // ON
  uint32_t offAtMs = 0;

  // BLINK
  bool blinkIsOn = false;
  uint16_t blinkIntervalMs = 500;
  uint16_t blinkRemainingToggles = 0;
  uint32_t blinkNextToggleMs = 0;

  // BREATHE
  uint16_t phase16 = 0;
  uint16_t phaseStep = 1;
};

LedSlot slots[MAX_SLOTS];

// Rotation for easy rotation applications
int ROTATION = 0;

bool connectedScreenShown = false;
bool mqttWasConnected = false;
bool mqttBrokerConfigured = false;

volatile bool forceLedRender = false;
const uint16_t BREATHE_FRAME_MS = 15;

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

// Led setup
#define NUM_LEDS 200
#define DATA_PIN 8
CRGB leds[NUM_LEDS];

// display buffer
uint8_t ImageBW[(EPD_W * EPD_H) / 8];
char buffer[200];

// Screen/layout helpers for the new panel
const int SCREEN_MARGIN_X = 10;
const int SCREEN_MARGIN_Y = 10;
const int QR_MARGIN_RIGHT = 8;
const int QR_DEFAULT_SCALE = 4;
const int QR_LARGE_SCALE = 4;
const int QR_SMALL_SCALE = 3;

// Forward declarations
int rssiToBars(long rssi);
void displayQRCodeOnEPD(const char *text, int yOffset = -1, int qrScale = 4);
void drawWiFiIcon(int x, int y, int bars);
void showConnectedDashboard();
void showUpdatingScreenBase();
void updateUpdatingDots(uint8_t dotCount);
void remove();
void showBootScreenBase();
void updateBootDots(uint8_t dotCount);
float readBatteryVoltage();
void fadeLed(const char *color);
void blinkLed(const char *color, int on_off_time);
void handleLightPayload(JsonObject doc);
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

////////////////////////////////////// Led Functions /////////////////////////////////////////

// Functions for status led
void fadeLed(const char *color) {
  static uint32_t lastFadeMs = 0;
  if (millis() - lastFadeMs < 8)
    return; // ~120 steps per second
  lastFadeMs = millis();

  if (fadeValue < 1)
    fadingDown = false;
  else if (fadeValue > 250)
    fadingDown = true;

  fadeValue += (fadingDown ? -1 : +1);

  uint8_t off = 255;
  uint8_t on = (uint8_t)fadeValue;

  if (strcmp(color, "blue") == 0) {
    analogWrite(B, on);
    analogWrite(R, off);
    analogWrite(G, off);
  } else if (strcmp(color, "green") == 0) {
    analogWrite(B, off);
    analogWrite(R, off);
    analogWrite(G, on);
  } else if (strcmp(color, "red") == 0) {
    analogWrite(B, off);
    analogWrite(R, on);
    analogWrite(G, off);
  } else if (strcmp(color, "cyan") == 0) {
    analogWrite(B, on);
    analogWrite(R, off);
    analogWrite(G, on);
  } else if (strcmp(color, "magenta") == 0) {
    analogWrite(B, on);
    analogWrite(R, on);
    analogWrite(G, off);
  } else if (strcmp(color, "yellow") == 0) {
    analogWrite(B, off);
    analogWrite(R, on);
    analogWrite(G, on);
  }
}

void blinkLed(const char *color, int on_off_time) {
  if (millis() - ledTimer <= (unsigned long)on_off_time)
    return;
  ledTimer = millis();
  ledState = !ledState;

  uint8_t dim = ledState ? 50 : 255;
  uint8_t off = 255;

  analogWrite(R, off);
  analogWrite(G, off);
  analogWrite(B, off); // clear first

  if (strcmp(color, "blue") == 0) {
    analogWrite(B, dim);
  } else if (strcmp(color, "green") == 0) {
    analogWrite(G, dim);
  } else if (strcmp(color, "red") == 0) {
    analogWrite(R, dim);
  } else if (strcmp(color, "cyan") == 0) {
    analogWrite(B, dim);
    analogWrite(G, dim);
  } else if (strcmp(color, "magenta") == 0) {
    analogWrite(B, dim);
    analogWrite(R, dim);
  } else if (strcmp(color, "yellow") == 0) {
    analogWrite(R, dim);
    analogWrite(G, dim);
  }
}

// Slot creation so multiple areas on the cart can be on and updated at the same
// time

// ---------------------- Slot lifecycle ----------------------

static void clearSlot(uint8_t i) {
  if (i >= MAX_SLOTS)
    return;
  slots[i].active = false;
  slots[i].type = EFFECT_NONE;
  slots[i].rangeCount = 0;

  slots[i].offAtMs = 0;

  slots[i].blinkIsOn = false;
  slots[i].blinkIntervalMs = 500;
  slots[i].blinkRemainingToggles = 0;
  slots[i].blinkNextToggleMs = 0;

  slots[i].phase16 = 0;
  slots[i].phaseStep = 1;
}

static bool subtractOneRange(const LedRange &src, const LedRange &cut,
                             LedRange *out, uint8_t &outN) {
  // returns up to 2 ranges in out (appends); true if src changed
  uint32_t s0 = src.start;
  uint32_t s1 = (uint32_t)src.start + (uint32_t)src.count; // exclusive
  uint32_t c0 = cut.start;
  uint32_t c1 = (uint32_t)cut.start + (uint32_t)cut.count; // exclusive

  // no overlap
  if (!(s0 < c1 && c0 < s1)) {
    if (outN < MAX_RANGES)
      out[outN++] = src;
    return false;
  }

  // left remainder [s0, min(s1,c0))
  uint32_t left1 = (c0 < s1) ? c0 : s1;
  if (s0 < left1) {
    if (outN < MAX_RANGES)
      out[outN++] = {(uint16_t)s0, (uint16_t)(left1 - s0)};
  }

  // right remainder [max(s0,c1), s1)
  uint32_t right0 = (c1 > s0) ? c1 : s0;
  if (right0 < s1) {
    if (outN < MAX_RANGES)
      out[outN++] = {(uint16_t)right0, (uint16_t)(s1 - right0)};
  }

  return true;
}

static bool subtractCutsFromSlotByIndex(uint8_t slotIdx, const LedRange *cuts,
                                        uint8_t cutN) {
  if (slotIdx >= MAX_SLOTS)
    return false;
  LedSlot &s = slots[slotIdx];
  if (!s.active)
    return false;

  // run subtract, but clear by known index
  LedRange working[MAX_RANGES];
  uint8_t workingN = 0;

  uint8_t srcN = s.rangeCount;
  if (srcN > MAX_RANGES)
    srcN = MAX_RANGES;
  for (uint8_t i = 0; i < srcN; i++)
    working[workingN++] = s.ranges[i];

  bool changed = false;
  if (cutN > MAX_RANGES)
    cutN = MAX_RANGES;

  for (uint8_t c = 0; c < cutN; c++) {
    LedRange next[MAX_RANGES];
    uint8_t nextN = 0;

    for (uint8_t i = 0; i < workingN; i++) {
      bool didChange = subtractOneRange(working[i], cuts[c], next, nextN);
      changed = changed || didChange;
      if (nextN >= MAX_RANGES)
        break;
    }

    workingN = nextN;
    for (uint8_t i = 0; i < workingN; i++)
      working[i] = next[i];

    if (workingN == 0)
      break;
  }

  if (workingN == 0) {
    clearSlot(slotIdx);
    return true;
  }

  if (changed) {
    s.rangeCount = workingN;
    for (uint8_t i = 0; i < workingN; i++)
      s.ranges[i] = working[i];
  }

  return changed;
}

// Newest command wins per LED: trim overlapping ranges in other slots only.
static void applyPerLedReplacement(const LedRange *list, uint8_t listCount) {
  if (!list || listCount == 0)
    return;
  if (listCount > MAX_RANGES)
    listCount = MAX_RANGES;

  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active)
      continue;
    subtractCutsFromSlotByIndex(i, list, listCount);
  }
}

// ---------------------- LED write helpers ----------------------

static inline uint8_t scale8u(uint8_t v, uint8_t scale) {
  // FastLED has scale8(), but keeping local avoids extra includes/assumptions.
  return (uint8_t)(((uint16_t)v * (uint16_t)scale) / 255u);
}

// Writes a solid color into leds[] for a single range (no FastLED.show here)
static void fillRangeRGB(uint16_t start, uint16_t count, uint8_t r, uint8_t g,
                         uint8_t b) {
  uint32_t end = (uint32_t)start + (uint32_t)count;
  if (start >= NUM_LEDS)
    return;
  if (end > NUM_LEDS)
    end = NUM_LEDS;

  for (uint32_t i = start; i < end; i++) {
    leds[i].setRGB(r, g, b);
  }
}

// helper: copy ranges into slot (clamps to MAX_RANGES)
static void copyRangesToSlot(LedSlot &s, const LedRange *ranges, uint8_t n) {
  if (n > MAX_RANGES)
    n = MAX_RANGES;
  s.rangeCount = n;
  for (uint8_t i = 0; i < n; i++)
    s.ranges[i] = ranges[i];
}

// Pick a slot. Prefer an inactive one; otherwise steal the first overlapping
// one; otherwise steal slot 0.
static bool rangesOverlap(const LedRange *a, uint8_t aN, const LedRange *b,
                          uint8_t bN) {
  if (aN > MAX_RANGES)
    aN = MAX_RANGES;
  if (bN > MAX_RANGES)
    bN = MAX_RANGES;

  for (uint8_t i = 0; i < aN; i++) {
    uint16_t a0 = a[i].start;
    uint16_t a1 = a0 + a[i].count; // exclusive
    for (uint8_t j = 0; j < bN; j++) {
      uint16_t b0 = b[j].start;
      uint16_t b1 = b0 + b[j].count; // exclusive
      if (a0 < b1 && b0 < a1)
        return true;
    }
  }
  return false;
}

static int8_t allocateSlotFor(const LedRange *newRanges, uint8_t newN) {
  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active)
      return (int8_t)i;
  }
  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    if (rangesOverlap(slots[i].ranges, slots[i].rangeCount, newRanges, newN))
      return (int8_t)i;
  }
  return 0;
}

void on(uint8_t red, uint8_t green, uint8_t blue, int16_t duration_s,
        uint8_t brightness, const LedRange *ranges, uint8_t n) {
  if (!ranges || n == 0)
    return;

  applyPerLedReplacement(ranges, n);
  int8_t idx = allocateSlotFor(ranges, n);
  clearSlot(idx);

  LedSlot &s = slots[idx];
  s.active = true;
  s.type = EFFECT_ON;

  s.r = red;
  s.g = green;
  s.b = blue;
  s.maxBrightness = brightness;

  copyRangesToSlot(s, ranges, n);

  s.offAtMs = (duration_s < 0) ? 0 : (millis() + (uint32_t)duration_s * 1000UL);

  // initialize timing fields so ledTick updates cleanly
  s.lastFrameMs = millis();
  s.blinkNextToggleMs = 0;
}

void blink(uint8_t red, uint8_t green, uint8_t blue, uint16_t interval_ms,
           uint16_t times, uint8_t brightness, const LedRange *ranges,
           uint8_t n) {
  if (!ranges || n == 0)
    return;

  applyPerLedReplacement(ranges, n);
  int8_t idx = allocateSlotFor(ranges, n);
  clearSlot(idx);

  LedSlot &s = slots[idx];
  s.active = true;
  s.type = EFFECT_BLINK;

  s.r = red;
  s.g = green;
  s.b = blue;
  s.maxBrightness = brightness;

  copyRangesToSlot(s, ranges, n);

  s.blinkIntervalMs = interval_ms;
  s.blinkIsOn = true;
  s.blinkNextToggleMs = millis() + interval_ms;
  s.blinkRemainingToggles = (times == 0) ? 0 : (uint16_t)times * 2;

  s.offAtMs = 0;
  s.lastFrameMs = millis();
}

void breathe(uint8_t red, uint8_t green, uint8_t blue,
             int16_t duration_s, // HALF cycle seconds (same meaning as before)
             uint8_t brightness, const LedRange *ranges, uint8_t n) {
  if (!ranges || n == 0)
    return;

  applyPerLedReplacement(ranges, n);
  int8_t idx = allocateSlotFor(ranges, n);
  clearSlot(idx);

  uint32_t halfMs = (uint32_t)max<int16_t>(1, duration_s) * 1000UL;
  uint32_t cycleMs = halfMs * 2UL;
  if (cycleMs == 0)
    cycleMs = 1;

  LedSlot &s = slots[idx];
  s.active = true;
  s.type = EFFECT_BREATHE;

  s.r = red;
  s.g = green;
  s.b = blue;
  s.maxBrightness = brightness;

  copyRangesToSlot(s, ranges, n);

  s.phaseStep = (uint16_t)((65536UL * BREATHE_FRAME_MS) / cycleMs);
  if (s.phaseStep == 0)
    s.phaseStep = 1;

  s.phase16 = (uint16_t)(64 << 8); // start bright
  s.lastFrameMs = millis();

  s.offAtMs = 0;
}

void stopLedEffects() {
  for (uint8_t i = 0; i < MAX_SLOTS; i++)
    clearSlot(i);
}

// ---------------------- Slot state update ----------------------

static bool updateSlotState(LedSlot &s, uint32_t now) {
  if (!s.active)
    return false;

  // Expire timed effects (ON and OFF mask)
  if ((s.type == EFFECT_ON || s.type == EFFECT_MASK_OFF) && s.offAtMs != 0 &&
      (int32_t)(now - s.offAtMs) >= 0) {
    s.active = false;
    return true;
  }

  // Blink: toggle on schedule
  if (s.type == EFFECT_BLINK) {
    if ((int32_t)(now - s.blinkNextToggleMs) >= 0) {
      s.blinkNextToggleMs = now + s.blinkIntervalMs;
      s.blinkIsOn = !s.blinkIsOn;

      if (s.blinkRemainingToggles > 0) {
        s.blinkRemainingToggles--;
        if (s.blinkRemainingToggles == 0) {
          s.active = false;
        }
      }
      return true;
    }
    return false;
  }

  // Breathe: advance phase on frame cadence
  if (s.type == EFFECT_BREATHE) {
    uint32_t elapsed = now - s.lastFrameMs;
    if (elapsed < BREATHE_FRAME_MS)
      return false;

    uint32_t frames = elapsed / BREATHE_FRAME_MS;
    if (frames > 3)
      frames = 3;
    s.lastFrameMs += frames * BREATHE_FRAME_MS;
    s.phase16 += (uint16_t)(frames * s.phaseStep);
    return true;
  }

  return false;
}

// Convert slot -> brightness scale 0..255
static uint8_t slotLevel(const LedSlot &s) {
  if (!s.active)
    return 0;

  if (s.type == EFFECT_ON) {
    return s.maxBrightness; // constant
  }

  if (s.type == EFFECT_BLINK) {
    return s.blinkIsOn ? s.maxBrightness : 0;
  }

  if (s.type == EFFECT_BREATHE) {
    uint8_t phase8 = (uint8_t)(s.phase16 >> 8);
    uint8_t wave = sin8(phase8); // 0..255..0
    return (uint8_t)((uint32_t)s.maxBrightness * wave / 255UL);
  }

  return 0;
}

// ---------------------- Multi-slot tick + single show ----------------------

void ledTick() {
  uint32_t now = millis();

  bool anyActive = false;
  bool needRender = forceLedRender;

  // Update slot state machines
  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active)
      continue;
    anyActive = true;

    if (updateSlotState(slots[i], now)) {
      needRender = true;
    }
  }

  // If nothing is active, we may still need to clear the strip
  if (!anyActive) {
    if (needRender) {
      FastLED.clear();
      FastLED.show();
      forceLedRender = false;
    }
    return;
  }

  // If nothing changed and no forced render, skip
  if (!needRender)
    return;

  // Rebuild the LED buffer from all active (non-mask) slots
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }

  // 1) Render normal effects first (ON/BLINK/BREATHE)
  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    const LedSlot &s = slots[i];
    if (!s.active)
      continue;
    if (s.type == EFFECT_MASK_OFF)
      continue; // skip masks for now

    uint8_t level = slotLevel(s);
    if (level == 0)
      continue;

    uint8_t rr = scale8u(s.r, level);
    uint8_t gg = scale8u(s.g, level);
    uint8_t bb = scale8u(s.b, level);

    uint8_t n = s.rangeCount;
    if (n > MAX_RANGES)
      n = MAX_RANGES;

    for (uint8_t k = 0; k < n; k++) {
      fillRangeRGB(s.ranges[k].start, s.ranges[k].count, rr, gg, bb);
    }
  }

  // 2) Apply OFF masks last (force black in those ranges)
  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    const LedSlot &s = slots[i];
    if (!s.active)
      continue;
    if (s.type != EFFECT_MASK_OFF)
      continue;

    uint8_t n = s.rangeCount;
    if (n > MAX_RANGES)
      n = MAX_RANGES;

    for (uint8_t k = 0; k < n; k++) {
      fillRangeRGB(s.ranges[k].start, s.ranges[k].count, 0, 0, 0);
    }
  }

  FastLED.show();
  forceLedRender = false;
}
///////////////////////////////////////////////////////////////////////////////////////////

// -------- ledList parsing + overlap helpers --------

static uint8_t parseLedList(JsonArray list, LedRange *out, uint8_t outMax) {
  uint8_t n = 0;
  for (JsonVariant v : list) {
    JsonArray pair = v.as<JsonArray>();
    if (pair.isNull() || pair.size() < 2)
      continue;

    uint16_t start = pair[0] | 0;
    uint16_t count = pair[1] | 0;
    if (count == 0)
      continue;
    if (start >= NUM_LEDS)
      continue;

    // clamp to strip
    uint16_t maxCount = (NUM_LEDS - start);
    if (count > maxCount)
      count = maxCount;

    out[n++] = {start, count};
    if (n >= outMax)
      break;
  }
  return n;
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
  IPAddress broker;
  if (MQTT_BROKER_HOST == nullptr || MQTT_BROKER_HOST[0] == '\0') {
    broker = WiFi.gatewayIP();
  } else {
    broker.fromString(MQTT_BROKER_HOST);
  }
  if (broker == IPAddress(0, 0, 0, 0)) {
    Serial.println("MQTT broker address not available yet");
    return;
  }
  mqttClient.setServer(broker, MQTT_BROKER_PORT);
  mqttBrokerConfigured = true;
  Serial.print("MQTT broker: ");
  Serial.println(broker);
}

void onTransportLost() {
  gatewayId = "";
  HeartbeatTopic = "";
  heartbeatTopic = nullptr;
  ntpStarted = false;
  clockSynced = false;
  lastHeartbeatMs = 0;
  connectedScreenShown = false;
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

void handleLightPayload(JsonObject doc) {
  const char *cmd = doc["cmd"] | "";
  if (cmd[0] == '\0') {
    return;
  }

  if (strcmp(cmd, "update") == 0) {
    needToUpdate = true;
    return;
  }

  if (strcmp(cmd, "remove") == 0) {
    remove();
    forceLedRender = true;
    return;
  }

  if (strcmp(cmd, "on") == 0) {
    digitalWrite(LED, HIGH);
    uint8_t r = doc["r"] | 255;
    uint8_t g = doc["g"] | 255;
    uint8_t b = doc["b"] | 255;
    uint8_t brightness = doc["brightness"] | 255;
    int16_t duration_s = doc["duration"] | -1;

    if (!doc.containsKey("ledList")) {
      return;
    }

    LedRange tmp[MAX_RANGES];
    JsonArray list = doc["ledList"].as<JsonArray>();
    uint8_t n = parseLedList(list, tmp, MAX_RANGES);
    if (n == 0) {
      return;
    }

    on(r, g, b, duration_s, brightness, tmp, n);
    forceLedRender = true;
    return;
  }

  if (strcmp(cmd, "blink") == 0) {
    digitalWrite(LED, HIGH);
    uint8_t r = doc["r"] | 255;
    uint8_t g = doc["g"] | 255;
    uint8_t b = doc["b"] | 255;
    uint16_t interval_ms = doc["interval"] | 500;
    uint16_t times = doc["times"] | 0;
    uint8_t brightness = doc["brightness"] | 255;

    if (!doc.containsKey("ledList")) {
      return;
    }

    LedRange tmp[MAX_RANGES];
    JsonArray list = doc["ledList"].as<JsonArray>();
    uint8_t n = parseLedList(list, tmp, MAX_RANGES);
    if (n == 0) {
      return;
    }

    blink(r, g, b, interval_ms, times, brightness, tmp, n);
    forceLedRender = true;
    return;
  }

  if (strcmp(cmd, "breathe") == 0) {
    digitalWrite(LED, HIGH);
    uint8_t r = doc["r"] | 255;
    uint8_t g = doc["g"] | 255;
    uint8_t b = doc["b"] | 255;
    int16_t duration_s = doc["duration"] | 1;
    uint8_t brightness = doc["brightness"] | 255;

    if (!doc.containsKey("ledList")) {
      return;
    }

    LedRange tmp[MAX_RANGES];
    JsonArray list = doc["ledList"].as<JsonArray>();
    uint8_t n = parseLedList(list, tmp, MAX_RANGES);
    if (n == 0) {
      return;
    }

    breathe(r, g, b, duration_s, brightness, tmp, n);
    forceLedRender = true;
    return;
  }

  if (strcmp(cmd, "off") == 0) {
    bool all = doc["all"] | false;

    if (all || !doc.containsKey("ledList")) {
      digitalWrite(LED, LOW);
      stopLedEffects();
      forceLedRender = true;
      return;
    }

    LedRange tmp[MAX_RANGES];
    JsonArray list = doc["ledList"].as<JsonArray>();
    uint8_t n = parseLedList(list, tmp, MAX_RANGES);
    if (n == 0) {
      return;
    }

    bool changed = false;
    for (uint8_t i = 0; i < MAX_SLOTS; i++) {
      if (!slots[i].active)
        continue;
      if (slots[i].type == EFFECT_MASK_OFF)
        continue;
      if (subtractCutsFromSlotByIndex(i, tmp, n))
        changed = true;
    }

    if (changed)
      forceLedRender = true;
    return;
  }
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

  handleLightPayload(payload);
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

  if (mqttClient.connect(wifiID.c_str())) {
    mqttClient.subscribe(gatewayTopic, 1);
    if (commandTopic != nullptr) {
      mqttClient.subscribe(commandTopic, 0);
    }
    gatewayId = "";
    lastHeartbeatMs = 0;

    Serial.println("MQTT connected + subscribed");

  } else {
    Serial.println("MQTT connected failed");
  }
}

void remove() {
  Serial.println("Remove command received");

  digitalWrite(LED, LOW);
  stopLedEffects();
  forceLedRender = true;

  mqttClient.disconnect();
  lastMqttAttemptMs = 0;
  lastHeartbeatMs = 0;

  generateSessionId();
  deviceSequence = 0;
  gatewayId = "";
  HeartbeatTopic = "";
  heartbeatTopic = nullptr;
  connectedScreenShown = false;
  mqttWasConnected = false;
}

// Pixels - 154, 161, 168

void clearBootDotsInBuffer() {
  // Clear just the dots area in the full-screen buffer
  // Covers the three dots at x = 154, 161, 168
  for (int x = 154; x < 176; x++) {
    for (int y = 10; y < 26; y++) {
      Paint_SetPixel(x, y, WHITE);
    }
  }
}

void showBootScreenBase() {
  Paint_NewImage(ImageBW, EPD_W, EPD_H, ROTATION, WHITE);
  Paint_Clear(WHITE);

  EPD_ShowString(10, 10, "Connecting to WiFi", 16, BLACK);

  // First draw after boot should be full refresh
  EPD_Display(ImageBW);

  // Seed the panel for later partial updates
  EPD_SetRAMValue_BaseMap(ImageBW);
}

void updateBootDots(uint8_t dotCount) {
  // Modify the existing full-screen buffer using the same coordinates
  // that already look correct in your normal drawing code.
  clearBootDotsInBuffer();

  if (dotCount >= 1)
    EPD_ShowString(154, 10, ".", 16, BLACK);
  if (dotCount >= 2)
    EPD_ShowString(161, 10, ".", 16, BLACK);
  if (dotCount >= 3)
    EPD_ShowString(168, 10, ".", 16, BLACK);

  // Partial-refresh using the full buffer
  EPD_Dis_PartAll(ImageBW);
}

void showUpdatingScreenBase() {
  EPD_FastInit();

  Paint_NewImage(ImageBW, EPD_W, EPD_H, ROTATION, WHITE);
  Paint_Clear(WHITE);

  EPD_ShowString(10, 10, "UPDATE DETECTED", 16, BLACK);
  EPD_ShowString(10, 34, "Updating device", 16, BLACK);

  // first draw of this screen should be full
  EPD_Display(ImageBW);

  // seed panel memory for repeated partial updates
  EPD_SetRAMValue_BaseMap(ImageBW);
}

void clearUpdatingDotsInBuffer() {
  for (int x = 132; x < 150; x++) {
    for (int y = 34; y < 50; y++) {
      Paint_SetPixel(x, y, WHITE);
    }
  }
}

void updateUpdatingDots(uint8_t dotCount) {
  clearUpdatingDotsInBuffer();

  if (dotCount >= 1)
    EPD_ShowString(132, 34, ".", 16, BLACK);
  if (dotCount >= 2)
    EPD_ShowString(138, 34, ".", 16, BLACK);
  if (dotCount >= 3)
    EPD_ShowString(144, 34, ".", 16, BLACK);

  EPD_Dis_PartAll(ImageBW);
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

  currentRSSI = WiFi.RSSI();

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
    payload["rssi"] = currentRSSI;
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
  pinMode(SCREEN, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);
  digitalWrite(SCREEN, HIGH);
  digitalWrite(LED, LOW);
  analogWrite(R, 255);
  analogWrite(G, 255);
  analogWrite(B, 255);

  FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);
  stopLedEffects();
  FastLED.show();

  // ----- EPD INIT -----
  EPD_GPIOInit();
  EPD_Init();

  Paint_NewImage(ImageBW, EPD_W, EPD_H, ROTATION, WHITE);
  Paint_Clear(WHITE);

  mqttClient.setCallback(WiFi_callback);
  mqttClient.setBufferSize(1024);

  showBootScreenBase();

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
      updateBootDots(dotCount);
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
    ledTick();
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

  ledTick();

  if (needToUpdate) {
    fadeLed("magenta");
  } else if (!mqttClient.connected()) {
    blinkLed("green", 333);
  } else if (!clockSynced || gatewayId.length() == 0) {
    blinkLed("cyan", 333);
  } else {
    fadeLed("green");
  }

  if (!connectedScreenShown && mqttClient.connected() && clockSynced &&
      gatewayId.length() > 0) {
    connectedScreenShown = true;
    showConnectedDashboard();
  }

  if (needToUpdate) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Updating Firmware");

      showUpdatingScreenBase();

      for (uint8_t i = 0; i < 4; i++) {
        updateUpdatingDots(i % 4); // 0,1,2,3
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

// ----------------------------------------------------
// CONNECTED DASHBOARD
// ----------------------------------------------------
void showConnectedDashboard() {
  EPD_FastInit();

  Paint_NewImage(ImageBW, EPD_W, EPD_H, ROTATION, WHITE);
  Paint_Clear(WHITE);

  EPD_ShowString(10, 10, "CONNECTED", 16, BLACK);

  snprintf(buffer, sizeof(buffer), "SSID: %s", WiFi.SSID().c_str());
  EPD_ShowString(10, 30, buffer, 16, BLACK);

  snprintf(buffer, sizeof(buffer), "IP: %s", WiFi.localIP().toString().c_str());
  EPD_ShowString(10, 50, buffer, 16, BLACK);

  long rssi = WiFi.RSSI();
  compareRSSI = WiFi.RSSI();
  snprintf(buffer, sizeof(buffer), "Signal: %ld dBm", rssi);
  EPD_ShowString(10, 70, buffer, 16, BLACK);

  int bars = rssiToBars(rssi);
  drawWiFiIcon(10, 92, bars);

  snprintf(buffer, sizeof(buffer), "ID: %s", deviceId.c_str());
  EPD_ShowString(10, 126, buffer, 16, BLACK);

  snprintf(buffer, sizeof(buffer), "Ver: %s", CURRENT_VERSION.c_str());
  EPD_ShowString(10, 146, buffer, 16, BLACK);

  snprintf(buffer, sizeof(buffer), "Voltage: %.3f V", v);
  EPD_ShowString(10, 166, buffer, 16, BLACK);

  displayQRCodeOnEPD(deviceId.c_str(), -1, QR_LARGE_SCALE);

  EPD_WhiteScreen_ALL_Fast(ImageBW);
  EPD_DeepSleep();
}

// ----------------------------------------------------
// QR CODE
// ----------------------------------------------------
void displayQRCodeOnEPD(const char *text, int yOffset, int qrScale) {
  QRCode qrcode;
  uint8_t qrcodeData[qrcode_getBufferSize(4)];
  qrcode_initText(&qrcode, qrcodeData, 4, 0, text);

  int scale = qrScale;
  if (scale < 1)
    scale = QR_DEFAULT_SCALE;

  int qrPixSize = qrcode.size * scale;

  // Use actual drawable dimensions from the GUI library
  int displayWidth = Paint.width;
  int displayHeight = Paint.height;

  // Place QR on the right with a small margin
  int pxOffset = displayWidth - qrPixSize - QR_MARGIN_RIGHT;

  // Vertically center unless caller gives a specific y
  int pyOffset;
  if (yOffset >= 0)
    pyOffset = yOffset;
  else
    pyOffset = (displayHeight - qrPixSize) / 2;

  // Clamp into visible area
  if (pxOffset < SCREEN_MARGIN_X)
    pxOffset = SCREEN_MARGIN_X;
  if (pyOffset < SCREEN_MARGIN_Y)
    pyOffset = SCREEN_MARGIN_Y;

  if (pxOffset + qrPixSize >= displayWidth) {
    pxOffset = displayWidth - qrPixSize - 1;
  }
  if (pyOffset + qrPixSize >= displayHeight) {
    pyOffset = displayHeight - qrPixSize - 1;
  }

  if (pxOffset < 0)
    pxOffset = 0;
  if (pyOffset < 0)
    pyOffset = 0;

  for (uint8_t y = 0; y < qrcode.size; y++) {
    for (uint8_t x = 0; x < qrcode.size; x++) {
      bool module = qrcode_getModule(&qrcode, x, y);

      for (int dx = 0; dx < scale; dx++) {
        for (int dy = 0; dy < scale; dy++) {
          int px = pxOffset + x * scale + dx;
          int py = pyOffset + y * scale + dy;

          if (px >= 0 && px < displayWidth && py >= 0 && py < displayHeight) {
            Paint_SetPixel(px, py, module ? BLACK : WHITE);
          }
        }
      }
    }
  }
}

// ----------------------------------------------------
// RSSI → SIGNAL BARS
// ----------------------------------------------------
int rssiToBars(long rssi) {
  if (rssi >= -60)
    return 4; // Excellent
  if (rssi >= -70)
    return 3; // Good
  if (rssi >= -80)
    return 2; // Fair
  if (rssi >= -90)
    return 1; // Weak
  return 0;   // No signal
}

void drawWiFiIcon(int x, int y, int bars) {
  const int barWidth = 4;
  const int gap = 3;

  for (int i = 0; i < 4; i++) {
    int barXStart = x + i * (barWidth + gap);
    int barXEnd = barXStart + barWidth;
    int barHeight = 6 * (i + 1); // 6, 12, 18, 24 px

    int barYStart = y + (24 - barHeight);
    int barYEnd = y + 24;

    // 1) Always draw OUTLINE in BLACK (hollow bar)
    EPD_DrawRectangle(barXStart, barYStart, barXEnd, barYEnd, BLACK,
                      0); // 0 = outline only

    // 2) If this bar is "active", fill INSIDE the outline
    if (i < bars) {
      // Slightly inset rectangle so we don't overwrite the outline
      EPD_DrawRectangle(barXStart + 1, barYStart + 1, barXEnd - 1, barYEnd - 1,
                        BLACK, 1); // 1 = filled
    }
  }
}