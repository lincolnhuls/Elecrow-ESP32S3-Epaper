// Flash settings for this board are fairly specific
// 
// Board: ESP32S3 Dev Module 
// Flash Size: 8MB (64Mb)
// Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
// PSRAM: "OPI PSRAM"
// Upload Speed: "460800" (Board will not upload if this is not set) 


// Import needed libraries
#include <PubSubClient.h>
#include <FastLED.h>
#include <QRCodeGenerator.h>
#include <WiFiManager.h>
#include <Arduino.h>
#include "EPD.h"
#include "EPD_GUI.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <Preferences.h>

// Declare the WiFiManager and Preferences(non-volitile storage) to use in code
WiFiManager wifiManager;
Preferences prefs;

#define MAX_SLOTS 8
#define MAX_RANGES 16

// Flag for register state
bool registered = false;
bool registeringScreenShown = false;
bool connectedScreenShown = false;

// Led helpers
bool ledsActive = false;
uint32_t ledsOffAtMs = 0;
uint8_t lastBrightness = 255;
volatile bool forceLedRender = false;

// Blink Aids
bool blinkActive = false;
bool blinkIsOn = false;
uint32_t blinkNextToggleMs = 0;

uint16_t blinkIntervalMs = 0;
uint16_t blinkRemaining = 0;

uint8_t blinkR, blinkG, blinkB;
uint8_t blinkBrightness = 255;
uint16_t blinkStart, blinkCount;

// Breate Aids
bool breatheActive = false;
uint32_t breatheStartMs = 0;
uint32_t breatheHalfPeriodMs = 1000;

uint8_t breatheR, breatheG, breatheB;
uint8_t breatheMaxBrightness = 255;
uint16_t breatheStart, breatheCount;

uint32_t breatheNextFrameMs = 0;
uint32_t breatheLastFrameMs = 0;
const uint16_t BREATHE_FRAME_MS = 15; 

uint16_t breathePhase16 = 0;
uint16_t breathePhaseStep = 0;
uint8_t lastBreatheBrightness = 255;

// Timers to help with state transitions
unsigned long lastMqttAttemptMs = 0;
unsigned long lastRegisterAttemptMs = 0;

// Timers for heartbeat
unsigned long lastHeartbeatMs = 0;
const unsigned long HEARTBEAT_PERIOD_MS = 30000;

// Declare strings
String SERVICE_UUID = "";
String ReportTopic;
String SubscribeTopic;
String HeartbeatTopic;
String apName;
String wifiID;

// Mqtt topics
const char *reportTopic ;
const char *subscribeTopic ;
const char *heartbeatTopic ;

// Setup WiFi Client
WiFiClient      Wifi_net;

// Led setup
#define NUM_LEDS 100
#define DATA_PIN 8
CRGB leds[NUM_LEDS];

// 240x416 buffer
uint8_t ImageBW[12480];
char buffer[200];

// Forward declarations
int rssiToBars(long rssi);
void displayQRCodeOnEPD(const char* text);
void drawWiFiIcon(int x, int y, int bars);

// Declare space for chipid and shortId
uint64_t chipid;
char chipIdStr[32];
char shortId[7];

// callback helpers
volatile bool mqttMsgReady = false;
char mqttTopicBuff[256];
char mqttPayloadBuff[1024];

// Declare space for secret from preferences
String secret;
String deviceName;

///////////////// MQTT Broker Setup //////////////////////////
const char* mqttServer = "mqttbroker.tetontechnology.com";        
const char* brokerName = "fuelbroker";
const char* brokerPassword = "N3tJPFTHYYNcsHw";
/////////////////////////////////////////////////////////////


////////////////////////////////////// Led Functions //////////////////////////////////////

// Slot creation so multiple areas on the cart can be on and updated at the same time
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

uint32_t ledsLastFrameMs = 0;
const uint16_t LED_FRAME_MS = 15;

// Range helpers for the slots
static inline uint32_t rangeEnd(const LedRange& r) {
  return (uint32_t)r.start + (uint32_t)r.count; // exclusive end
}

static bool rangesOverlap(const LedRange& a, const LedRange& b) {
  uint32_t a0 = a.start, a1 = rangeEnd(a);
  uint32_t b0 = b.start, b1 = rangeEnd(b);
  return (a0 < b1) && (b0 < a1);
}

static bool slotOverlapsList(const LedSlot& s, const LedRange* list, uint8_t listCount) {
  uint8_t sN = s.rangeCount;
  if (sN > MAX_RANGES) sN = MAX_RANGES;
  if (listCount > MAX_RANGES) listCount = MAX_RANGES;

  for (uint8_t i = 0; i < sN; i++) {
    for (uint8_t j = 0; j < listCount; j++) {
      if (rangesOverlap(s.ranges[i], list[j])) return true;
    }
  }
  return false;
}

// ---------------------- Slot lifecycle ----------------------

static void clearSlot(uint8_t i) {
  if (i >= MAX_SLOTS) return;
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

// Find an available slot; if none, reuse slot 0 (simple + cheap)
static int8_t allocSlot() {
  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active) return (int8_t)i;
  }
  return 0;
}

// "Last command wins" for overlaps:
// clear any existing slot whose ranges overlap the incoming list.
static void cancelOverlappingSlots(const LedRange* list, uint8_t listCount) {
  if (!list || listCount == 0) return;
  if (listCount > MAX_RANGES) listCount = MAX_RANGES;

  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active) continue;

    uint8_t sN = slots[i].rangeCount;
    if (sN > MAX_RANGES) sN = MAX_RANGES;

    if (rangesOverlap(slots[i].ranges, sN, list, listCount)) {
      clearSlot(i);
    }
  }
}

// ---------------------- LED write helpers ----------------------

static inline uint8_t scale8u(uint8_t v, uint8_t scale) {
  // FastLED has scale8(), but keeping local avoids extra includes/assumptions.
  return (uint8_t)(((uint16_t)v * (uint16_t)scale) / 255u);
}

// Writes a solid color into leds[] for a single range (no FastLED.show here)
static void fillRangeRGB(uint16_t start, uint16_t count, uint8_t r, uint8_t g, uint8_t b) {
  uint32_t end = (uint32_t)start + (uint32_t)count;
  if (start >= NUM_LEDS) return;
  if (end > NUM_LEDS) end = NUM_LEDS;

  for (uint32_t i = start; i < end; i++) {
    leds[i].setRGB(r, g, b);
  }
}

// helper: copy ranges into slot (clamps to MAX_RANGES)
static void copyRangesToSlot(LedSlot& s, const LedRange* ranges, uint8_t n) {
  if (n > MAX_RANGES) n = MAX_RANGES;
  s.rangeCount = n;
  for (uint8_t i = 0; i < n; i++) s.ranges[i] = ranges[i];
}

void on(uint8_t red, uint8_t green, uint8_t blue,
        int16_t duration_s,
        uint8_t brightness,
        const LedRange* ranges,
        uint8_t n) {
  if (!ranges || n == 0) return;

  cancelOverlappingSlots(ranges, n);
  int8_t idx = allocateSlotFor(ranges, n);
  clearSlot(idx);

  LedSlot& s = slots[idx];
  s.active = true;
  s.type = EFFECT_ON;

  s.r = red; s.g = green; s.b = blue;
  s.maxBrightness = brightness;

  copyRangesToSlot(s, ranges, n);

  s.offAtMs = (duration_s < 0) ? 0 : (millis() + (uint32_t)duration_s * 1000UL);

  // initialize timing fields so ledTick updates cleanly
  s.lastFrameMs = millis();
  s.blinkNextToggleMs = 0;
}

void blink(uint8_t red, uint8_t green, uint8_t blue,
           uint16_t interval_ms,
           uint8_t times,
           uint8_t brightness,
           const LedRange* ranges,
           uint8_t n) {
  if (!ranges || n == 0) return;

  cancelOverlappingSlots(ranges, n);
  int8_t idx = allocateSlotFor(ranges, n);
  clearSlot(idx);

  LedSlot& s = slots[idx];
  s.active = true;
  s.type = EFFECT_BLINK;

  s.r = red; s.g = green; s.b = blue;
  s.maxBrightness = brightness;

  copyRangesToSlot(s, ranges, n);

  s.blinkIntervalMs = interval_ms;
  s.blinkIsOn = false;
  s.blinkNextToggleMs = millis();                 // toggle immediately on next tick
  s.blinkRemainingToggles = (times == 0) ? 0 : (uint16_t)times * 2;

  s.offAtMs = 0;
  s.lastFrameMs = millis();
}

void breathe(uint8_t red, uint8_t green, uint8_t blue,
             int16_t duration_s,   // HALF cycle seconds (same meaning as before)
             uint8_t brightness,
             const LedRange* ranges,
             uint8_t n) {
  if (!ranges || n == 0) return;

  cancelOverlappingSlots(ranges, n);
  int8_t idx = allocateSlotFor(ranges, n);
  clearSlot(idx);

  uint32_t halfMs  = (uint32_t)max<int16_t>(1, duration_s) * 1000UL;
  uint32_t cycleMs = halfMs * 2UL;
  if (cycleMs == 0) cycleMs = 1;

  LedSlot& s = slots[idx];
  s.active = true;
  s.type = EFFECT_BREATHE;

  s.r = red; s.g = green; s.b = blue;
  s.maxBrightness = brightness;

  copyRangesToSlot(s, ranges, n);

  s.phaseStep = (uint16_t)((65536UL * BREATHE_FRAME_MS) / cycleMs);
  if (s.phaseStep == 0) s.phaseStep = 1;

  s.phase16 = (uint16_t)(64 << 8);                // start bright
  s.lastFrameMs = millis();

  s.offAtMs = 0;
}

void stopLedEffects() {
  for (uint8_t i = 0; i < MAX_SLOTS; i++) clearSlot(i);
}


// ---------------------- Slot state update ----------------------

static bool updateSlotState(LedSlot& s, uint32_t now) {
  // returns true if anything about the slot changed (so we should re-render)
  if (!s.active) return false;

  // 1) timed ON: expire
  if (s.type == EFFECT_ON && s.offAtMs != 0 && (int32_t)(now - s.offAtMs) >= 0) {
    s.active = false;
    return true;
  }

  // 2) blink: toggle on schedule
  if (s.type == EFFECT_BLINK) {
    if ((int32_t)(now - s.blinkNextToggleMs) >= 0) {
      s.blinkNextToggleMs = now + s.blinkIntervalMs;
      s.blinkIsOn = !s.blinkIsOn;

      if (s.blinkRemainingToggles > 0) {
        s.blinkRemainingToggles--;
        if (s.blinkRemainingToggles == 0) {
          s.active = false; // done blinking
        }
      }
      return true;
    }
    return false;
  }

  // 3) breathe: advance phase on frame cadence
  if (s.type == EFFECT_BREATHE) {
    uint32_t elapsed = now - s.lastFrameMs;
    if (elapsed < BREATHE_FRAME_MS) return false;

    uint32_t frames = elapsed / BREATHE_FRAME_MS;
    if (frames > 3) frames = 3;          // cap catch-up
    s.lastFrameMs += frames * BREATHE_FRAME_MS;
    s.phase16 += (uint16_t)(frames * s.phaseStep);
    return true;
  }

  return false;
}

// Convert slot -> brightness scale 0..255
static uint8_t slotLevel(const LedSlot& s) {
  if (!s.active) return 0;

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
    if (!slots[i].active) continue;
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
  if (!needRender) return;

  // Rebuild the LED buffer from all active (non-mask) slots
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }

  // 1) Render normal effects first (ON/BLINK/BREATHE)
  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    const LedSlot& s = slots[i];
    if (!s.active) continue;
    if (s.type == EFFECT_MASK_OFF) continue; // <-- skip masks for now

    uint8_t level = slotLevel(s);
    if (level == 0) continue;

    uint8_t rr = scale8u(s.r, level);
    uint8_t gg = scale8u(s.g, level);
    uint8_t bb = scale8u(s.b, level);

    uint8_t n = s.rangeCount;
    if (n > MAX_RANGES) n = MAX_RANGES;

    for (uint8_t k = 0; k < n; k++) {
      fillRangeRGB(s.ranges[k].start, s.ranges[k].count, rr, gg, bb);
    }
  }

  // 2) Apply OFF masks last (force black in those ranges)
  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    const LedSlot& s = slots[i];
    if (!s.active) continue;
    if (s.type != EFFECT_MASK_OFF) continue;

    uint8_t n = s.rangeCount;
    if (n > MAX_RANGES) n = MAX_RANGES;

    for (uint8_t k = 0; k < n; k++) {
      fillRangeRGB(s.ranges[k].start, s.ranges[k].count, 0, 0, 0);
    }
  }

  FastLED.show();
  forceLedRender = false;
}
///////////////////////////////////////////////////////////////////////////////////////////

// -------- ledList parsing + overlap helpers --------

static uint8_t parseLedList(JsonArray list, LedRange* out, uint8_t outMax) {
  uint8_t n = 0;
  for (JsonVariant v : list) {
    JsonArray pair = v.as<JsonArray>();
    if (pair.isNull() || pair.size() < 2) continue;

    uint16_t start = pair[0] | 0;
    uint16_t count = pair[1] | 0;
    if (count == 0) continue;
    if (start >= NUM_LEDS) continue;

    // clamp to strip
    uint16_t maxCount = (NUM_LEDS - start);
    if (count > maxCount) count = maxCount;

    out[n++] = { start, count };
    if (n >= outMax) break;
  }
  return n;
}

static bool rangesOverlap(const LedRange* a, uint8_t aN, const LedRange* b, uint8_t bN) {
  if (aN > MAX_RANGES) aN = MAX_RANGES;
  if (bN > MAX_RANGES) bN = MAX_RANGES;

  for (uint8_t i = 0; i < aN; i++) {
    uint16_t a0 = a[i].start;
    uint16_t a1 = a0 + a[i].count; // exclusive
    for (uint8_t j = 0; j < bN; j++) {
      uint16_t b0 = b[j].start;
      uint16_t b1 = b0 + b[j].count; // exclusive
      if (a0 < b1 && b0 < a1) return true;
    }
  }
  return false;
}

// Pick a slot. Prefer an inactive one; otherwise steal the first overlapping one; otherwise steal slot 0.
static int8_t allocateSlotFor(const LedRange* newRanges, uint8_t newN) {
  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active) return (int8_t)i;
  }
  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    if (rangesOverlap(slots[i].ranges, slots[i].rangeCount, newRanges, newN)) return (int8_t)i;
  }
  return 0;
}


// Wifi callback for digesting commands over Mqtt
void WiFi_callback(char* topic, byte* payload, unsigned int length) {
  // Copy topic
  strncpy(mqttTopicBuff, topic, sizeof(mqttTopicBuff));
  mqttTopicBuff[sizeof(mqttTopicBuff) - 1] = '\0';

  // Copy payload
  size_t n = (length < sizeof(mqttPayloadBuff) - 1) ? length : (sizeof(mqttPayloadBuff) - 1);
  memcpy(mqttPayloadBuff, payload, n);
  mqttPayloadBuff[n] = '\0';

  mqttMsgReady = true;
}

void handleMqttMessage() {
  if (!mqttMsgReady) return;
  mqttMsgReady = false;

  if (strcmp(mqttTopicBuff, subscribeTopic) != 0) return;

  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, mqttPayloadBuff);
  if (err) {
    Serial.print("MQTT JSON parse failed: ");
    Serial.println(err.c_str());
    return;
  }

  // Debug: print received JSON
  String output;
  serializeJson(doc, output);
  Serial.println(output);

  // ------------------------------------------------------------
  // NOT REGISTERED: expect adoption_complete event (not "cmd")
  // ------------------------------------------------------------
  if (!registered) {
    const char* event = doc["event"] | "";

    // Only accept adoption_complete while unregistered
    if (strcmp(event, "adoption_complete") != 0) {
      Serial.print("Ignoring message while unregistered; event=");
      Serial.println(event);
      return;
    }

    const char* s  = doc["secret"] | "";
    const char* dN = doc["deviceName"] | "";

    if (s[0] == '\0') {
      Serial.println("adoption_complete missing secret");
      return;
    }
    if (dN[0] == '\0') {
      Serial.println("adoption_complete missing deviceName");
      return;
    }

    prefs.begin("reg", false);
    prefs.putString("secret", s);
    prefs.putString("deviceName", dN);
    prefs.end();

    secret = s;
    deviceName = dN;
    registered = true;
    connectedScreenShown = false;
    registeringScreenShown = false;

    Serial.println("Adoption complete: secret + deviceName saved");
    return;
  }

  // ------------------------------------------------------------
  // REGISTERED: accept normal cmds (remove/on/blink/breathe/off)
  // ------------------------------------------------------------
  const char* incomingSecret = doc["secret"] | "";
  if (incomingSecret[0] == '\0' || secret.length() == 0) {
    Serial.println("Rejected command: Missing secret");
    return;
  }

  if (secret != incomingSecret) {
    Serial.println("Rejected command: Bad secret");
    return;
  }

  const char* cmd = doc["cmd"] | "";
  if (cmd[0] == '\0') {
    Serial.println("No cmd provided");
    return;
  }

  if (strcmp(cmd, "remove") == 0) {
    remove();
    forceLedRender = true;
    return;
  }

  if (strcmp(cmd, "on") == 0) {
    uint8_t r = doc["r"] | 255;
    uint8_t g = doc["g"] | 255;
    uint8_t b = doc["b"] | 255;

    uint8_t brightness = doc["brightness"] | 255;
    int16_t duration_s = doc["duration"] | -1;

    if (!doc.containsKey("ledList")) {
      Serial.println("on rejected: missing ledList");
      return;
    }

    LedRange tmp[MAX_RANGES];
    JsonArray list = doc["ledList"].as<JsonArray>();
    uint8_t n = parseLedList(list, tmp, MAX_RANGES);
    if (n == 0) {
      Serial.println("on rejected: empty ledList");
      return;
    }

    cancelOverlappingSlots(tmp, n);
    int8_t idx = allocateSlotFor(tmp, n);
    clearSlot((uint8_t)idx);

    LedSlot& s = slots[idx];
    s.active = true;
    s.type = EFFECT_ON;
    s.r = r; s.g = g; s.b = b;
    s.maxBrightness = brightness;

    s.rangeCount = n;
    for (uint8_t i = 0; i < n; i++) s.ranges[i] = tmp[i];

    if (duration_s < 0) s.offAtMs = 0;
    else s.offAtMs = millis() + (uint32_t)duration_s * 1000UL;

    // force render next tick
    s.blinkNextToggleMs = 0;
    s.lastFrameMs = 0;

    forceLedRender = true;
    return;
    }

  if (strcmp(cmd, "blink") == 0) {
    uint8_t r = doc["r"] | 255;
    uint8_t g = doc["g"] | 255;
    uint8_t b = doc["b"] | 255;

    uint16_t interval_ms = doc["interval"] | 500;
    uint8_t times = doc["times"] | 0; // 0 = infinite
    uint8_t brightness = doc["brightness"] | 255;

    if (!doc.containsKey("ledList")) {
      Serial.println("blink rejected: missing ledList");
      return;
    }

    LedRange tmp[MAX_RANGES];
    JsonArray list = doc["ledList"].as<JsonArray>();
    uint8_t n = parseLedList(list, tmp, MAX_RANGES);
    if (n == 0) {
      Serial.println("blink rejected: empty ledList");
      return;
    }

    cancelOverlappingSlots(tmp, n);
    int8_t idx = allocateSlotFor(tmp, n);
    clearSlot((uint8_t)idx);

    LedSlot& s = slots[idx];
    s.active = true;
    s.type = EFFECT_BLINK;
    s.r = r; s.g = g; s.b = b;
    s.maxBrightness = brightness;

    s.rangeCount = n;
    for (uint8_t i = 0; i < n; i++) s.ranges[i] = tmp[i];

    s.blinkIntervalMs = interval_ms;
    s.blinkIsOn = false;
    s.blinkNextToggleMs = millis(); // toggle immediately

    s.blinkRemainingToggles = (times == 0) ? 0 : (uint16_t)times * 2;

    forceLedRender = true;
    return;
  }


  if (strcmp(cmd, "breathe") == 0) {
    uint8_t r = doc["r"] | 255;
    uint8_t g = doc["g"] | 255;
    uint8_t b = doc["b"] | 255;

    int16_t duration_s = doc["duration"] | 1;   // HALF cycle seconds
    uint8_t brightness = doc["brightness"] | 255;

    if (!doc.containsKey("ledList")) {
      Serial.println("breathe rejected: missing ledList");
      return;
    }

    LedRange tmp[MAX_RANGES];
    JsonArray list = doc["ledList"].as<JsonArray>();
    uint8_t n = parseLedList(list, tmp, MAX_RANGES);
    if (n == 0) {
      Serial.println("breathe rejected: empty ledList");
      return;
    }

    cancelOverlappingSlots(tmp, n);
    int8_t idx = allocateSlotFor(tmp, n);
    clearSlot((uint8_t)idx);

    uint32_t halfMs = (uint32_t)max<int16_t>(1, duration_s) * 1000UL;
    uint32_t cycleMs = halfMs * 2UL;
    if (cycleMs == 0) cycleMs = 1;

    LedSlot& s = slots[idx];
    s.active = true;
    s.type = EFFECT_BREATHE;
    s.r = r; s.g = g; s.b = b;
    s.maxBrightness = brightness;

    s.rangeCount = n;
    for (uint8_t i = 0; i < n; i++) s.ranges[i] = tmp[i];

    s.phaseStep = (uint16_t)((65536UL * BREATHE_FRAME_MS) / cycleMs);
    if (s.phaseStep == 0) s.phaseStep = 1;

    s.phase16 = (uint16_t)(64 << 8); // start bright
    s.lastFrameMs = millis();

    forceLedRender = true;
    return;
  }


  if (strcmp(cmd, "off") == 0) {
    bool all = doc["all"] | false;

    if (all || !doc.containsKey("ledList")) {
      stopLedEffects();
      forceLedRender = true;
      return;
    }

    LedRange tmp[MAX_RANGES];
    JsonArray list = doc["ledList"].as<JsonArray>();
    uint8_t n = parseLedList(list, tmp, MAX_RANGES);
    if (n == 0) {
      Serial.println("off rejected: empty ledList");
      return;
    }

    int8_t idx = allocSlot();
    clearSlot((uint8_t)idx);

    LedSlot& s = slots[idx];
    s.active = true;
    s.type = EFFECT_MASK_OFF;
    copyRangesToSlot(s, tmp, n);

    forceLedRender = true;
    return;
  }



    Serial.print("Unknown cmd: ");
    Serial.println(cmd);
  }

// Set up Mqtt Client 
PubSubClient    mqttClient(mqttServer, 1883, WiFi_callback, Wifi_net);

void mqttConnectAttempt() {
  if (mqttClient.connected()) return;

  if (millis() - lastMqttAttemptMs < 5000) return;
  lastMqttAttemptMs = millis();

  Serial.println("Attempting MQTT connect...");
  if (mqttClient.connect(wifiID.c_str(), brokerName, brokerPassword)) {
    mqttClient.subscribe(subscribeTopic);
    mqttClient.subscribe(heartbeatTopic);
    Serial.println("MQTT connected + subscribed");
  } else {
    Serial.println("MQTT connected failed");
  }
}
void remove() {
  Serial.println("Remove command recieved");

  prefs.begin("reg", false);
  prefs.remove("secret");
  prefs.remove("deviceName");
  prefs.end();

  secret = "";
  deviceName = "";
  registered = false;
  connectedScreenShown = false;
  registeringScreenShown = false;

  mqttClient.disconnect();
  lastMqttAttemptMs = 0;
  lastRegisterAttemptMs = 0;

  showRegisteringScreen();
}


void registerAttempt() {
  if (registered) return;

  if (!mqttClient.connected()) return;

  if (millis() - lastRegisterAttemptMs < 3000) return;
  lastRegisterAttemptMs = millis();

  bool ok = requestRegister();
  Serial.print("Register Publish: ");
  Serial.println(ok ? "ok" : "failed");
}

// Function to call once a device has wifi and mqtt connection for registering it with the organization
bool requestRegister() {
  if (!mqttClient.connected()) return false;
  
  StaticJsonDocument<128> doc;
  doc["chipId"] = SERVICE_UUID;
  doc["code"] = shortId;

  char out[128];
  size_t n = serializeJson(doc, out, sizeof(out));

  return mqttClient.publish(reportTopic, out, n);
}

void heartbeat() {
  if (!registered) return;

  if (!mqttClient.connected()) return;

  if (millis() - lastHeartbeatMs < HEARTBEAT_PERIOD_MS) return;
  lastHeartbeatMs = millis();

  StaticJsonDocument<256> doc;
  doc["secret"] = secret;
  doc["code"] = shortId;
  doc["uuid"] = SERVICE_UUID;
  doc["ssid"] = WiFi.SSID().c_str();
  doc["ip"] = WiFi.localIP().toString();
  doc["rssi"] = WiFi.RSSI();

  char out[256];
  size_t n = serializeJson(doc, out, sizeof(out));

  bool ok = mqttClient.publish(heartbeatTopic, out, n);
  Serial.print("Heartbeat publish: ");
  Serial.println(ok ? "ok" : "failed");
}

// WiFi setup screen
void showWifiSetupScreen(const char* apName) {
  EPD_FastInit();
  EPD_Display_Clear();

  Paint_NewImage(ImageBW, EPD_W, EPD_H, 0, WHITE);
  Paint_Clear(WHITE);

  // Left side
  EPD_ShowString(10, 10,  "WIFI SETUP REQUIRED", 16, BLACK);
  EPD_ShowString(10, 50,  "Connect to AP:", 16, BLACK);
  EPD_ShowString(10, 70,  apName, 16, BLACK);
  EPD_ShowString(10, 90,  "Navitage to: ", 16, BLACK);
  EPD_ShowString(10, 110,  "URL: 192.168.4.1", 16, BLACK);
  EPD_ShowString(10, 150,  "Or scan QR to open", 16, BLACK);
  EPD_ShowString(10, 170,  "setup portal", 16, BLACK);
 
  // Right side QR to portal
  snprintf(buffer, sizeof(buffer), "WIFI:T:nopass;S:%s;;", apName);
  displayQRCodeOnEPD(buffer);

  EPD_Display(ImageBW);
  EPD_Update();
}

// Stage two screen while connecting to mqtt and getting registered
void showRegisteringScreen() {
  EPD_FastInit();
  EPD_Display_Clear();

  Paint_NewImage(ImageBW, EPD_W, EPD_H, 0, WHITE);
  Paint_Clear(WHITE);

  EPD_ShowString(10, 10,  "WIFI CONNECTED", 16, BLACK);
  EPD_ShowString(10, 30,  "Registering device...", 16, BLACK);

  snprintf(buffer, sizeof(buffer), "Code: %s", shortId);
  EPD_ShowString(10, 55, buffer, 24, BLACK);

  EPD_ShowString(10, 80,  "Connecting MQTT...", 16, BLACK);
  EPD_ShowString(10, 100, "Waiting for secret", 16, BLACK);

  displayQRCodeOnEPD(shortId);

  EPD_Display(ImageBW);
  EPD_Update();
}


// ----------------------------------------------------
// SETUP
// ----------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  chipid = ESP.getEfuseMac();

  // Create small code for wh connection
  snprintf(shortId, sizeof(shortId), "%06X", (uint32_t)(chipid & 0xFFFFFF));

  // Build service uuid using chip id
  SERVICE_UUID = "mqtt" + (String)chipid;

  // Make readable chip id string
  sprintf(chipIdStr, "%04X%08X",
          (uint16_t)(chipid >> 32),
          (uint32_t)chipid);
 
  apName = "Cart Setup ";
  apName += shortId;

  // Reset settings for testing
  // wifiManager.resetSettings();

  FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);

  // ----- EPD INIT -----
  EPD_GPIOInit();
  EPD_Init();

  // Get wifi id
  wifiID = shortId;

  Paint_NewImage(ImageBW, EPD_W, EPD_H, 0, WHITE);
  Paint_Clear(WHITE);
  
  // True means that namespace can be read but not written to, false is read and write
  // Ready from prefs to find if a secret key is stored
  prefs.begin("reg", true);
  secret = prefs.getString("secret", "");
  deviceName = prefs.getString("deviceName", "");
  prefs.end();

  // Verify if we have a secret (if we do then we are registered, not if we don't)
  registered = (secret.length() > 0);

  // Setup the mqttClient
  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(WiFi_callback);

  /////////////////// Setup MQTT topics ///////////////////////
  ReportTopic = "wh/register/" + SERVICE_UUID;
  SubscribeTopic = "wh/device/" + SERVICE_UUID;
  HeartbeatTopic = SubscribeTopic + "/heartbeat";

  reportTopic = ReportTopic.c_str();
  subscribeTopic = SubscribeTopic.c_str();
  heartbeatTopic = HeartbeatTopic.c_str();
  ////////////////////////////////////////////////////////////

  // BOOT SCREEN
  EPD_ShowString(10, 10, "Connecting to WiFi...", 16, BLACK);
  EPD_Display(ImageBW);
  EPD_Update();

  delay(1500); // allow panel to settle

  // ----- WIFI INIT (NON-BLOCKING) -----
  WiFi.mode(WIFI_STA);
  WiFi.begin();

  // 10 seconds for trying to connected to a saved network
  wifiManager.setConnectTimeout(10);
  wifiManager.setEnableConfigPortal(false);
  bool ok = wifiManager.autoConnect(apName.c_str());

  if (!ok) {
    // Show setup screen before starting the portal to ensure display
    showWifiSetupScreen(apName.c_str());

    wifiManager.setEnableConfigPortal(true);
    wifiManager.setConfigPortalTimeout(180);

    while (!wifiManager.autoConnect(apName.c_str())) {
      // Loop until connected
    }
  }
}

// ----------------------------------------------------
// LOOP 
// ----------------------------------------------------
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectedScreenShown = false;
    registeringScreenShown = false;

    wifiManager.setConfigPortalTimeout(5);
    bool ok = wifiManager.autoConnect(apName.c_str());
    if (ok) return;

    showWifiSetupScreen(apName.c_str());

    wifiManager.setConfigPortalTimeout(180);
    while (!wifiManager.autoConnect(apName.c_str())) {}
    return;
  }
  
  mqttClient.loop();
  handleMqttMessage();

  mqttConnectAttempt();
  heartbeat();

  ledTick();

  if (!registered) {
    if (!registeringScreenShown) {
      registeringScreenShown = true;
      connectedScreenShown = false;
      showRegisteringScreen();
    }

    registerAttempt();
    return;
  }

  if (!connectedScreenShown) {
    connectedScreenShown = true;
    registeringScreenShown = false;
    showConnectedDashboard();
  }
}

// ----------------------------------------------------
// CONNECTED DASHBOARD
// ----------------------------------------------------
void showConnectedDashboard() {
  EPD_FastInit();
  EPD_Display_Clear();

  Paint_NewImage(ImageBW, EPD_W, EPD_H, 0, WHITE);
  Paint_Clear(WHITE);

  // Title
  EPD_ShowString(10, 10, "CONNECTED", 16, BLACK);

  // SSID
  snprintf(buffer, sizeof(buffer), "SSID: %s", WiFi.SSID().c_str());
  EPD_ShowString(10, 30, buffer, 16, BLACK);

  // IP Address
  snprintf(buffer, sizeof(buffer), "IP: %s", WiFi.localIP().toString().c_str());
  EPD_ShowString(10, 50, buffer, 16, BLACK);

  // RSSI Value
  long rssi = WiFi.RSSI();
  snprintf(buffer, sizeof(buffer), "Signal: %ld dBm", rssi);
  EPD_ShowString(10, 70, buffer, 16, BLACK);

  // Signal Bars
  int bars = rssiToBars(rssi);
  drawWiFiIcon(10, 95, bars);   // under RSSI text

  snprintf(buffer, sizeof(buffer), "Device name: %s", deviceName.c_str());
  EPD_ShowString(10, 135, buffer, 16, BLACK);


  // QR Code (right side)
  displayQRCodeOnEPD(SERVICE_UUID.c_str());

  EPD_Display(ImageBW);
  EPD_Update();
  EPD_DeepSleep();
}

// ----------------------------------------------------
// QR CODE
// ----------------------------------------------------
void displayQRCodeOnEPD(const char* text) {
  QRCode qrcode;
  uint8_t qrcodeData[qrcode_getBufferSize(4)];
  qrcode_initText(&qrcode, qrcodeData, 4, 0, text);

  const int scale = 5;
  int qrPixSize = qrcode.size * scale;
  int displayWidth = Paint.width;

  const int pxOffset = displayWidth - qrPixSize - 12;
  const int pyOffset = 40;

  for (uint8_t y = 0; y < qrcode.size; y++) {
    for (uint8_t x = 0; x < qrcode.size; x++) {
      bool module = qrcode_getModule(&qrcode, x, y);
      for (int dx = 0; dx < scale; dx++) {
        for (int dy = 0; dy < scale; dy++) {
          Paint_SetPixel(pxOffset + x * scale + dx,
                         pyOffset + y * scale + dy,
                         module ? BLACK : WHITE);
        }
      }
    }
  }
}


// ----------------------------------------------------
// RSSI → SIGNAL BARS
// ----------------------------------------------------
int rssiToBars(long rssi) {
  if (rssi >= -60) return 4;   // Excellent
  if (rssi >= -70) return 3;   // Good
  if (rssi >= -80) return 2;   // Fair
  if (rssi >= -90) return 1;   // Weak
  return 0;                    // No signal
}

void drawWiFiIcon(int x, int y, int bars) {
  const int barWidth = 4;
  const int gap = 3;

  for (int i = 0; i < 4; i++) {
    int barXStart = x + i * (barWidth + gap);
    int barXEnd   = barXStart + barWidth;
    int barHeight = 6 * (i + 1);      // 6, 12, 18, 24 px

    int barYStart = y + (24 - barHeight);
    int barYEnd   = y + 24;

    // 1) Always draw OUTLINE in BLACK (hollow bar)
    EPD_DrawRectangle(barXStart, barYStart,
                      barXEnd,   barYEnd,
                      BLACK, 0);   // 0 = outline only

    // 2) If this bar is "active", fill INSIDE the outline
    if (i < bars) {
      // Slightly inset rectangle so we don't overwrite the outline
      EPD_DrawRectangle(barXStart + 1, barYStart + 1,
                        barXEnd   - 1, barYEnd   - 1,
                        BLACK, 1);   // 1 = filled
    }
  }
}


