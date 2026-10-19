#include "LedManager.h"
#include <Arduino.h>
#include <FastLED.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>

// ======================= Public Methods =======================
void LedManager::init() {
  pinMode(LED, OUTPUT);
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);

  digitalWrite(LED, LOW);
  analogWrite(R, 255);
  analogWrite(G, 255);
  analogWrite(B, 255);

  FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);
  stopLedEffects();
  FastLED.show();
}

void LedManager::fadeLed(const char *color) {
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

void LedManager::blinkLed(const char *color, int on_off_time) {
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

void LedManager::stopLedEffects() {
  for (uint8_t i = 0; i < MAX_SLOTS; i++)
    clearSlot(i);
  forceLedRender = true;
}

void LedManager::ledTick() {
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

void LedManager::handleLightPayload(JsonObject doc) {
  const char *cmd = doc["cmd"] | "";
  if (cmd[0] == '\0') {
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

void LedManager::turnOff() {
  digitalWrite(LED, LOW);
}

// ======================= Private Methods =======================

void LedManager::clearSlot(uint8_t i) {
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

bool LedManager::subtractOneRange(const LedRange &src, const LedRange &cut,
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

bool LedManager::subtractCutsFromSlotByIndex(uint8_t slotIdx, const LedRange *cuts,
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

void LedManager::applyPerLedReplacement(const LedRange *list, uint8_t listCount) {
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

inline uint8_t LedManager::scale8u(uint8_t v, uint8_t scale) {
  // FastLED has scale8(), but keeping local avoids extra includes/assumptions.
  return (uint8_t)(((uint16_t)v * (uint16_t)scale) / 255u);
}

// Writes a solid color into leds[] for a single range (no FastLED.show here)
void LedManager::fillRangeRGB(uint16_t start, uint16_t count, uint8_t r, uint8_t g,
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
void LedManager::copyRangesToSlot(LedSlot &s, const LedRange *ranges, uint8_t n) {
  if (n > MAX_RANGES)
    n = MAX_RANGES;
  s.rangeCount = n;
  for (uint8_t i = 0; i < n; i++)
    s.ranges[i] = ranges[i];
}

// Pick a slot. Prefer an inactive one; otherwise steal the first overlapping
// one; otherwise steal slot 0.
bool LedManager::rangesOverlap(const LedRange *a, uint8_t aN, const LedRange *b,
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

int8_t LedManager::allocateSlotFor(const LedRange *newRanges, uint8_t newN) {
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

void LedManager::on(uint8_t red, uint8_t green, uint8_t blue, int16_t duration_s,
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

void LedManager::blink(uint8_t red, uint8_t green, uint8_t blue, uint16_t interval_ms,
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

void LedManager::breathe(uint8_t red, uint8_t green, uint8_t blue,
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

bool LedManager::updateSlotState(LedSlot &s, uint32_t now) {
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
uint8_t LedManager::slotLevel(const LedSlot &s) {
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

uint8_t LedManager::parseLedList(JsonArray list, LedRange *out, uint8_t outMax) {
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
