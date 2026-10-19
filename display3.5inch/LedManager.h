#ifndef LED_MANAGER_H
#define LED_MANAGER_H

#include <FastLED.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>

class LedManager {
// Extra private above methods to make structs usable
private:
  static constexpr uint8_t R = 4;
  static constexpr uint8_t G = 5;
  static constexpr uint8_t B = 6;
  static constexpr uint8_t LED = 14;
  static constexpr uint8_t MAX_SLOTS = 25;
  static constexpr uint8_t MAX_RANGES = 25;
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

    static constexpr uint16_t NUM_LEDS = 200;
    static constexpr uint8_t DATA_PIN = 8;
    CRGB leds[NUM_LEDS];

public:
  // Methods 
  void init();
  void fadeLed(const char *color);
  void blinkLed(const char *color, int on_off_time);
  void stopLedEffects();
  void ledTick();
  void handleLightPayload(JsonObject doc);
  void turnOff();

private:
  // Methods
  void clearSlot(uint8_t i);
  bool subtractOneRange(const LedRange &src, const LedRange &cut, LedRange *out, uint8_t &outN);
  bool subtractCutsFromSlotByIndex(uint8_t slotIdx, const LedRange *cuts, uint8_t cutN);
  void applyPerLedReplacement(const LedRange *list, uint8_t listCount);
  inline uint8_t scale8u(uint8_t v, uint8_t scale);
  void fillRangeRGB(uint16_t start, uint16_t count, uint8_t r, uint8_t g,
                         uint8_t b);
  void copyRangesToSlot(LedSlot &s, const LedRange *ranges, uint8_t n);
  bool rangesOverlap(const LedRange *a, uint8_t aN, const LedRange *b,
                          uint8_t bN);
  int8_t allocateSlotFor(const LedRange *newRanges, uint8_t newN);
  void on(uint8_t red, uint8_t green, uint8_t blue, int16_t duration_s,
        uint8_t brightness, const LedRange *ranges, uint8_t n);
  void blink(uint8_t red, uint8_t green, uint8_t blue, uint16_t interval_ms,
      uint16_t times, uint8_t brightness, const LedRange *ranges,
      uint8_t n);
  void breathe(uint8_t red, uint8_t green, uint8_t blue,
             int16_t duration_s, // HALF cycle seconds (same meaning as before)
             uint8_t brightness, const LedRange *ranges, uint8_t n);
  bool updateSlotState(LedSlot &s, uint32_t now);
  uint8_t slotLevel(const LedSlot &s);
  uint8_t parseLedList(JsonArray list, LedRange *out, uint8_t outMax);

  // Attributes

  // Variables for Connection Status Led
  bool ledState = false;
  unsigned long ledTimer = 0;
  int fadeValue = -1;
  bool fadingDown = false;
  volatile bool forceLedRender = false;
  const uint16_t BREATHE_FRAME_MS = 15;

};

#endif