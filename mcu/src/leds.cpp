#include "include/leds.h"
#include "include/config.h"

struct LedSlot {
  uint8_t pin;
  LedMode mode;
};

static LedSlot leds[4] = {
  { LED_STARTUP_PIN, LED_MODE_OFF },
  { LED_CALIB_PIN,   LED_MODE_OFF },
  { LED_TEMP_PIN,    LED_MODE_OFF },
  { LED_BATTERY_PIN, LED_MODE_OFF },
};

// Pattern decoder: returns true if the LED should be on right now for the
// given mode, given the current millisecond clock.
static bool _resolve(LedMode mode, uint32_t now_ms) {
  switch (mode) {
    case LED_MODE_OFF:   return false;
    case LED_MODE_ON:    return true;
    case LED_MODE_BLINK: {
      // 250 ms on, 250 ms off => 2 Hz
      return (((now_ms / 250UL) & 1UL) == 0UL);
    }
    case LED_MODE_FLASH_2: {
      // Two 100 ms flashes inside a 1000 ms window — clearly distinct from
      // the steady-blink pattern.
      uint32_t t = now_ms % 1000UL;
      return (t < 100UL) || (t >= 200UL && t < 300UL);
    }
  }
  return false;
}

void ledsInit() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(leds[i].pin, OUTPUT);
    digitalWrite(leds[i].pin, LOW);
  }
}

void ledsUpdate(uint32_t now_ms) {
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(leds[i].pin, _resolve(leds[i].mode, now_ms) ? HIGH : LOW);
  }
}

void ledStartupSet(LedMode m) { leds[0].mode = m; }
void ledCalibSet  (LedMode m) { leds[1].mode = m; }
void ledTempSet   (LedMode m) { leds[2].mode = m; }
void ledBatterySet(LedMode m) { leds[3].mode = m; }
