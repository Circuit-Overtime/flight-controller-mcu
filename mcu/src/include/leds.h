#pragma once
#include <Arduino.h>

// Status LEDs with non-blocking patterns. Each LED has one of four modes;
// ledsUpdate() must be called periodically (every loop iteration is fine)
// to advance the patterns.
//
// LED slots:
//   STARTUP  (pin 24) — system-level state
//   CALIB    (pin 26) — calibration / sensor state
//   TEMP     (pin 28) — MPU6050 chip temperature warning
//   BATTERY  (pin 30) — battery voltage warning

enum LedMode : uint8_t {
  LED_MODE_OFF = 0,
  LED_MODE_ON,
  LED_MODE_BLINK,    // ~2 Hz steady blink (250 ms on, 250 ms off)
  LED_MODE_FLASH_2,  // two quick flashes per 1 s — fault indicator
};

void ledsInit();

// Advance all four LED patterns to the state appropriate for `now_ms`.
// Cheap to call every loop iteration.
void ledsUpdate(uint32_t now_ms);

void ledStartupSet(LedMode mode);
void ledCalibSet  (LedMode mode);
void ledTempSet   (LedMode mode);
void ledBatterySet(LedMode mode);
