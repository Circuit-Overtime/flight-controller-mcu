#pragma once
#include <Arduino.h>
#include "config.h"

// FlySky FS-R6B PWM capture. Pin assignments and timing constants live in
// config.h so the same API works whether the implementation is PCINT-based,
// external-interrupt based, or hardware-Input-Capture based.

void     rxInit();
uint16_t rxGet(uint8_t ch);                     // pulse width in µs; 0 if never seen
bool     rxAlive(uint8_t ch, uint32_t now_us);  // false if no recent valid pulse
