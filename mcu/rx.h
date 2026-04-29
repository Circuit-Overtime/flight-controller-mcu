#pragma once
#include <Arduino.h>

// FlySky FS-R6B 6-channel PWM capture on Port K (Mega A8..A13).

void     rxInit();
uint16_t rxGet(uint8_t ch);                 // pulse width in µs; 0 if never seen
bool     rxAlive(uint8_t ch, uint32_t now_us);  // false if no pulse in 100 ms
