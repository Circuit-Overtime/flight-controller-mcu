#pragma once
#include <Arduino.h>

// Status LEDs — 4 simple indicators wired pin->resistor->LED->GND.
// Each Set* function takes a bool: true lights the LED, false turns it off.

void ledsInit();
void ledStartup(bool on);
void ledCalib(bool on);
void ledTempHigh(bool on);
void ledBatLow(bool on);
