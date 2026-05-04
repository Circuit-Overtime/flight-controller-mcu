#pragma once
#include <Arduino.h>

// 3S Li-ION battery monitoring via voltage divider on BATTERY_ADC_PIN.
//
// Resistor values, ADC reference, and the low-voltage threshold are in
// config.h. The voltage reading is lightly EMA-smoothed across calls so
// brief brownouts under motor inrush don't flicker the warning LED.

void  batteryInit();
float batteryReadVolts();   // most recent smoothed reading, in volts
bool  batteryIsLow();       // true if smoothed voltage < BATTERY_LOW_V
