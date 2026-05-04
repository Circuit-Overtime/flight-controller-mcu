#include "include/battery.h"
#include "include/config.h"

// Convert raw ADC count -> battery volts (full divider math).
static inline float _adc_to_volts(uint16_t raw) {
  const float v_adc      = (float)raw * (BATTERY_ADC_REF_V / (float)BATTERY_ADC_RES);
  const float divider    = (BATTERY_R_TOP_OHM + BATTERY_R_BOTTOM_OHM) /
                           BATTERY_R_BOTTOM_OHM;
  return v_adc * divider;
}

// Smoothed reading; EMA with alpha=0.2 (roughly 1 s time constant at 5 Hz
// poll). Slow reading is fine — the warning LED doesn't need to be reactive.
static float battery_volts = 0.0f;

void batteryInit() {
  pinMode(BATTERY_ADC_PIN, INPUT);
  // Seed the EMA with one direct reading so the first batteryIsLow() call
  // doesn't see a zero and falsely warn.
  battery_volts = _adc_to_volts(analogRead(BATTERY_ADC_PIN));
}

float batteryReadVolts() {
  const uint16_t raw = analogRead(BATTERY_ADC_PIN);
  const float    v   = _adc_to_volts(raw);
  battery_volts = 0.2f * v + 0.8f * battery_volts;
  return battery_volts;
}

bool batteryIsLow() {
  return battery_volts < (float)BATTERY_LOW_V;
}
