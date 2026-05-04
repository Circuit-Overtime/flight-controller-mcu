#include "include/leds.h"
#include "include/config.h"

void ledsInit() {
  pinMode(LED_STARTUP_PIN, OUTPUT);
  pinMode(LED_CALIB_PIN,   OUTPUT);
  pinMode(LED_TEMP_PIN,    OUTPUT);
  pinMode(LED_BATTERY_PIN, OUTPUT);
  digitalWrite(LED_STARTUP_PIN, LOW);
  digitalWrite(LED_CALIB_PIN,   LOW);
  digitalWrite(LED_TEMP_PIN,    LOW);
  digitalWrite(LED_BATTERY_PIN, LOW);
}

void ledStartup(bool on)  { digitalWrite(LED_STARTUP_PIN, on ? HIGH : LOW); }
void ledCalib(bool on)    { digitalWrite(LED_CALIB_PIN,   on ? HIGH : LOW); }
void ledTempHigh(bool on) { digitalWrite(LED_TEMP_PIN,    on ? HIGH : LOW); }
void ledBatLow(bool on)   { digitalWrite(LED_BATTERY_PIN, on ? HIGH : LOW); }
