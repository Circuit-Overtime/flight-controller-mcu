#pragma once
#include <Arduino.h>

// ESC driver — wraps the Servo library to send 50 Hz / 1000-2000 µs PWM to
// four BLDC ESCs. Same API as a higher-level motor abstraction so the FC
// pipeline stays identical when we later swap to OneShot/DShot on Teensy.

void motorsInit();                         // attach pins, send DISARM pulse
void motorsArmEscs();                      // hold DISARM long enough for ESCs to recognize
void motorsWrite(const uint16_t out[4]);   // send the four pulses (M1..M4)
void motorsDisarm();                       // immediately drive all four to DISARM
