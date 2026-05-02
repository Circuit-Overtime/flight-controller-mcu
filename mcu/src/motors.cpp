#include <Servo.h>
#include "include/motors.h"
#include "include/config.h"

static Servo m1, m2, m3, m4;

void motorsInit() {
  m1.attach(MOTOR_M1_PIN, MOTOR_DISARM_US, MOTOR_MAX_US);
  m2.attach(MOTOR_M2_PIN, MOTOR_DISARM_US, MOTOR_MAX_US);
  m3.attach(MOTOR_M3_PIN, MOTOR_DISARM_US, MOTOR_MAX_US);
  m4.attach(MOTOR_M4_PIN, MOTOR_DISARM_US, MOTOR_MAX_US);
  motorsDisarm();
}

void motorsArmEscs() {
  // Hold the disarm pulse long enough for cheap ESCs to register it as the
  // "throttle low" reference and accept run commands afterward.
  uint32_t deadline = millis() + ESC_BOOT_HOLD_MS;
  while (millis() < deadline) {
    motorsDisarm();
    delay(20);
  }
}

void motorsWrite(const uint16_t out[4]) {
  m1.writeMicroseconds(out[0]);
  m2.writeMicroseconds(out[1]);
  m3.writeMicroseconds(out[2]);
  m4.writeMicroseconds(out[3]);
}

void motorsDisarm() {
  m1.writeMicroseconds(MOTOR_DISARM_US);
  m2.writeMicroseconds(MOTOR_DISARM_US);
  m3.writeMicroseconds(MOTOR_DISARM_US);
  m4.writeMicroseconds(MOTOR_DISARM_US);
}
