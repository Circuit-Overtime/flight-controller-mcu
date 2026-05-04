// Motor spin-direction verification.
//
// Same idea as motor_test, but instead of sweeping up to 1300 us each motor
// is gently ramped from idle (1000 us) up to 1100 us — the slowest speed
// at which the bell still turns reliably. The slow ramp gives you a few
// seconds to watch the bell rotate and identify CW vs CCW *visually*
// before it spins up too fast to see.
//
// Order: M1 -> M2 -> M3 -> M4 (same as motor_test).
//
// What you should see (looking DOWN at the drone from above):
//   M1 (front-right) -> CCW
//   M2 (rear-right ) -> CW
//   M3 (rear-left  ) -> CCW
//   M4 (front-left ) -> CW
//
// If a motor spins the wrong direction, fix in HARDWARE: swap any two of
// the three thick motor wires between that ESC and that motor. (Software
// reversal isn't possible with standard PWM ESCs.)
//
// PROPS OFF. Same power rules as motor_test:
//   - Upload over USB (no battery).
//   - Unplug USB, plug battery; sketch starts after ESC arming hold.

#include <Servo.h>

static const uint8_t  MOTOR_PIN[4]    = { 52, 44, 48, 46 };
static const char*    MOTOR_LABEL[4]  = { "M1 front-right (expect CCW)",
                                          "M2 rear-right  (expect CW )",
                                          "M3 rear-left   (expect CCW)",
                                          "M4 front-left  (expect CW )" };
static const uint16_t US_DISARM       = 1000;
static const uint16_t US_SLOW_PEAK    = 1100;   // slowest reliable spin
static const uint32_t ESC_BOOT_MS     = 4000;
static const uint16_t RAMP_STEP_US    = 5;      // 5 us per step
static const uint32_t RAMP_STEP_MS    = 150;    // 150 ms per step
static const uint32_t HOLD_AT_PEAK_MS = 2000;
static const uint32_t COAST_MS        = 2500;   // gap with motor stopped

Servo motors[4];

static void writeAll(uint16_t us) {
  for (uint8_t i = 0; i < 4; i++) motors[i].writeMicroseconds(us);
}

static void runOne(uint8_t idx) {
  Serial.print(F("# >>> ")); Serial.println(MOTOR_LABEL[idx]);
  Serial.println(F("#     ramping slowly — watch which way the bell turns"));

  // Slow ramp from idle to slow-spin peak.
  for (uint16_t us = US_DISARM; us <= US_SLOW_PEAK; us += RAMP_STEP_US) {
    motors[idx].writeMicroseconds(us);
    delay(RAMP_STEP_MS);
  }
  // Hold at peak so you have a steady-state look.
  motors[idx].writeMicroseconds(US_SLOW_PEAK);
  delay(HOLD_AT_PEAK_MS);
  // Stop and let the bell coast down.
  motors[idx].writeMicroseconds(US_DISARM);
  delay(COAST_MS);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("# ============================================"));
  Serial.println(F("# motor_dir — spin direction check"));
  Serial.println(F("# PROPS MUST BE OFF"));
  Serial.println(F("# Each motor ramps 1000 -> 1100 us, slow spin."));
  Serial.println(F("# Order: M1 -> M2 -> M3 -> M4"));
  Serial.println(F("# ============================================"));

  for (uint8_t i = 0; i < 4; i++) {
    motors[i].attach(MOTOR_PIN[i], US_DISARM, 2000);
    motors[i].writeMicroseconds(US_DISARM);
  }
  Serial.print(F("# holding DISARM for ESC startup ("));
  Serial.print(ESC_BOOT_MS); Serial.println(F(" ms)..."));
  uint32_t t0 = millis();
  while (millis() - t0 < ESC_BOOT_MS) { writeAll(US_DISARM); delay(20); }
  Serial.println(F("# ESCs armed."));
}

void loop() {
  static bool done = false;
  if (!done) {
    for (uint8_t i = 0; i < 4; i++) runOne(i);
    Serial.println(F("# All four motors checked. Holding DISARM forever."));
    done = true;
  }
  writeAll(US_DISARM);
  delay(50);
}
