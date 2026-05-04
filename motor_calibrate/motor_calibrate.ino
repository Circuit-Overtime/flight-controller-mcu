// One-shot ESC throttle range calibration for all 4 ESCs.
//
// Why run this:
//   Different ESCs ship with different default throttle ranges. Mismatched
//   ranges cause asymmetric thrust (one motor is "ahead" of the others at
//   the same PWM) and break PID tuning. This sketch programs all four ESCs
//   to the SAME 1000..2000 us range so they respond identically.
//
// PROCEDURE — read this BEFORE running:
//   0. PROPS MUST BE OFF.
//   1. Upload this sketch via USB (battery NOT connected).
//   2. Unplug USB.
//   3. Plug in the LiPo. The sketch is already running, so the Mega
//      starts sending MAX-throttle (2000 us) the moment ESCs get power.
//      ESCs detect "throttle high at power-up" and enter calibration
//      mode. You'll hear a sequence of confirmation beeps within ~3 s
//      ("MAX position registered").
//   4. After 6 s the sketch drops to MIN-throttle (1000 us). ESCs
//      register the low endpoint and beep again ("MIN position
//      registered, calibration complete"). Their stored range is now
//      1000..2000 us.
//   5. Wait 4 more seconds, ESCs go fully armed at idle. Sketch then
//      holds DISARM forever.
//   6. Disconnect battery. Re-upload either motor_test/ or mcu/ as
//      needed for the next test.
//
// SAFETY:
//   Props off, drone secured. Motors WILL try to spin during the MIN
//   phase if the calibration completes — at idle they're slow but real.

#include <Servo.h>

static const uint8_t  MOTOR_PIN[4]  = { 52, 44, 48, 46 };  // M1, M2, M3, M4
static const uint16_t US_MIN        = 1000;
static const uint16_t US_MAX        = 2000;
static const uint32_t HOLD_MAX_MS   = 6000;  // ESCs need to see HIGH for ~3-5 s
static const uint32_t HOLD_MIN_MS   = 4000;  // and LOW for ~2-4 s to confirm

Servo motors[4];

static void writeAll(uint16_t us) {
  for (uint8_t i = 0; i < 4; i++) motors[i].writeMicroseconds(us);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("# ============================================"));
  Serial.println(F("# ESC throttle range calibration"));
  Serial.println(F("# PROPS MUST BE OFF"));
  Serial.println(F("# ============================================"));

  // Attach all four pins immediately and start sending MAX throttle, so
  // when the user plugs in the battery the ESCs see HIGH from the very
  // first PWM frame.
  for (uint8_t i = 0; i < 4; i++) {
    motors[i].attach(MOTOR_PIN[i], US_MIN, US_MAX);
    motors[i].writeMicroseconds(US_MAX);
  }

  Serial.print(F("# phase 1: MAX throttle for "));
  Serial.print(HOLD_MAX_MS);
  Serial.println(F(" ms"));
  Serial.println(F("# (ESCs should beep MAX-registered confirmation)"));
  uint32_t t0 = millis();
  while (millis() - t0 < HOLD_MAX_MS) { writeAll(US_MAX); delay(20); }

  Serial.print(F("# phase 2: MIN throttle for "));
  Serial.print(HOLD_MIN_MS);
  Serial.println(F(" ms"));
  Serial.println(F("# (ESCs should beep MIN-registered confirmation)"));
  t0 = millis();
  while (millis() - t0 < HOLD_MIN_MS) { writeAll(US_MIN); delay(20); }

  Serial.println(F("# Calibration complete. All ESCs now at 1000..2000 us range."));
  Serial.println(F("# Disconnect battery, re-upload motor_test/ or mcu/ as needed."));
}

void loop() {
  writeAll(US_MIN);
  delay(50);
}
