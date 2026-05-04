// Single-motor spin direction check.
//
// Hard-coded to one motor at a time. To test a different motor, change
// TARGET_MOTOR below (0=M1, 1=M2, 2=M3, 3=M4), recompile, upload, and run.
// The selected motor is held at a steady slow speed (1200 us) for as long
// as the sketch is powered, so you can stare at it and identify CW vs CCW
// without rush.
//
// Expected directions (looking DOWN from above):
//   M1 front-right -> CCW
//   M2 rear-right  -> CW
//   M3 rear-left   -> CCW
//   M4 front-left  -> CW
//
// Wrong direction -> swap any two of the three thick motor wires between
// that ESC and that motor.
//
// PROPS OFF. Same power dance as the other test sketches:
//   - upload via USB (no battery)
//   - unplug USB, plug battery, watch the chosen motor

#include <Servo.h>

// >>> CHANGE THIS for each motor you want to test. <<<
#define TARGET_MOTOR  0    // 0=M1 front-right, 1=M2 rear-right, 2=M3 rear-left, 3=M4 front-left

static const uint8_t  MOTOR_PIN[4]   = { 52, 44, 48, 46 };
static const char*    MOTOR_LABEL[4] = { "M1 front-right (expect CCW)",
                                         "M2 rear-right  (expect CW )",
                                         "M3 rear-left   (expect CCW)",
                                         "M4 front-left  (expect CW )" };
static const uint16_t US_DISARM      = 1000;
static const uint16_t US_HOLD        = 1200;   // steady slow-spin level
static const uint32_t ESC_BOOT_MS    = 4000;
static const uint16_t RAMP_STEP_US   = 5;
static const uint32_t RAMP_STEP_MS   = 60;     // ~2.4 s ramp from 1000 to 1200

Servo motor;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("# ============================================"));
  Serial.println(F("# motor_dir — single-motor spin check"));
  Serial.println(F("# PROPS MUST BE OFF"));
  Serial.print  (F("# Testing: "));
  Serial.println(MOTOR_LABEL[TARGET_MOTOR]);
  Serial.print  (F("# Pin: "));
  Serial.println(MOTOR_PIN[TARGET_MOTOR]);
  Serial.println(F("# ============================================"));

  motor.attach(MOTOR_PIN[TARGET_MOTOR], US_DISARM, 2000);
  motor.writeMicroseconds(US_DISARM);

  Serial.print(F("# holding DISARM for ESC startup ("));
  Serial.print(ESC_BOOT_MS); Serial.println(F(" ms)..."));
  uint32_t t0 = millis();
  while (millis() - t0 < ESC_BOOT_MS) { motor.writeMicroseconds(US_DISARM); delay(20); }

  Serial.println(F("# ramping up slowly..."));
  for (uint16_t us = US_DISARM; us <= US_HOLD; us += RAMP_STEP_US) {
    motor.writeMicroseconds(us);
    delay(RAMP_STEP_MS);
  }
  Serial.print(F("# holding at "));
  Serial.print(US_HOLD);
  Serial.println(F(" us forever — observe spin direction"));
}

void loop() {
  motor.writeMicroseconds(US_HOLD);
  delay(50);
}
