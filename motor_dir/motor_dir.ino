// Single-motor spin direction check.
//
// Attaches ALL FOUR ESC outputs and holds them at DISARM, then ramps just
// the TARGET_MOTOR up to a slow steady speed. Keeping the other three ESCs
// fed with valid throttle-low pulses prevents them from going into
// no-signal mode and randomly twitching while you're trying to read the
// direction of the one you actually care about.
//
// Workflow per motor:
//   1. Edit TARGET_MOTOR below (0=M1, 1=M2, 2=M3, 3=M4).
//   2. Recompile + upload via USB (no battery).
//   3. Unplug USB, plug battery — selected motor reaches 1200 us after
//      ~7 s, others stay at DISARM.
//   4. Read the spin direction.
//   5. Disconnect battery, repeat for the next motor.
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
// PROPS MUST BE OFF.

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

Servo motors[4];

static void writeOthersDisarm() {
  for (uint8_t i = 0; i < 4; i++) {
    if (i != TARGET_MOTOR) motors[i].writeMicroseconds(US_DISARM);
  }
}

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
  Serial.println(F("# (other three motors held at DISARM)"));
  Serial.println(F("# ============================================"));

  // Attach ALL four ESCs and start them at DISARM. This keeps the unused
  // three ESCs in a known idle state instead of seeing no-signal noise.
  for (uint8_t i = 0; i < 4; i++) {
    motors[i].attach(MOTOR_PIN[i], US_DISARM, 2000);
    motors[i].writeMicroseconds(US_DISARM);
  }

  Serial.print(F("# holding DISARM on all four for ESC startup ("));
  Serial.print(ESC_BOOT_MS); Serial.println(F(" ms)..."));
  uint32_t t0 = millis();
  while (millis() - t0 < ESC_BOOT_MS) {
    for (uint8_t i = 0; i < 4; i++) motors[i].writeMicroseconds(US_DISARM);
    delay(20);
  }

  Serial.println(F("# ramping target motor up slowly..."));
  for (uint16_t us = US_DISARM; us <= US_HOLD; us += RAMP_STEP_US) {
    motors[TARGET_MOTOR].writeMicroseconds(us);
    writeOthersDisarm();
    delay(RAMP_STEP_MS);
  }
  Serial.print(F("# holding target at "));
  Serial.print(US_HOLD);
  Serial.println(F(" us forever — observe spin direction"));
}

void loop() {
  motors[TARGET_MOTOR].writeMicroseconds(US_HOLD);
  writeOthersDisarm();
  delay(50);
}
