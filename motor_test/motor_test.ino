// Standalone motor sweep test for the F450 build.
//
// Autonomous: on power-up the sketch sends DISARM (1000 us) to all four ESCs
// long enough to clear their startup beeps, then sweeps each motor once in
// order M1 -> M2 -> M3 -> M4 (gentle ramp 1000 -> 1300 -> 1000 us). After
// all four have been tested it idles all motors at DISARM forever.
//
// No serial input required — you can run this on battery power with the
// USB unplugged. Lets you verify pin-to-motor mapping and motor spin
// direction without any FC, PID, or RX involvement.
//
// SAFETY:
//   - PROPS MUST BE OFF.
//   - The sketch starts spinning ~4 seconds after power-up. If you need
//     more time, hit the Mega RESET button to abort and re-arm.
//
// Power modes (BEC red wires connected to Mega 5V):
//   - USB plugged in, no battery -> Mega runs, ESCs have no power, no spin.
//   - Battery plugged in, USB unplugged -> ESCs power Mega via BEC, motors
//     can spin. This is when the sketch actually exercises the motors.
//
// Wiring (matches the main FC sketch):
//   M1 (front-right, CCW) signal -> pin 52
//   M2 (rear-right,  CW)  signal -> pin 44   (was 50; pin 50 wouldn't drive ESC2)
//   M3 (rear-left,   CCW) signal -> pin 48
//   M4 (front-left,  CW)  signal -> pin 46
//
// Upload (USB, no battery):
//   arduino-cli compile --fqbn arduino:avr:mega motor_test/
//   arduino-cli upload  -p /dev/ttyACM0 --fqbn arduino:avr:mega motor_test/
// Then unplug USB, plug in battery; the sketch starts the sweep ~4 s later.

#include <Servo.h>

static const uint8_t  MOTOR_PIN[4]   = { 52, 44, 48, 46 };
static const char*    MOTOR_NAME[4]  = { "M1 (front-right, CCW)",
                                         "M2 (rear-right,  CW )",
                                         "M3 (rear-left,   CCW)",
                                         "M4 (front-left,  CW )" };
static const uint16_t US_DISARM      = 1000;
static const uint16_t US_TEST_MAX    = 1300;   // gentle — no props, low RPM
static const uint32_t ESC_BOOT_MS    = 4000;   // long enough to clear startup beeps
static const uint32_t SWEEP_HALF_MS  = 1500;   // ramp up + ramp down each ~1.5 s
static const uint32_t INTER_MS       = 800;    // pause between motors

Servo motors[4];

static void writeAll(uint16_t us) {
  for (uint8_t i = 0; i < 4; i++) motors[i].writeMicroseconds(us);
}

static void sweepOne(uint8_t idx) {
  Serial.print(F("# >>> ")); Serial.print(MOTOR_NAME[idx]);
  Serial.print(F("  pin=")); Serial.println(MOTOR_PIN[idx]);

  // Ramp up.
  uint32_t t0 = millis();
  while (millis() - t0 < SWEEP_HALF_MS) {
    float u = (float)(millis() - t0) / (float)SWEEP_HALF_MS;
    uint16_t us = US_DISARM + (uint16_t)(u * (US_TEST_MAX - US_DISARM));
    motors[idx].writeMicroseconds(us);
    delay(15);
  }
  // Hold so the spin is unmistakable.
  motors[idx].writeMicroseconds(US_TEST_MAX);
  delay(500);
  // Ramp down.
  t0 = millis();
  while (millis() - t0 < SWEEP_HALF_MS) {
    float u = (float)(millis() - t0) / (float)SWEEP_HALF_MS;
    uint16_t us = US_TEST_MAX - (uint16_t)(u * (US_TEST_MAX - US_DISARM));
    motors[idx].writeMicroseconds(us);
    delay(15);
  }
  motors[idx].writeMicroseconds(US_DISARM);
  delay(INTER_MS);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("# ============================================"));
  Serial.println(F("# motor_test — PROPS MUST BE OFF"));
  Serial.println(F("# Sweeping M1 -> M2 -> M3 -> M4 once."));
  Serial.println(F("# ============================================"));

  for (uint8_t i = 0; i < 4; i++) {
    motors[i].attach(MOTOR_PIN[i], US_DISARM, 2000);
    motors[i].writeMicroseconds(US_DISARM);
  }
  // ESC calibration / startup-beep clearance: hold DISARM continuously so
  // the ESCs see a valid throttle-low signal, run their cell-count beeps,
  // and arm. Any cheap ESC that auto-calibrates throttle range will pick
  // 1000 us as its low endpoint here.
  Serial.print(F("# holding DISARM for ESC startup ("));
  Serial.print(ESC_BOOT_MS); Serial.println(F(" ms)..."));
  uint32_t t0 = millis();
  while (millis() - t0 < ESC_BOOT_MS) { writeAll(US_DISARM); delay(20); }
  Serial.println(F("# ESCs armed. Beginning sweep."));
}

void loop() {
  static bool done = false;
  if (!done) {
    for (uint8_t i = 0; i < 4; i++) sweepOne(i);
    Serial.println(F("# All motors swept. Holding DISARM forever."));
    Serial.println(F("# Reset the board to run again."));
    done = true;
  }
  writeAll(US_DISARM);
  delay(50);
}
