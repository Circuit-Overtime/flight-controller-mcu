// Standalone motor sweep test for the F450 build.
//
// Spins each motor one at a time from idle (1000 us) up to a gentle 1300 us
// and back down again, in the order M1 -> M2 -> M3 -> M4. Lets you verify
// pin-to-motor mapping ("does the front-right motor really run on pin 52?")
// and motor spin direction (looking down: should be CCW for M1+M3, CW for
// M2+M4) without any FC logic, PID, or RX involvement.
//
// SAFETY:
//   - PROPS MUST BE OFF.
//   - Battery connected, USB connected (or one or the other).
//   - The sketch will NOT spin motors on its own. You have to send 'g' over
//     serial to start each round. Anything else stops.
//
// Wiring (matches the main FC sketch):
//   M1 (front-right, CCW) signal -> pin 52
//   M2 (rear-right,  CW)  signal -> pin 50
//   M3 (rear-left,   CCW) signal -> pin 48
//   M4 (front-left,  CW)  signal -> pin 46
//
// Upload + monitor:
//   arduino-cli upload  -p /dev/ttyACM0 --fqbn arduino:avr:mega motor_test/
//   arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200

#include <Servo.h>

static const uint8_t  MOTOR_PIN[4]   = { 52, 50, 48, 46 };
static const char*    MOTOR_NAME[4]  = { "M1 (front-right, CCW)",
                                         "M2 (rear-right,  CW )",
                                         "M3 (rear-left,   CCW)",
                                         "M4 (front-left,  CW )" };
static const uint16_t US_DISARM      = 1000;
static const uint16_t US_TEST_MAX    = 1300;   // gentle — no props, low RPM
static const uint32_t ESC_BOOT_MS    = 2500;   // ESCs need this much DISARM
static const uint32_t SWEEP_HALF_MS  = 1500;   // ramp up + ramp down

Servo motors[4];

static void writeAll(uint16_t us) {
  for (uint8_t i = 0; i < 4; i++) motors[i].writeMicroseconds(us);
}

static void waitForGo() {
  Serial.println(F("# Send 'g' to start; any other char stops."));
  while (Serial.available() == 0) {
    writeAll(US_DISARM);  // keep ESCs happy while we wait
    delay(50);
  }
  char c = (char)Serial.read();
  while (Serial.available() > 0) Serial.read();
  if (c != 'g' && c != 'G') {
    Serial.println(F("# stop requested. Holding DISARM forever."));
    while (true) { writeAll(US_DISARM); delay(100); }
  }
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
  // Hold a moment so you can clearly see/hear which motor it is.
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
  delay(800);  // gap before next motor
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}  // wait for USB CDC on USB-serial Mega clones
  Serial.println();
  Serial.println(F("# ============================================"));
  Serial.println(F("# motor_test — PROPS MUST BE OFF"));
  Serial.println(F("# Each motor sweeps 1000us -> 1300us -> 1000us"));
  Serial.println(F("# Order: M1, M2, M3, M4"));
  Serial.println(F("# ============================================"));

  for (uint8_t i = 0; i < 4; i++) {
    motors[i].attach(MOTOR_PIN[i], US_DISARM, 2000);
    motors[i].writeMicroseconds(US_DISARM);
  }
  // Hold DISARM long enough for ESCs to recognize throttle-low and arm.
  Serial.print(F("# holding DISARM for ESC startup ("));
  Serial.print(ESC_BOOT_MS); Serial.println(F(" ms)..."));
  uint32_t t0 = millis();
  while (millis() - t0 < ESC_BOOT_MS) { writeAll(US_DISARM); delay(20); }
}

void loop() {
  waitForGo();
  for (uint8_t i = 0; i < 4; i++) sweepOne(i);
  Serial.println(F("# round complete. Send 'g' for another, anything else to stop."));
}
