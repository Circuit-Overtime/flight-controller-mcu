// Quick wiring sanity check for the 4 status LEDs on pins 24, 26, 28, 30.
//
// Each pass: lights LED 24 alone for 0.5 s, then 26, then 28, then 30. Then
// all four together for 1 s, then all off for 0.5 s. Repeats forever.
//
// Verifies:
//   - correct pin -> LED mapping (you'll see them light in order)
//   - LED polarity (if a particular LED never lights, its anode/cathode is
//     reversed)
//   - resistor in series (no smoke, current limited)
//   - common ground with Mega
//
// Upload via USB:
//   arduino-cli compile --fqbn arduino:avr:mega led_test/
//   arduino-cli upload  -p /dev/ttyACM0 --fqbn arduino:avr:mega led_test/

static const uint8_t LED_PIN[4] = { 24, 26, 28, 30 };
static const char*   LED_NAME[4] = { "STARTUP (24)",
                                     "CALIB   (26)",
                                     "TEMP    (28)",
                                     "BATTERY (30)" };

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(LED_PIN[i], OUTPUT);
    digitalWrite(LED_PIN[i], LOW);
  }
  Serial.println(F("# led_test: blinks each LED in sequence."));
}

void loop() {
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(F("# on: ")); Serial.println(LED_NAME[i]);
    digitalWrite(LED_PIN[i], HIGH);
    delay(500);
    digitalWrite(LED_PIN[i], LOW);
    delay(100);
  }
  // All four together so you can see they share GND properly.
  Serial.println(F("# all on"));
  for (uint8_t i = 0; i < 4; i++) digitalWrite(LED_PIN[i], HIGH);
  delay(1000);
  for (uint8_t i = 0; i < 4; i++) digitalWrite(LED_PIN[i], LOW);
  delay(500);
}
