// Standalone RX channel-mapping test for FS-R6B on Arduino Mega.
// Prints all 6 channel pulse widths once per second.
// Wiring: CH1..CH6 -> A8..A13.  V+ -> 5V.  GND -> GND.
//
// Use: move ONE stick on the TX at a time; watch the printout to learn
//      which physical stick / switch maps to which channel number.
//
// Upload:
//   arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega rx_test/
// Monitor:
//   arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200

static const uint8_t  RX_CHANNELS   = 6;
static const uint32_t RX_TIMEOUT_US = 100000UL;

volatile uint32_t rx_rise[RX_CHANNELS]        = {0};
volatile uint16_t rx_pulse[RX_CHANNELS]       = {0};
volatile uint32_t rx_last_update[RX_CHANNELS] = {0};
volatile uint8_t  rx_prev_state               = 0;

ISR(PCINT2_vect) {
  uint8_t state   = PINK;
  uint8_t changed = state ^ rx_prev_state;
  rx_prev_state   = state;
  uint32_t now    = micros();
  for (uint8_t i = 0; i < RX_CHANNELS; i++) {
    uint8_t mask = (uint8_t)(1 << i);
    if (!(changed & mask)) continue;
    if (state & mask) {
      rx_rise[i] = now;
    } else if (rx_rise[i] != 0) {
      uint16_t width = (uint16_t)(now - rx_rise[i]);
      if (width >= 800 && width <= 2200) {
        rx_pulse[i]       = width;
        rx_last_update[i] = now;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  DDRK   &= ~0x3F;
  PORTK  &= ~0x3F;
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= 0x3F;
  rx_prev_state = PINK;
  Serial.println(F("FS-R6B channel mapper - move one stick at a time"));
}

void loop() {
  delay(1000);
  uint32_t now_us = micros();
  for (uint8_t ch = 0; ch < RX_CHANNELS; ch++) {
    uint8_t s = SREG; cli();
    uint16_t w    = rx_pulse[ch];
    uint32_t last = rx_last_update[ch];
    SREG = s;
    bool alive = (last != 0) && ((now_us - last) < RX_TIMEOUT_US);
    Serial.print(F("CH")); Serial.print(ch + 1); Serial.print(F(":"));
    if (alive) {
      if (w < 1000) Serial.print(' ');
      Serial.print(w); Serial.print(F("us "));
    } else {
      Serial.print(F("  --- "));
    }
    Serial.print(' ');
  }
  Serial.println();
}
