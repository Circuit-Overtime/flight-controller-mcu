// FlySky FS-R6B receiver — 6-channel PWM capture via Pin-Change Interrupt.
// All 6 channels connect to Port K (Mega pins A8..A13 = PCINT16..21), so a
// single interrupt vector covers them all.
//
// Wiring: CH1..CH6 signal -> A8..A13.  V+ from any channel -> Mega 5V.  GND.

#include "rx.h"

static volatile uint32_t rx_rise[RX_NUM_CHANNELS]        = {0};
static volatile uint16_t rx_pulse[RX_NUM_CHANNELS]       = {0};
static volatile uint32_t rx_last_update[RX_NUM_CHANNELS] = {0};
static volatile uint8_t  rx_prev_state                   = 0;

ISR(PCINT2_vect) {
  uint8_t state   = PINK;
  uint8_t changed = state ^ rx_prev_state;
  rx_prev_state   = state;
  uint32_t now    = micros();

  for (uint8_t i = 0; i < RX_NUM_CHANNELS; i++) {
    uint8_t mask = (uint8_t)(1 << i);
    if (!(changed & mask)) continue;
    if (state & mask) {
      rx_rise[i] = now;
    } else if (rx_rise[i] != 0) {
      uint16_t width = (uint16_t)(now - rx_rise[i]);
      if (width >= RX_PULSE_MIN_US && width <= RX_PULSE_MAX_US) {
        rx_pulse[i]       = width;
        rx_last_update[i] = now;
      }
    }
  }
}

void rxInit() {
  DDRK   &= ~0x3F;            // A8..A13 inputs
  PORTK  &= ~0x3F;            // no pull-ups (RX is push-pull)
  PCICR  |= (1 << PCIE2);     // enable Port K pin-change group
  PCMSK2 |= 0x3F;             // unmask PCINT16..21
  rx_prev_state = PINK;
}

uint16_t rxGet(uint8_t ch) {
  if (ch >= RX_NUM_CHANNELS) return 0;
  uint8_t s = SREG; cli();
  uint16_t v = rx_pulse[ch];
  SREG = s;
  return v;
}

bool rxAlive(uint8_t ch, uint32_t now_us) {
  if (ch >= RX_NUM_CHANNELS) return false;
  uint8_t s = SREG; cli();
  uint32_t last = rx_last_update[ch];
  SREG = s;
  if (last == 0) return false;
  return (now_us - last) < RX_ALIVE_TIMEOUT_US;
}
