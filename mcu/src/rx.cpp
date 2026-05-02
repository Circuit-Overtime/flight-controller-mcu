// FlySky FS-R6B receiver — 4-channel PWM capture via external interrupts.
//
// Each AETR channel gets its own dedicated INT vector on the Mega 2560:
//   CH1 (roll)     -> pin 19 (PD2, INT2)
//   CH2 (pitch)    -> pin 18 (PD3, INT3)
//   CH3 (throttle) -> pin  2 (PE4, INT4)
//   CH4 (yaw)      -> pin  3 (PE5, INT5)
//
// Why external interrupts and not PCINT?
//   - Each channel has its own ISR — no port-state diff needed in the handler,
//     so the ISR is shorter and more deterministic.
//   - INT2..5 sit at lower vector numbers than PCINT2, so they win arbitration
//     when two interrupts go pending simultaneously (e.g. two channels edge
//     during the same I2C-blocked window).
//   - Edge sensitivity is configured in EICRA/EICRB; we use "any logical change"
//     so a single vector services both rising and falling edges.
//
// CH5/CH6 are not handled here. They will return rxGet=0/rxAlive=false until
// connected via a future PCINT or ICP path.

#include "include/rx.h"

static const uint8_t RX_HW_CHANNELS = 4;

static volatile uint32_t rx_rise[RX_NUM_CHANNELS]        = {0};
static volatile uint16_t rx_pulse[RX_NUM_CHANNELS]       = {0};
static volatile uint32_t rx_last_update[RX_NUM_CHANNELS] = {0};

// Read pin level directly from PIN register — faster than digitalRead in ISR.
static inline bool _read_ch1() { return (PIND & _BV(PD2)) != 0; }   // pin 19
static inline bool _read_ch2() { return (PIND & _BV(PD3)) != 0; }   // pin 18
static inline bool _read_ch3() { return (PINE & _BV(PE4)) != 0; }   // pin 2
static inline bool _read_ch4() { return (PINE & _BV(PE5)) != 0; }   // pin 3

// Common edge handler. Inlined into each ISR via the per-channel wrapper.
static inline void _handle_edge(uint8_t ch, bool level, uint32_t now) {
  if (level) {
    rx_rise[ch] = now;
  } else if (rx_rise[ch] != 0) {
    uint16_t width = (uint16_t)(now - rx_rise[ch]);
    if (width >= RX_PULSE_MIN_US && width <= RX_PULSE_MAX_US) {
      rx_pulse[ch]       = width;
      rx_last_update[ch] = now;
    }
  }
}

ISR(INT2_vect) { _handle_edge(0, _read_ch1(), micros()); }
ISR(INT3_vect) { _handle_edge(1, _read_ch2(), micros()); }
ISR(INT4_vect) { _handle_edge(2, _read_ch3(), micros()); }
ISR(INT5_vect) { _handle_edge(3, _read_ch4(), micros()); }

void rxInit() {
  // Pins as inputs, no internal pull-ups (RX is push-pull).
  pinMode(RX_CH1_PIN, INPUT);
  pinMode(RX_CH2_PIN, INPUT);
  pinMode(RX_CH3_PIN, INPUT);
  pinMode(RX_CH4_PIN, INPUT);

  // Configure edge sensitivity to "any logical change" for INT2..5.
  // EICRA layout: ISC30, ISC31 | ISC20, ISC21 | ISC10, ISC11 | ISC00, ISC01
  // EICRB layout: ISC70, ISC71 | ISC60, ISC61 | ISC50, ISC51 | ISC40, ISC41
  // ISCx0=1, ISCx1=0  =>  any-edge trigger.
  EICRA &= ~((1 << ISC21) | (1 << ISC31));
  EICRA |=  ((1 << ISC20) | (1 << ISC30));
  EICRB &= ~((1 << ISC41) | (1 << ISC51));
  EICRB |=  ((1 << ISC40) | (1 << ISC50));

  // Clear any pending flags that accumulated during config.
  EIFR  = (1 << INTF2) | (1 << INTF3) | (1 << INTF4) | (1 << INTF5);

  // Enable INT2..INT5.
  EIMSK |= (1 << INT2) | (1 << INT3) | (1 << INT4) | (1 << INT5);
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
