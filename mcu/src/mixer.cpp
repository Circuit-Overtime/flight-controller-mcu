#include <math.h>
#include "include/mixer.h"
#include "include/config.h"

static inline uint16_t _clamp_us(float v) {
  // NaN guard: (uint16_t)NaN is undefined and on AVR resolves to 0, which
  // would silently send a totally invalid pulse to the ESC. Treat NaN as the
  // safest bet — minimum commandable throttle.
  if (isnan(v)) return MOTOR_MIN_US;
  if (v < (float)MOTOR_MIN_US) return MOTOR_MIN_US;
  if (v > (float)MOTOR_MAX_US) return MOTOR_MAX_US;
  return (uint16_t)v;
}

void mixerComputeXQuad(float throttle_us, float roll_us, float pitch_us, float yaw_us,
                       uint16_t out[4]) {
  // Standard betaflight-compatible X-quad mix. See mixer.h for sign rationale.
  float m1 = throttle_us - roll_us - pitch_us + yaw_us;  // FR, CCW
  float m2 = throttle_us - roll_us + pitch_us - yaw_us;  // RR, CW
  float m3 = throttle_us + roll_us + pitch_us + yaw_us;  // RL, CCW
  float m4 = throttle_us + roll_us - pitch_us - yaw_us;  // FL, CW

  out[0] = _clamp_us(m1);
  out[1] = _clamp_us(m2);
  out[2] = _clamp_us(m3);
  out[3] = _clamp_us(m4);
}
