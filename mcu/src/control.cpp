#include "include/control.h"
#include "include/config.h"

// Arming gesture: throttle low + yaw far-right for ARM_HOLD_MS. Disarming uses
// a deliberate two-stick bottom-left corner for DISARM_HOLD_MS; yaw-left alone
// must never stop the motors in flight.

static bool     armed_;
static uint32_t gesture_start_ms_;
static uint32_t settle_until_ms_;   // 0 once post-arm settle window is over

void controlInit() {
  armed_            = false;
  gesture_start_ms_ = 0;
  settle_until_ms_  = 0;
}

void controlForceDisarm() {
  armed_            = false;
  gesture_start_ms_ = 0;
  settle_until_ms_  = 0;
}

// Map a centered stick (1500 ± half-range) to [-1, +1] with a deadband
// around center to prevent noise from creating ghost setpoints.
static float _stickNorm(uint16_t pwm) {
  int16_t delta = (int16_t)pwm - 1500;
  int16_t mag   = (delta < 0) ? -delta : delta;
  if (mag <= STICK_DEAD_BAND_US) return 0.0f;
  // Continuous output past the deadband: subtract deadband from magnitude.
  float n = (float)(mag - STICK_DEAD_BAND_US) /
            (float)(STICK_RANGE_HALF_US - STICK_DEAD_BAND_US);
  if (n > 1.0f) n = 1.0f;
  return (delta < 0) ? -n : n;
}

// Throttle: clamp to [STICK_THROTTLE_LO_US, STICK_THROTTLE_HI_US].
static float _throttleClamp(uint16_t pwm) {
  if (pwm < STICK_THROTTLE_LO_US) return (float)STICK_THROTTLE_LO_US;
  if (pwm > STICK_THROTTLE_HI_US) return (float)STICK_THROTTLE_HI_US;
  return (float)pwm;
}

Setpoints controlUpdate(uint16_t roll_us, uint16_t pitch_us,
                        uint16_t throttle_us, uint16_t yaw_us,
                        bool failsafe, uint32_t now_ms) {
  bool was_armed = armed_;
  bool suppress_for_disarm = false;

  // rxAlive() already tolerates missing frames for RX_ALIVE_TIMEOUT_US. Once
  // the caller reports failsafe, do not add another delay: disarm immediately.
  if (failsafe) {
    armed_            = false;
    gesture_start_ms_ = 0;
  } else {
    bool thr_low   = (throttle_us <= ARM_THROTTLE_MAX_US);
    bool yaw_left  = (yaw_us      <= ARM_YAW_LOW_US);
    bool yaw_right = (yaw_us      >= ARM_YAW_HIGH_US);
    bool disarm_corner = thr_low && yaw_left &&
                         (roll_us  <= DISARM_ROLL_LOW_US) &&
                         (pitch_us <= DISARM_PITCH_LOW_US);

    if (!armed_) {
      // Arm gesture: throttle low + yaw right.
      if (thr_low && yaw_right) {
        if (gesture_start_ms_ == 0) gesture_start_ms_ = now_ms;
        else if (now_ms - gesture_start_ms_ >= ARM_HOLD_MS) {
          armed_            = true;
          gesture_start_ms_ = 0;
        }
      } else {
        gesture_start_ms_ = 0;
      }
    } else {
      // Disarm gesture: both Mode-2 sticks held at the bottom-left corner.
      // Suppress commanded attitude/rate while it is held so the gesture does
      // not itself request a sustained roll, pitch, or yaw disturbance.
      if (disarm_corner) {
        suppress_for_disarm = true;
        if (gesture_start_ms_ == 0) gesture_start_ms_ = now_ms;
        else if (now_ms - gesture_start_ms_ >= DISARM_HOLD_MS) {
          armed_            = false;
          gesture_start_ms_ = 0;
        }
      } else {
        gesture_start_ms_ = 0;
      }
    }
  }

  // On the rising edge of armed, start a settle window during which roll /
  // pitch / yaw setpoints are forced to zero — pilot's hand is still on the
  // arming gesture (yaw stick far right), so honoring it would immediately
  // command a yaw spin and drive M1+M3 up while M2+M4 hit the floor.
  if ((!was_armed) && armed_) {
    settle_until_ms_ = now_ms + POST_ARM_SETTLE_MS;
  }
  bool suppress_sticks = (now_ms < settle_until_ms_) || suppress_for_disarm;

  // Map sticks to setpoints. Angle mode for roll/pitch; rate mode for yaw.
  float roll_n  = _stickNorm(roll_us);
  float pitch_n = _stickNorm(pitch_us);
  float yaw_n   = _stickNorm(yaw_us);

  Setpoints sp;
  sp.angle_roll_deg  = suppress_sticks ? 0.0f : (roll_n  * MAX_TILT_DEG);
  sp.angle_pitch_deg = suppress_sticks ? 0.0f : (pitch_n * MAX_TILT_DEG);
  sp.yaw_rate_dps    = suppress_sticks ? 0.0f : (yaw_n   * MAX_YAW_RATE_DPS);
  sp.throttle_us     =  _throttleClamp(throttle_us);
  sp.armed           =  armed_;
  sp.just_armed      = (!was_armed) && armed_;
  sp.just_disarmed   =  was_armed   && (!armed_);
  return sp;
}
