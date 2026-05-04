#include "include/control.h"
#include "include/config.h"

// Arming gesture: throttle below ARM_THROTTLE_MAX_US AND yaw stick parked at
// far-right (>= ARM_YAW_HIGH_US) for ARM_HOLD_MS ms triggers arm. Same with
// yaw far-left to disarm. Holding the gesture is the safety: a brief flick
// won't arm the drone.

static bool     armed_;
static uint32_t gesture_start_ms_;
static uint16_t failsafe_streak_;   // consecutive failsafe-true ticks
static uint32_t settle_until_ms_;   // 0 once post-arm settle window is over

// A single tick of failsafe shouldn't kill an in-progress arm gesture —
// FlySky RXs occasionally drop one frame. Require this many consecutive
// failsafe ticks before we treat it as a real signal loss and disarm.
static const uint16_t FAILSAFE_DEBOUNCE_TICKS = 4;

void controlInit() {
  armed_            = false;
  gesture_start_ms_ = 0;
  failsafe_streak_  = 0;
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

  // Failsafe: only act on it after FAILSAFE_DEBOUNCE_TICKS consecutive ticks
  // — a single missed PWM frame from the RX shouldn't reset arming or kill
  // the in-progress gesture timer.
  if (failsafe) {
    if (failsafe_streak_ < 0xFFFF) failsafe_streak_++;
  } else {
    failsafe_streak_ = 0;
  }
  bool real_failsafe = (failsafe_streak_ >= FAILSAFE_DEBOUNCE_TICKS);

  if (real_failsafe) {
    armed_            = false;
    gesture_start_ms_ = 0;
  } else {
    bool thr_low   = (throttle_us <= ARM_THROTTLE_MAX_US);
    bool yaw_left  = (yaw_us      <= ARM_YAW_LOW_US);
    bool yaw_right = (yaw_us      >= ARM_YAW_HIGH_US);

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
      // Disarm gesture: throttle low + yaw left.
      if (thr_low && yaw_left) {
        if (gesture_start_ms_ == 0) gesture_start_ms_ = now_ms;
        else if (now_ms - gesture_start_ms_ >= ARM_HOLD_MS) {
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
  bool settling = (now_ms < settle_until_ms_);

  // Map sticks to setpoints. Angle mode for roll/pitch; rate mode for yaw.
  float roll_n  = _stickNorm(roll_us);
  float pitch_n = _stickNorm(pitch_us);
  float yaw_n   = _stickNorm(yaw_us);

  Setpoints sp;
  sp.angle_roll_deg  =  settling ? 0.0f : (roll_n  * MAX_TILT_DEG);
  sp.angle_pitch_deg =  settling ? 0.0f : (pitch_n * MAX_TILT_DEG);
  sp.yaw_rate_dps    =  settling ? 0.0f : (yaw_n   * MAX_YAW_RATE_DPS);
  sp.throttle_us     =  _throttleClamp(throttle_us);
  sp.armed           =  armed_;
  sp.just_armed      = (!was_armed) && armed_;
  sp.just_disarmed   =  was_armed   && (!armed_);
  return sp;
}
