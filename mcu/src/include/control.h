#pragma once
#include <Arduino.h>

// Control layer: stick → setpoint mapping, arming state machine, failsafe.
//
// The FC pipeline is:
//   RX(raw us)  --> control  --> PID    --> mixer --> motors
//                   ^^^^^^^^      ^^^^      ^^^^^
//                   THIS LAYER    rate      X-quad
//                                 loop      mix
//
// This module produces a Setpoints struct each loop tick. It does NOT touch
// motors directly — that's the mixer's job. It only signals "armed" so the
// upper layer can decide whether to honor the setpoints or send disarm.

struct Setpoints {
  // Angle mode: roll/pitch are commanded tilt in degrees. The outer angle
  // PID converts tilt error into a rate setpoint for the inner rate PIDs.
  float angle_roll_deg;
  float angle_pitch_deg;

  // Yaw is rate-controlled even in angle mode (we don't track absolute
  // heading without a magnetometer).
  float yaw_rate_dps;

  // Base throttle for the mixer (PWM µs). Below MOTOR_MIN_US when disarmed.
  float throttle_us;

  bool armed;
  bool just_armed;    // true for one tick when arming transitions off→on
  bool just_disarmed; // true for one tick when arming transitions on→off
};

void      controlInit();

// All four channel inputs are post-offset, post-clamp µs (1000..2000).
// failsafe = true when any flight-critical channel hasn't seen a pulse
// recently (caller decides; we just respect the flag).
Setpoints controlUpdate(uint16_t roll_us, uint16_t pitch_us,
                        uint16_t throttle_us, uint16_t yaw_us,
                        bool failsafe, uint32_t now_ms);
