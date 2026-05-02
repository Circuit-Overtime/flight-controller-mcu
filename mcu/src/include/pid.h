#pragma once
#include <Arduino.h>

// Single-axis PID controller with anti-windup and "D-on-measurement".
//
// Why D on measurement (not on error)? When a stick command snaps from 0 to
// full deflection, the setpoint jumps but the drone hasn't moved yet — so an
// "error derivative" would spike enormously and slap the motors. Differen-
// tiating the *measurement* alone gives smooth D behavior under stick input.
//
// Anti-windup: integral term is hard-clamped to |i_limit_us|. Combined with
// reset() (called when disarmed) this keeps integrators sane through stick
// flicks and ESC saturation.

class Pid {
public:
  Pid(float kp, float ki, float kd, float i_limit_us, float out_limit_us);

  // Compute the controller output. setpoint and measured share units (deg/s
  // for rate loops, deg for angle loop). dt_s is the loop period in seconds.
  // Output is clamped to ±out_limit_us.
  float update(float setpoint, float measured, float dt_s);

  // Zero the integrator and D-state. Call when arming or after a major mode
  // change so the controller starts from a known-clean state.
  void  reset();

  // Hot-swap gains (e.g. while tuning over telemetry). Does not reset state.
  void  setGains(float kp, float ki, float kd);

  float lastP() const { return p_term_; }
  float lastI() const { return i_term_; }
  float lastD() const { return d_term_; }

private:
  float kp_, ki_, kd_;
  float i_limit_us_;
  float out_limit_us_;

  float integral_;
  float prev_meas_;
  bool  first_;

  // Cached for telemetry / tuning visualization.
  float p_term_, i_term_, d_term_;
};
