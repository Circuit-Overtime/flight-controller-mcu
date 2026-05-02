#include "pid.h"

static inline float _clamp(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

Pid::Pid(float kp, float ki, float kd, float i_limit_us, float out_limit_us)
    : kp_(kp), ki_(ki), kd_(kd),
      i_limit_us_(i_limit_us), out_limit_us_(out_limit_us),
      integral_(0.0f), prev_meas_(0.0f), first_(true),
      p_term_(0.0f), i_term_(0.0f), d_term_(0.0f) {}

void Pid::reset() {
  integral_  = 0.0f;
  prev_meas_ = 0.0f;
  first_     = true;
  p_term_ = i_term_ = d_term_ = 0.0f;
}

void Pid::setGains(float kp, float ki, float kd) {
  kp_ = kp; ki_ = ki; kd_ = kd;
}

float Pid::update(float setpoint, float measured, float dt_s) {
  float error = setpoint - measured;

  // Proportional.
  p_term_ = kp_ * error;

  // Integral with hard clamp (anti-windup).
  integral_ += error * dt_s;
  float raw_i = ki_ * integral_;
  i_term_ = _clamp(raw_i, -i_limit_us_, i_limit_us_);
  // If we clamped, also pull the integrator back so it doesn't keep growing.
  if (raw_i != i_term_ && ki_ != 0.0f) integral_ = i_term_ / ki_;

  // Derivative on measurement (not error). Negative sign because d/dt(error)
  // = -d/dt(measured) when setpoint is held — and the controller wants to
  // resist *change in measurement*, regardless of what setpoint did.
  if (first_) {
    d_term_   = 0.0f;
    first_    = false;
  } else {
    float dmeas = (measured - prev_meas_) / dt_s;
    d_term_ = -kd_ * dmeas;
  }
  prev_meas_ = measured;

  float out = p_term_ + i_term_ + d_term_;
  return _clamp(out, -out_limit_us_, out_limit_us_);
}
