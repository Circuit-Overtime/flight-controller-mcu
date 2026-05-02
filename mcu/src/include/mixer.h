#pragma once
#include <Arduino.h>

// X-quad motor mixer.
//
// Layout (looking down from above, +X is "front"):
//
//        FRONT
//    M4 . . . M1
//     .       .
//     .       .
//    M3 . . . M2
//        REAR
//
// Spin direction (CCW means counter-clockwise looking down):
//   M1 front-right  CCW
//   M2 rear-right   CW
//   M3 rear-left    CCW
//   M4 front-left   CW
//
// Sign convention for control inputs (all in PWM µs):
//   throttle_us   — base throttle; M1..M4 all add this
//   roll_us       — positive = roll right (right side dips). Right motors
//                   slow (-roll_us); left motors speed up (+roll_us).
//   pitch_us      — positive = pitch up (nose lifts). Front motors slow
//                   (-pitch_us); rear motors speed up (+pitch_us).
//   yaw_us        — positive = yaw right (nose swings right). Net body
//                   torque must be CW; CCW props produce CW reaction torque,
//                   so CCW motors (M1, M3) get +yaw_us.
//
// Output is clamped to [MOTOR_MIN_US, MOTOR_MAX_US].

void mixerComputeXQuad(float throttle_us, float roll_us, float pitch_us, float yaw_us,
                       uint16_t out[4]);
