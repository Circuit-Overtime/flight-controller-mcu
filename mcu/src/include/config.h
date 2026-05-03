#pragma once

// =============================================================================
// flight-controller-mcu  --  central configuration
// -----------------------------------------------------------------------------
// All hardware-specific values live here. Changing target boards (Mega -> Teensy
// -> custom PCB) should mostly mean swapping this file, not the logic modules.
// Group constants by subsystem; keep comments short but explanatory.
// =============================================================================


// ---- Serial telemetry -------------------------------------------------------
// 230400 baud: ~23 kchar/s, comfortably above the ~15 kchar/s the 22-field
// CSV needs at 100 Hz. Picked over 250000 because Linux termios rejects
// non-standard rates (250 kbaud isn't on the kernel's whitelist).
#define TELEMETRY_BAUD          230400UL
#define TELEMETRY_HZ            100      // CSV stream rate to host


// ---- IMU: MPU6050 over I2C --------------------------------------------------
#define MPU_I2C_ADDR            0x68
#define MPU_I2C_CLOCK           400000UL // Hz; chip datasheet max is 400 kHz
#define MPU_SAMPLE_RATE_DIVIDER 0x07     // base 1 kHz / (1 + 7) = 125 Hz
#define MPU_DLPF_CONFIG         0x03     // ~44 Hz low-pass on accel+gyro
#define MPU_GYRO_FS             0x00     // 0=±250 dps, 1=±500, 2=±1000, 3=±2000
#define MPU_ACCEL_FS            0x00     // 0=±2g, 1=±4g, 2=±8g, 3=±16g

// Sensitivity at the FS settings above (datasheet §6.1, §6.2).
#define MPU_ACCEL_LSB_PER_G     16384.0f
#define MPU_GYRO_LSB_PER_DPS    131.0f

#define MPU_CALIB_SAMPLES       2000     // ~4 s @ ~500 Hz inner loop


// ---- Attitude estimator -----------------------------------------------------
// Complementary filter weighting. Higher = trust gyro more (smoother but slower
// gravity correction); lower = trust accel more (faster correction, more noise).
#define COMP_FILTER_ALPHA       0.98f


// ---- Receiver: FlySky FS-R6B PWM, 4 channels (AETR) -------------------------
// Channel-to-axis mapping (verified against this user's TX, 2026-04-30):
//   CH1 = roll      (right stick L/R)
//   CH2 = pitch     (right stick U/D)
//   CH3 = throttle  (left stick U/D, no spring return)
//   CH4 = yaw       (left stick L/R)
//   CH5/CH6 unused for now (will be arm-switch + flight-mode later)
#define RX_NUM_CHANNELS         6        // streamed; only 1..4 are wired

// Pin assignments: all six channels on Port K (Mega A8..A13 = PCINT16..21).
// Single PCINT2_vect ISR services all of them — see rx.cpp for the mask.
#define RX_CH1_PIN              A8
#define RX_CH2_PIN              A9
#define RX_CH3_PIN              A10
#define RX_CH4_PIN              A11
#define RX_CH5_PIN              A12
#define RX_CH6_PIN              A13

// Pulse-width validation; values outside this band are rejected as noise.
#define RX_PULSE_MIN_US         800
#define RX_PULSE_MAX_US         2200

// A channel is "alive" if it has produced a valid pulse in the last N µs.
#define RX_ALIVE_TIMEOUT_US     100000UL // 100 ms = 5 PWM frames at 50 Hz

// Output smoothing + center calibration.
#define RX_EMA_ALPHA            0.20f    // lower = smoother but laggier
#define RX_CALIB_SAMPLES        400      // ~2 s of stick-centered sampling


// ---- Diagnostics ------------------------------------------------------------
// Channels that should read ~1500 µs at rest (used for boot offset calibration).
// Index matches RX channel - 1.
#define RX_IS_CENTERED_INIT     { true, true, false, true, false, false }


// =============================================================================
// FLIGHT CONTROLLER
// =============================================================================
// Airframe: F450, X-quad, 450 mm motor-to-motor, ~900 g AUW estimated.
// Power:    3S 11.1 V LiION, 1400 KV BLDC, 30 A SparkFun ESC.
// Mode:     indoor self-leveling hover.

// ---- Loop rates -------------------------------------------------------------
#define FC_LOOP_HZ              200      // inner rate-PID loop frequency (Hz)
#define FC_DT_S                 (1.0f / (float)FC_LOOP_HZ)


// ---- Motor outputs (Servo PWM) ---------------------------------------------
// All four ESCs run standard 50 Hz / 1000-2000 µs PWM via the Servo library.
// Note: pins 50 and 52 double as SPI MISO/SCK — using them for motors blocks
// future SPI peripherals on this Mega. Servo lib doesn't care which digital
// pin; the choice here matches the user's physical wiring.
#define MOTOR_M1_PIN            52       // front-right (CCW)
#define MOTOR_M2_PIN            50       // rear-right  (CW)
#define MOTOR_M3_PIN            48       // rear-left   (CCW)
#define MOTOR_M4_PIN            46       // front-left  (CW)

// Pulse widths sent to ESCs.
#define MOTOR_DISARM_US         1000     // ESCs MUST see this at boot to arm
#define MOTOR_IDLE_US           1080     // armed but barely spinning
#define MOTOR_MIN_US            1100     // minimum commandable in-flight throttle
#define MOTOR_MAX_US            1900     // capped below 2000 to leave PID headroom

// SparkFun ESC needs THROTTLE_LOW for ~1 s after powering up to recognize the
// signal; we send MOTOR_DISARM_US continuously for this duration before the
// loop allows armed throttle.
#define ESC_BOOT_HOLD_MS        2500


// ---- Stick mapping (post-calibration) --------------------------------------
// Centered sticks read 1500 ± 400 µs (after firmware center-offset). Throttle
// stick range is 1100..1900 µs. Anything outside saturates.
#define STICK_DEAD_BAND_US      15        // ±15 µs around center -> zero input
#define STICK_RANGE_HALF_US     400       // ±this from 1500 = full deflection
#define STICK_THROTTLE_LO_US    1100
#define STICK_THROTTLE_HI_US    1900

// Maximum commanded angle (angle mode) and rate (acro / yaw).
#define MAX_TILT_DEG            25.0f     // ±25° banking from full stick
#define MAX_YAW_RATE_DPS        120.0f    // ±120 deg/s yaw at full rudder


// ---- PID gains (initial guess for F450 / 1400 KV / 3S) ---------------------
// Tuning starts with P only, then add D, then I. Edit these and re-flash.
//
// Outer angle loop: angle error (deg) -> rate setpoint (deg/s).
//   3.0 deg/s per deg of tilt error feels gentle; raise for snappier leveling.
#define PID_ANGLE_KP            3.0f

// Inner roll/pitch rate loop (gyro feedback). Output in PWM µs.
#define PID_ROLL_RATE_KP        0.50f
#define PID_ROLL_RATE_KI        0.30f
#define PID_ROLL_RATE_KD        0.005f
#define PID_PITCH_RATE_KP       PID_ROLL_RATE_KP   // F450 is symmetric on X
#define PID_PITCH_RATE_KI       PID_ROLL_RATE_KI
#define PID_PITCH_RATE_KD       PID_ROLL_RATE_KD

// Yaw rate loop. More P, less D — yaw axis has much higher inertia.
#define PID_YAW_RATE_KP         1.50f
#define PID_YAW_RATE_KI         0.50f
#define PID_YAW_RATE_KD         0.0f

// Anti-windup and saturation (in PWM µs).
#define PID_I_LIMIT_US          100.0f    // |integral term| <= this
#define PID_OUTPUT_LIMIT_US     300.0f    // |PID output|    <= this per axis


// ---- Arming / safety -------------------------------------------------------
// Arm gesture: throttle low + yaw stick held to a pre-configured side for N ms.
// Disarm: throttle low + opposite yaw stick.
#define ARM_THROTTLE_MAX_US     1080      // throttle must be ≤ this to arm
#define ARM_YAW_LOW_US          1150      // yaw stick "held left"
#define ARM_YAW_HIGH_US         1850      // yaw stick "held right"
#define ARM_HOLD_MS             1500      // gesture hold time

// Failsafe: if any flight-critical channel goes silent for this long, disarm.
#define FAILSAFE_MS             250
