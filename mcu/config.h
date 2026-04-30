#pragma once

// =============================================================================
// flight-controller-mcu  --  central configuration
// -----------------------------------------------------------------------------
// All hardware-specific values live here. Changing target boards (Mega -> Teensy
// -> custom PCB) should mostly mean swapping this file, not the logic modules.
// Group constants by subsystem; keep comments short but explanatory.
// =============================================================================


// ---- Serial telemetry -------------------------------------------------------
#define TELEMETRY_BAUD          115200UL
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

// Pin assignments: external interrupts INT2..INT5 on Mega 2560.
//   INT2 = pin 19 (PD2)
//   INT3 = pin 18 (PD3)
//   INT4 = pin  2 (PE4)
//   INT5 = pin  3 (PE5)
// CH5/CH6 stay on Port K PCINT (A12/A13) so the existing wiring still works
// for any future switch channels.
#define RX_CH1_PIN              19
#define RX_CH2_PIN              18
#define RX_CH3_PIN              2
#define RX_CH4_PIN              3
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
