// Flight controller — main sketch.
//
// Pipeline per FC tick (1 / FC_LOOP_HZ s):
//
//   IMU read --> attitude estimate (complementary filter)
//                       |
//                       v
//   RX (interrupt-driven, applied per-channel offset, failsafe check)
//                       |
//                       v
//   control: stick -> Setpoints (angle, yaw rate, throttle, armed)
//                       |
//                       v
//   if armed:
//     angle PID (outer):   tilt error  -> rate setpoint
//     rate PID  (inner):   rate error  -> PWM correction (us)
//     mixer:               throttle + roll/pitch/yaw -> 4 motor pulses
//     motors:              writeMicroseconds to ESCs
//   else:
//     motorsDisarm()
//
// Telemetry CSV stream stays unchanged so the visualizer keeps working.

#include <Wire.h>
#include "src/include/config.h"
#include "src/include/rx.h"
#include "src/include/pid.h"
#include "src/include/mixer.h"
#include "src/include/motors.h"
#include "src/include/control.h"
#include "src/include/leds.h"
#include "src/include/battery.h"

// MPU6050 register addresses (datasheet "Register Map", §3).
static const uint8_t REG_PWR_MGMT = 0x6B;
static const uint8_t REG_SMPLRT   = 0x19;
static const uint8_t REG_CONFIG   = 0x1A;
static const uint8_t REG_GYRO_CFG = 0x1B;
static const uint8_t REG_ACC_CFG  = 0x1C;
static const uint8_t REG_ACC_XOUT = 0x3B;

// IMU calibration offsets (raw LSB).
float ax_off = 0, ay_off = 0, az_off = 0;
float gx_off = 0, gy_off = 0, gz_off = 0;

// Attitude state.
float roll_filt = 0, pitch_filt = 0, yaw = 0;
uint32_t last_us = 0;
uint32_t last_stream_ms = 0;
uint32_t last_fc_us     = 0;

// Last-computed motor commands and arming state, updated by the FC tick and
// read by the telemetry stream so the visualizer can render them.
uint16_t motor_out[4]   = {MOTOR_DISARM_US, MOTOR_DISARM_US,
                           MOTOR_DISARM_US, MOTOR_DISARM_US};
bool     fc_armed       = false;

// RX center-calibration state. The telemetry EMA (rx_ema) smooths the CSV
// stream for the visualizer; the FC EMA (rx_fc_ema) is a lighter filter on
// values feeding the PIDs so motor commands aren't twitchy at sub-microsecond
// jitter without adding meaningful latency.
static const bool RX_IS_CENTERED[RX_NUM_CHANNELS] = RX_IS_CENTERED_INIT;
float    rx_ema[RX_NUM_CHANNELS]    = {0};
float    rx_fc_ema[RX_NUM_CHANNELS] = {0};
int16_t  rx_offset[RX_NUM_CHANNELS] = {0};

// Battery / temperature monitor cadence.
uint32_t last_battery_ms = 0;
float    last_temp_c     = 0.0f;

// Flight controllers: one outer angle loop per axis (P-only), one inner rate
// loop per axis (full PID). Yaw has no outer loop — yaw is rate-controlled.
Pid pid_roll_angle (PID_ANGLE_KP, 0, 0, 0, MAX_YAW_RATE_DPS);
Pid pid_pitch_angle(PID_ANGLE_KP, 0, 0, 0, MAX_YAW_RATE_DPS);
Pid pid_roll_rate (PID_ROLL_RATE_KP,  PID_ROLL_RATE_KI,  PID_ROLL_RATE_KD,
                   PID_I_LIMIT_US, PID_OUTPUT_LIMIT_US);
Pid pid_pitch_rate(PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD,
                   PID_I_LIMIT_US, PID_OUTPUT_LIMIT_US);
Pid pid_yaw_rate  (PID_YAW_RATE_KP,   PID_YAW_RATE_KI,   PID_YAW_RATE_KD,
                   PID_I_LIMIT_US, PID_OUTPUT_LIMIT_US);

// ---- MPU6050 helpers --------------------------------------------------------
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz, int16_t &temp) {
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(REG_ACC_XOUT);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_I2C_ADDR, 14, (int)true) != 14) return false;
  ax   = (Wire.read() << 8) | Wire.read();
  ay   = (Wire.read() << 8) | Wire.read();
  az   = (Wire.read() << 8) | Wire.read();
  temp = (Wire.read() << 8) | Wire.read();
  gx   = (Wire.read() << 8) | Wire.read();
  gy   = (Wire.read() << 8) | Wire.read();
  gz   = (Wire.read() << 8) | Wire.read();
  return true;
}

void calibrate() {
  Serial.println(F("# IMU calibrate - keep still and level"));
  double sax=0, say=0, saz=0, sgx=0, sgy=0, sgz=0;
  uint16_t got = 0;
  while (got < MPU_CALIB_SAMPLES) {
    int16_t ax, ay, az, gx, gy, gz, temp;
    if (!mpuReadRaw(ax, ay, az, gx, gy, gz, temp)) continue;
    sax += ax; say += ay; saz += az;
    sgx += gx; sgy += gy; sgz += gz;
    got++;
    ledsUpdate(millis());   // keep CALIB led blinking
    delay(2);
  }
  ax_off = sax / MPU_CALIB_SAMPLES;
  ay_off = say / MPU_CALIB_SAMPLES;
  az_off = (saz / MPU_CALIB_SAMPLES) - MPU_ACCEL_LSB_PER_G;
  gx_off = sgx / MPU_CALIB_SAMPLES;
  gy_off = sgy / MPU_CALIB_SAMPLES;
  gz_off = sgz / MPU_CALIB_SAMPLES;
}

void calibrateRx() {
  Serial.println(F("# RX calibrate - keep all sticks centered"));
  uint32_t deadline = millis() + 1000;
  while (millis() < deadline) {
    bool ready = true;
    for (uint8_t i = 0; i < RX_NUM_CHANNELS; i++) {
      if (RX_IS_CENTERED[i] && rxGet(i) == 0) { ready = false; break; }
    }
    if (ready) break;
    ledsUpdate(millis());
    delay(10);
  }
  uint32_t sums[RX_NUM_CHANNELS]   = {0};
  uint16_t counts[RX_NUM_CHANNELS] = {0};
  for (uint16_t s = 0; s < RX_CALIB_SAMPLES; s++) {
    for (uint8_t i = 0; i < RX_NUM_CHANNELS; i++) {
      uint16_t v = rxGet(i);
      if (v > 0) { sums[i] += v; counts[i]++; }
    }
    ledsUpdate(millis());
    delay(5);
  }
  for (uint8_t i = 0; i < RX_NUM_CHANNELS; i++) {
    if (RX_IS_CENTERED[i] && counts[i] > 50) {
      uint16_t mean = (uint16_t)(sums[i] / counts[i]);
      rx_offset[i]  = (int16_t)mean - 1500;
      rx_ema[i]     = mean;
    }
  }
}

// Tick callback for motorsArmEscs() — keeps LED patterns advancing during
// the ESC startup hold so the user sees the BLINK pattern immediately.
static void _setupTick() { ledsUpdate(millis()); }

// Apply firmware center offset + light EMA smoothing to a raw RX value,
// clamped to [1000, 2000]. The FC pipeline calls this every loop.
static uint16_t rxCorrected(uint8_t ch) {
  uint16_t raw = rxGet(ch);
  if (raw == 0) return 0;
  int32_t v = (int32_t)raw - rx_offset[ch];
  if (v < 1000) v = 1000;
  if (v > 2000) v = 2000;
  if (rx_fc_ema[ch] == 0.0f) rx_fc_ema[ch] = (float)v;
  else rx_fc_ema[ch] = RX_FC_EMA_ALPHA * (float)v
                     + (1.0f - RX_FC_EMA_ALPHA) * rx_fc_ema[ch];
  return (uint16_t)rx_fc_ema[ch];
}

// ---- setup / loop -----------------------------------------------------------
void setup() {
  Serial.begin(TELEMETRY_BAUD);
  Wire.begin();
  Wire.setClock(MPU_I2C_CLOCK);

  mpuWrite(REG_PWR_MGMT, 0x00);                    // wake
  delay(50);
  mpuWrite(REG_SMPLRT,   MPU_SAMPLE_RATE_DIVIDER);
  mpuWrite(REG_CONFIG,   MPU_DLPF_CONFIG);
  mpuWrite(REG_GYRO_CFG, MPU_GYRO_FS << 3);
  mpuWrite(REG_ACC_CFG,  MPU_ACCEL_FS << 3);
  delay(50);

  rxInit();
  motorsInit();
  controlInit();
  ledsInit();
  batteryInit();

  // STARTUP led blinks during boot to show the chip is alive even before
  // any of the slower init steps complete.
  ledStartupSet(LED_MODE_BLINK);

  // ESC + IMU + RX calibration phase: CALIB led blinks until everything
  // is settled. ESCs run their startup beeps during motorsArmEscs.
  // CALIB stays in BLINK after calibration completes — we only switch to
  // ON once the user actually arms via the TX gesture.
  ledCalibSet(LED_MODE_BLINK);
  motorsArmEscs(_setupTick);
  calibrate();
  calibrateRx();

  // Boot complete (CALIB still blinking, waiting for arm).
  ledStartupSet(LED_MODE_ON);
  last_us    = micros();
  last_fc_us = last_us;
  // Telemetry CSV: only fields the visualiser actually renders, plus a few
  // diagnostics. Raw accel/gyro stay in the IMU loop where they're needed
  // (PIDs) and don't hit the serial wire.
  Serial.println(F("roll,pitch,yaw,temp_c,"
                   "ch1,ch2,ch3,ch4,ch5,ch6,armed,m1,m2,m3,m4,t_ms"));
}

// Consecutive IMU read failures. If the I2C bus glitches or the MPU is
// physically disconnected we'll see this climb; flash the CALIB led when
// it does. Reset on every successful read.
static uint16_t imu_fail_streak = 0;
static const uint16_t IMU_FAIL_FLASH_THRESHOLD = 50;  // ~250 ms of failures

void loop() {
  int16_t rax, ray, raz, rgx, rgy, rgz, rtemp;
  if (!mpuReadRaw(rax, ray, raz, rgx, rgy, rgz, rtemp)) {
    if (imu_fail_streak < 0xFFFF) imu_fail_streak++;
    if (imu_fail_streak >= IMU_FAIL_FLASH_THRESHOLD) ledCalibSet(LED_MODE_FLASH_2);
    ledsUpdate(millis());
    return;
  }
  if (imu_fail_streak >= IMU_FAIL_FLASH_THRESHOLD) {
    // Recovered — return CALIB to its steady "calibration done" indicator.
    ledCalibSet(LED_MODE_ON);
  }
  imu_fail_streak = 0;

  float ax = (rax - ax_off) / MPU_ACCEL_LSB_PER_G;
  float ay = (ray - ay_off) / MPU_ACCEL_LSB_PER_G;
  float az = (raz - az_off) / MPU_ACCEL_LSB_PER_G;
  float gx = (rgx - gx_off) / MPU_GYRO_LSB_PER_DPS;
  float gy = (rgy - gy_off) / MPU_GYRO_LSB_PER_DPS;
  float gz = (rgz - gz_off) / MPU_GYRO_LSB_PER_DPS;
  float temp_c = rtemp / 340.0f + 36.53f;

  uint32_t now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;
  last_us = now_us;

  float accel_roll  = atan2f(ay, az) * 57.29578f;
  float accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.29578f;
  roll_filt  = COMP_FILTER_ALPHA * (roll_filt  + gx * dt) + (1 - COMP_FILTER_ALPHA) * accel_roll;
  pitch_filt = COMP_FILTER_ALPHA * (pitch_filt + gy * dt) + (1 - COMP_FILTER_ALPHA) * accel_pitch;
  yaw += gz * dt;

  // Sanity guard: a single bad I2C read or weird transient can poison the
  // comp filter with NaN, after which the IIR feedback keeps it stuck
  // forever (NaN propagates through every multiply/add). Reset to zero on
  // detection — better to lose a tick than ride a corrupted filter into
  // the PID + mixer.
  if (isnan(roll_filt))  roll_filt  = 0.0f;
  if (isnan(pitch_filt)) pitch_filt = 0.0f;
  if (isnan(yaw))        yaw        = 0.0f;

  float roll  = roll_filt;
  float pitch = pitch_filt;

  uint32_t now_ms = millis();

  // ---- Periodic monitoring (battery + chip temperature) -------------------
  // TEMP led: solid on while chip is hot, off otherwise.
  ledTempSet(temp_c >= MPU_TEMP_HIGH_C ? LED_MODE_ON : LED_MODE_OFF);
  // BATTERY led: blinks while voltage is low (more eye-catching than solid).
  if (now_ms - last_battery_ms >= 1000UL / BATTERY_CHECK_HZ) {
    last_battery_ms = now_ms;
    batteryReadVolts();
    ledBatterySet(batteryIsLow() ? LED_MODE_BLINK : LED_MODE_OFF);
  }
  last_temp_c = temp_c;
  ledsUpdate(now_ms);

  // ---- Flight controller tick ---------------------------------------------
  if (now_us - last_fc_us >= 1000000UL / FC_LOOP_HZ) {
    last_fc_us = now_us;

    bool failsafe = !rxAlive(0, now_us) || !rxAlive(1, now_us) ||
                    !rxAlive(2, now_us) || !rxAlive(3, now_us);

    // STARTUP led flashes on failsafe (RX silent on a flight-critical
    // channel) — the system is alive but can't be flown safely.
    ledStartupSet(failsafe ? LED_MODE_FLASH_2 : LED_MODE_ON);

    Setpoints sp = controlUpdate(rxCorrected(0), rxCorrected(1),
                                 rxCorrected(2), rxCorrected(3),
                                 failsafe, now_ms);

    // CALIB led tracks the armed state: solid when armed (ready to fly),
    // blinking when disarmed (calibrated but waiting for arm gesture). The
    // IMU-fault FLASH_2 in the IMU read path overrides this if it triggers
    // — we only set ON/BLINK here, never override the FLASH_2.
    if (imu_fail_streak < IMU_FAIL_FLASH_THRESHOLD) {
      ledCalibSet(sp.armed ? LED_MODE_ON : LED_MODE_BLINK);
    }

    // Wipe PID state every tick while disarmed so integrators can't wind up
    // from IMU drift / mounting-bias errors. This makes the displayed motor
    // commands a pure P-response in disarmed mode (instantaneous, repeatable),
    // which is what we want for sign-verification and tuning visualization.
    // Once armed, the integrators behave normally until the next disarm.
    if (!sp.armed) {
      pid_roll_angle.reset(); pid_pitch_angle.reset();
      pid_roll_rate.reset();  pid_pitch_rate.reset();  pid_yaw_rate.reset();
    }

    // Always run the PID + mixer so the simulator can show "what the FC
    // would command right now" even when disarmed. Only motorsWrite when
    // actually armed.
    float roll_rate_sp  = pid_roll_angle.update (sp.angle_roll_deg,  roll,  FC_DT_S);
    float pitch_rate_sp = pid_pitch_angle.update(sp.angle_pitch_deg, pitch, FC_DT_S);
    float roll_us       = pid_roll_rate.update  (roll_rate_sp,       gx,    FC_DT_S);
    float pitch_us      = pid_pitch_rate.update (pitch_rate_sp,      gy,    FC_DT_S);
    float yaw_us        = pid_yaw_rate.update   (sp.yaw_rate_dps,    gz,    FC_DT_S);
    mixerComputeXQuad(sp.throttle_us, roll_us, pitch_us, yaw_us, motor_out);

    fc_armed = sp.armed;
    if (fc_armed) motorsWrite(motor_out);
    else          motorsDisarm();
  }

  // ---- Telemetry stream — slim 16-field CSV ---------------------------
  // Trimmed to keep TX time per line well under one IMU loop tick so the
  // visualizer never sees stale data.
  if (now_ms - last_stream_ms >= 1000UL / TELEMETRY_HZ) {
    last_stream_ms = now_ms;
    Serial.print(roll, 1);   Serial.print(',');
    Serial.print(pitch, 1);  Serial.print(',');
    Serial.print(yaw, 1);    Serial.print(',');
    Serial.print(temp_c, 1); Serial.print(',');
    for (uint8_t ch = 0; ch < RX_NUM_CHANNELS; ch++) {
      uint16_t raw = rxGet(ch);
      if (raw > 0) {
        rx_ema[ch] = (rx_ema[ch] == 0)
            ? raw
            : RX_EMA_ALPHA * raw + (1.0f - RX_EMA_ALPHA) * rx_ema[ch];
      }
      int16_t out = 0;
      if (rx_ema[ch] > 0) {
        out = (int16_t)rx_ema[ch] - rx_offset[ch];
        if (out < 1000) out = 1000;
        if (out > 2000) out = 2000;
      }
      Serial.print(out);
      Serial.print(',');
    }
    Serial.print(fc_armed ? 1 : 0); Serial.print(',');
    Serial.print(motor_out[0]); Serial.print(',');
    Serial.print(motor_out[1]); Serial.print(',');
    Serial.print(motor_out[2]); Serial.print(',');
    Serial.print(motor_out[3]); Serial.print(',');
    Serial.println(now_ms);
  }
}
