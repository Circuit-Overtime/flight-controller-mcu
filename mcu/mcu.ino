// MPU6050 raw-I2C driver for Arduino Mega.
// Wiring: VCC->5V, GND->GND, SDA->20, SCL->21, AD0->GND.
// Streams CSV at 115200:
//   ax,ay,az,gx,gy,gz,roll,pitch,yaw,temp_c,ch1..ch6,t_ms
// Channels are FS-R6B PWM widths in µs (1000..2000); 0 means no signal.

#include <Wire.h>
#include "config.h"
#include "rx.h"

// MPU6050 register addresses (see datasheet "Register Map", §3).
static const uint8_t  REG_PWR_MGMT = 0x6B;
static const uint8_t  REG_SMPLRT   = 0x19;
static const uint8_t  REG_CONFIG   = 0x1A;
static const uint8_t  REG_GYRO_CFG = 0x1B;
static const uint8_t  REG_ACC_CFG  = 0x1C;
static const uint8_t  REG_ACC_XOUT = 0x3B;

float ax_off = 0, ay_off = 0, az_off = 0;
float gx_off = 0, gy_off = 0, gz_off = 0;

float roll_filt = 0, pitch_filt = 0, yaw = 0;
uint32_t last_us = 0;
uint32_t last_stream_ms = 0;

// RX smoothing + center calibration state. Tunables live in config.h; the
// arrays below are runtime state seeded at boot in calibrateRx().
static const bool RX_IS_CENTERED[RX_NUM_CHANNELS] = RX_IS_CENTERED_INIT;
float    rx_ema[RX_NUM_CHANNELS]    = {0};
int16_t  rx_offset[RX_NUM_CHANNELS] = {0};

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

void calibrateRx() {
  Serial.println(F("# RX calibrate - keep all sticks centered (throttle anywhere)"));
  // Wait up to 1s for first valid pulses on the centered channels.
  uint32_t deadline = millis() + 1000;
  while (millis() < deadline) {
    bool ready = true;
    for (uint8_t i = 0; i < 6; i++) {
      if (RX_IS_CENTERED[i] && rxGet(i) == 0) { ready = false; break; }
    }
    if (ready) break;
    delay(10);
  }
  uint32_t sums[6]   = {0};
  uint16_t counts[6] = {0};
  for (uint16_t s = 0; s < RX_CALIB_SAMPLES; s++) {
    for (uint8_t i = 0; i < 6; i++) {
      uint16_t v = rxGet(i);
      if (v > 0) { sums[i] += v; counts[i]++; }
    }
    delay(5);  // ~2s total
  }
  for (uint8_t i = 0; i < 6; i++) {
    if (RX_IS_CENTERED[i] && counts[i] > 50) {
      uint16_t mean = (uint16_t)(sums[i] / counts[i]);
      rx_offset[i] = (int16_t)mean - 1500;
      rx_ema[i]    = mean;
    }
  }
  Serial.print(F("# rx_offset "));
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(rx_offset[i]); Serial.print(',');
  }
  Serial.println();
}

void calibrate() {
  Serial.println(F("# calibrating - keep IMU still and level"));
  double sax=0, say=0, saz=0, sgx=0, sgy=0, sgz=0;
  uint16_t got = 0;
  while (got < MPU_CALIB_SAMPLES) {
    int16_t ax, ay, az, gx, gy, gz, temp;
    if (!mpuReadRaw(ax, ay, az, gx, gy, gz, temp)) continue;
    sax += ax; say += ay; saz += az;
    sgx += gx; sgy += gy; sgz += gz;
    got++;
    delay(2);
  }
  ax_off = sax / MPU_CALIB_SAMPLES;
  ay_off = say / MPU_CALIB_SAMPLES;
  az_off = (saz / MPU_CALIB_SAMPLES) - MPU_ACCEL_LSB_PER_G;  // gravity on +Z
  gx_off = sgx / MPU_CALIB_SAMPLES;
  gy_off = sgy / MPU_CALIB_SAMPLES;
  gz_off = sgz / MPU_CALIB_SAMPLES;
  Serial.print(F("# offsets accel ")); Serial.print(ax_off); Serial.print(',');
  Serial.print(ay_off); Serial.print(','); Serial.print(az_off);
  Serial.print(F(" gyro "));            Serial.print(gx_off); Serial.print(',');
  Serial.print(gy_off); Serial.print(','); Serial.println(gz_off);
}

void setup() {
  Serial.begin(TELEMETRY_BAUD);
  Wire.begin();
  Wire.setClock(MPU_I2C_CLOCK);

  mpuWrite(REG_PWR_MGMT, 0x00);                    // wake
  delay(50);
  mpuWrite(REG_SMPLRT,   MPU_SAMPLE_RATE_DIVIDER);
  mpuWrite(REG_CONFIG,   MPU_DLPF_CONFIG);
  mpuWrite(REG_GYRO_CFG, MPU_GYRO_FS << 3);         // FS bits live at [4:3]
  mpuWrite(REG_ACC_CFG,  MPU_ACCEL_FS << 3);
  delay(50);

  rxInit();
  calibrate();
  calibrateRx();
  last_us = micros();
  Serial.println(F("ax,ay,az,gx,gy,gz,roll,pitch,yaw,temp_c,ch1,ch2,ch3,ch4,ch5,ch6,t_ms"));
}

void loop() {
  int16_t rax, ray, raz, rgx, rgy, rgz, rtemp;
  if (!mpuReadRaw(rax, ray, raz, rgx, rgy, rgz, rtemp)) return;

  float ax = (rax - ax_off) / MPU_ACCEL_LSB_PER_G;
  float ay = (ray - ay_off) / MPU_ACCEL_LSB_PER_G;
  float az = (raz - az_off) / MPU_ACCEL_LSB_PER_G;
  float gx = (rgx - gx_off) / MPU_GYRO_LSB_PER_DPS;
  float gy = (rgy - gy_off) / MPU_GYRO_LSB_PER_DPS;
  float gz = (rgz - gz_off) / MPU_GYRO_LSB_PER_DPS;
  float temp_c = rtemp / 340.0f + 36.53f;  // datasheet §4.18

  uint32_t now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;
  last_us = now_us;

  float accel_roll  = atan2f(ay, az) * 57.29578f;
  float accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.29578f;
  roll_filt  = COMP_FILTER_ALPHA * (roll_filt  + gx * dt) + (1 - COMP_FILTER_ALPHA) * accel_roll;
  pitch_filt = COMP_FILTER_ALPHA * (pitch_filt + gy * dt) + (1 - COMP_FILTER_ALPHA) * accel_pitch;
  yaw += gz * dt;  // gyro-only — will drift; mag fusion comes later
  float roll  = roll_filt;
  float pitch = pitch_filt;

  uint32_t now_ms = millis();
  if (now_ms - last_stream_ms >= 1000UL / TELEMETRY_HZ) {
    last_stream_ms = now_ms;
    Serial.print(ax, 4); Serial.print(',');
    Serial.print(ay, 4); Serial.print(',');
    Serial.print(az, 4); Serial.print(',');
    Serial.print(gx, 3); Serial.print(',');
    Serial.print(gy, 3); Serial.print(',');
    Serial.print(gz, 3); Serial.print(',');
    Serial.print(roll, 2);  Serial.print(',');
    Serial.print(pitch, 2); Serial.print(',');
    Serial.print(yaw, 2);   Serial.print(',');
    Serial.print(temp_c, 2); Serial.print(',');
    // EMA-smooth + apply per-channel center offset. rx_ema is 0 until the
    // first valid pulse; once seeded, it never returns to 0 even on edge miss.
    for (uint8_t ch = 0; ch < 6; ch++) {
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
    Serial.println(now_ms);
  }
}
