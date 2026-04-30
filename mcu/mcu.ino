// MPU6050 raw-I2C driver for Arduino Mega.
// Wiring: VCC->5V, GND->GND, SDA->20, SCL->21, AD0->GND.
// Streams CSV at 115200:
//   ax,ay,az,gx,gy,gz,roll,pitch,yaw,temp_c,ch1..ch6,t_ms
// Channels are FS-R6B PWM widths in µs (1000..2000); 0 means no signal.

#include <Wire.h>
#include "rx.h"

static const uint8_t  MPU_ADDR     = 0x68;
static const uint8_t  REG_PWR_MGMT = 0x6B;
static const uint8_t  REG_SMPLRT   = 0x19;
static const uint8_t  REG_CONFIG   = 0x1A;
static const uint8_t  REG_GYRO_CFG = 0x1B;
static const uint8_t  REG_ACC_CFG  = 0x1C;
static const uint8_t  REG_ACC_XOUT = 0x3B;

// ±2g, ±250 dps → factory sensitivity scales.
static const float ACC_LSB_PER_G   = 16384.0f;
static const float GYRO_LSB_PER_DPS = 131.0f;

static const uint16_t CALIB_SAMPLES = 2000;
static const uint32_t STREAM_HZ     = 100;

float ax_off = 0, ay_off = 0, az_off = 0;
float gx_off = 0, gy_off = 0, gz_off = 0;

float roll_filt = 0, pitch_filt = 0, yaw = 0;
static const float COMP_ALPHA = 0.98f;  // gyro weight; (1-alpha) is accel weight
uint32_t last_us = 0;
uint32_t last_stream_ms = 0;

// RX smoothing + center calibration. Centered channels (CH1/CH2/CH4) get a
// per-channel offset measured at boot so an at-rest stick reads exactly 1500.
// Throttle (CH3) and switches (CH5/CH6) are not centered, so offset stays 0.
static const float    RX_ALPHA          = 0.20f;  // EMA: lower = smoother
static const uint16_t RX_CALIB_SAMPLES  = 400;
static const bool     RX_IS_CENTERED[6] = { true, true, false, true, false, false };
float    rx_ema[6]    = {0};
int16_t  rx_offset[6] = {0};

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz, int16_t &temp) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACC_XOUT);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14, (int)true) != 14) return false;

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
  while (got < CALIB_SAMPLES) {
    int16_t ax, ay, az, gx, gy, gz, temp;
    if (!mpuReadRaw(ax, ay, az, gx, gy, gz, temp)) continue;
    sax += ax; say += ay; saz += az;
    sgx += gx; sgy += gy; sgz += gz;
    got++;
    delay(2);
  }
  ax_off = sax / CALIB_SAMPLES;
  ay_off = say / CALIB_SAMPLES;
  az_off = (saz / CALIB_SAMPLES) - ACC_LSB_PER_G;  // gravity on +Z
  gx_off = sgx / CALIB_SAMPLES;
  gy_off = sgy / CALIB_SAMPLES;
  gz_off = sgz / CALIB_SAMPLES;
  Serial.print(F("# offsets accel ")); Serial.print(ax_off); Serial.print(',');
  Serial.print(ay_off); Serial.print(','); Serial.print(az_off);
  Serial.print(F(" gyro "));            Serial.print(gx_off); Serial.print(',');
  Serial.print(gy_off); Serial.print(','); Serial.println(gz_off);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  mpuWrite(REG_PWR_MGMT, 0x00);   // wake
  delay(50);
  mpuWrite(REG_SMPLRT,   0x07);   // 1kHz / (1+7) = 125 Hz
  mpuWrite(REG_CONFIG,   0x03);   // DLPF ~44Hz
  mpuWrite(REG_GYRO_CFG, 0x00);   // ±250 dps
  mpuWrite(REG_ACC_CFG,  0x00);   // ±2 g
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

  float ax = (rax - ax_off) / ACC_LSB_PER_G;
  float ay = (ray - ay_off) / ACC_LSB_PER_G;
  float az = (raz - az_off) / ACC_LSB_PER_G;
  float gx = (rgx - gx_off) / GYRO_LSB_PER_DPS;
  float gy = (rgy - gy_off) / GYRO_LSB_PER_DPS;
  float gz = (rgz - gz_off) / GYRO_LSB_PER_DPS;
  float temp_c = rtemp / 340.0f + 36.53f;  // datasheet §4.18

  uint32_t now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;
  last_us = now_us;

  float accel_roll  = atan2f(ay, az) * 57.29578f;
  float accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.29578f;
  roll_filt  = COMP_ALPHA * (roll_filt  + gx * dt) + (1 - COMP_ALPHA) * accel_roll;
  pitch_filt = COMP_ALPHA * (pitch_filt + gy * dt) + (1 - COMP_ALPHA) * accel_pitch;
  yaw += gz * dt;  // gyro-only — will drift; mag fusion comes later
  float roll  = roll_filt;
  float pitch = pitch_filt;

  uint32_t now_ms = millis();
  if (now_ms - last_stream_ms >= 1000UL / STREAM_HZ) {
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
            : RX_ALPHA * raw + (1.0f - RX_ALPHA) * rx_ema[ch];
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
