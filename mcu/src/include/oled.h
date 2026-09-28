#pragma once

#include <Arduino.h>

// Minimal SSD1306 128x64 I2C status display. Boot screens are sent while
// motors are disarmed; live screens are transferred in small chunks after
// flight-control calculations complete.
enum OledBootStage : uint8_t {
  OLED_BOOT_START = 0,
  OLED_BOOT_ESC,
  OLED_BOOT_IMU,
  OLED_BOOT_RX,
  OLED_BOOT_READY,
};

bool oledInit();
bool oledPresent();
void oledShowBoot(OledBootStage stage);
void oledQueueLive(bool armed, bool failsafe, bool battery_low,
                   float battery_v, const uint16_t rx_us[6],
                   float gx_dps, float gy_dps, float gz_dps,
                   float temp_c);
void oledService();
