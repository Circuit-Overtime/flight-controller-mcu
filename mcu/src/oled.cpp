#include <Arduino.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <string.h>

#include "include/config.h"
#include "include/oled.h"

static uint8_t framebuffer[OLED_WIDTH * OLED_PAGES];
static bool available = false;
static uint8_t active_address = 0;
static bool dirty = false;
static uint8_t tx_page = 0;
static uint8_t tx_column = 0;

// Compact 5x7 uppercase font. Lowercase input is converted to uppercase.
static const uint8_t digits[10][5] PROGMEM = {
  {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
  {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
  {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
  {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
  {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}
};

static const uint8_t letters[26][5] PROGMEM = {
  {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36},
  {0x3E,0x41,0x41,0x41,0x22}, {0x7F,0x41,0x41,0x22,0x1C},
  {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
  {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F},
  {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01},
  {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
  {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F},
  {0x3E,0x41,0x41,0x41,0x3E}, {0x7F,0x09,0x09,0x09,0x06},
  {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
  {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01},
  {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F},
  {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63},
  {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}
};

static void glyph(char c, uint8_t out[5]) {
  if (c >= 'a' && c <= 'z') c -= ('a' - 'A');
  if (c >= '0' && c <= '9') {
    for (uint8_t i = 0; i < 5; i++) out[i] = pgm_read_byte(&digits[c-'0'][i]);
    return;
  }
  if (c >= 'A' && c <= 'Z') {
    for (uint8_t i = 0; i < 5; i++) out[i] = pgm_read_byte(&letters[c-'A'][i]);
    return;
  }
  for (uint8_t i = 0; i < 5; i++) out[i] = 0;
  switch (c) {
    case '+': out[0]=0x08; out[1]=0x08; out[2]=0x3E; out[3]=0x08; out[4]=0x08; break;
    case '-': out[0]=0x08; out[1]=0x08; out[2]=0x08; out[3]=0x08; out[4]=0x08; break;
    case '.': out[1]=0x60; out[2]=0x60; break;
    case ':': out[1]=0x36; out[2]=0x36; break;
    case '/': out[0]=0x20; out[1]=0x10; out[2]=0x08; out[3]=0x04; out[4]=0x02; break;
    case '?': out[0]=0x02; out[1]=0x01; out[2]=0x51; out[3]=0x09; out[4]=0x06; break;
    default: break;
  }
}

static void clearFrame() {
  memset(framebuffer, 0, sizeof(framebuffer));
}

static void drawText(uint8_t row, const char *text) {
  if (row >= OLED_PAGES) return;
  uint16_t base = (uint16_t)row * OLED_WIDTH;
  uint8_t x = 0;
  while (*text && x + 5 < OLED_WIDTH) {
    uint8_t columns[5];
    glyph(*text++, columns);
    for (uint8_t i = 0; i < 5; i++) framebuffer[base + x++] = columns[i];
    framebuffer[base + x++] = 0;
  }
}

static bool sendCommands(const uint8_t *commands, uint8_t count) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(active_address);
  Wire.write((uint8_t)0x00);
  for (uint8_t i = 0; i < count; i++) Wire.write(commands[i]);
  uint8_t status = Wire.endTransmission();
  if (status != 0 || Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    available = false;
    return false;
  }
  return true;
}

static bool sendChunk(uint8_t page, uint8_t column, uint8_t count) {
  uint8_t absolute_column = (uint8_t)(OLED_COLUMN_OFFSET + column);
  const uint8_t position[] = {
    (uint8_t)(0xB0 | page),
    (uint8_t)(absolute_column & 0x0F),
    (uint8_t)(0x10 | ((absolute_column >> 4) & 0x0F))
  };
  if (!sendCommands(position, sizeof(position))) return false;

  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(active_address);
  Wire.write((uint8_t)0x40);
  uint16_t offset = (uint16_t)page * OLED_WIDTH + column;
  for (uint8_t i = 0; i < count; i++) Wire.write(framebuffer[offset + i]);
  uint8_t status = Wire.endTransmission();
  if (status != 0 || Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    available = false;
    return false;
  }
  return true;
}

static void flushBlocking() {
  if (!available) return;
  for (uint8_t page = 0; page < OLED_PAGES && available; page++) {
    for (uint8_t column = 0; column < OLED_WIDTH; column += OLED_TX_CHUNK) {
      uint8_t remaining = OLED_WIDTH - column;
      uint8_t count = remaining < OLED_TX_CHUNK ? remaining : OLED_TX_CHUNK;
      if (!sendChunk(page, column, count)) return;
    }
  }
  dirty = false;
  tx_page = 0;
  tx_column = 0;
}

bool oledInit() {
  const uint8_t candidates[] = {OLED_I2C_ADDR, OLED_I2C_ALT_ADDR};
  available = false;
  active_address = 0;
  for (uint8_t i = 0; i < sizeof(candidates); i++) {
    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(candidates[i]);
    bool acknowledged = (Wire.endTransmission() == 0 && !Wire.getWireTimeoutFlag());
    Wire.clearWireTimeoutFlag();
    if (acknowledged) {
      active_address = candidates[i];
      available = true;
      break;
    }
  }
  if (!available) return false;

  const uint8_t init_sequence[] = {
    0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
    0x8D, 0x14, 0x20, 0x02, 0xA1, 0xC8, 0xDA, 0x12,
    0x81, 0x7F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
    0x2E, 0xAF
  };
  if (!sendCommands(init_sequence, sizeof(init_sequence))) return false;
  clearFrame();
  flushBlocking();
  return available;
}

bool oledPresent() {
  return available;
}

uint8_t oledAddress() {
  return active_address;
}

void oledShowBoot(OledBootStage stage) {
  if (!available) return;
  clearFrame();
  char identity[22];
  snprintf(identity, sizeof(identity), "OLED 0X%02X ONLINE", active_address);
  drawText(0, "FLIGHT CONTROLLER");
  drawText(2, identity);
  switch (stage) {
    case OLED_BOOT_START: drawText(4, "BOOT START"); break;
    case OLED_BOOT_ESC:   drawText(4, "ESC INITIALIZE"); break;
    case OLED_BOOT_IMU:   drawText(4, "IMU CALIBRATE"); drawText(6, "KEEP LEVEL STILL"); break;
    case OLED_BOOT_RX:    drawText(4, "RX CALIBRATE");  drawText(6, "CENTER STICKS"); break;
    case OLED_BOOT_READY: drawText(4, "SYSTEM READY");  drawText(6, "DISARMED"); break;
  }
  flushBlocking();
}

static void formatTenths(float value, char *out, size_t out_size) {
  long scaled = (long)(value * 10.0f + (value >= 0.0f ? 0.5f : -0.5f));
  if (scaled > 9999) scaled = 9999;
  if (scaled < -9999) scaled = -9999;
  char sign = scaled < 0 ? '-' : '+';
  unsigned long magnitude = scaled < 0 ? (unsigned long)(-scaled) : (unsigned long)scaled;
  snprintf(out, out_size, "%c%lu.%lu", sign, magnitude / 10UL, magnitude % 10UL);
}

void oledQueueLive(bool armed, bool failsafe, bool battery_low,
                   float battery_v, const uint16_t rx_us[6],
                   float gx_dps, float gy_dps, float gz_dps,
                   float temp_c) {
  if (!available) return;
  char line[22];
  char gx[9], gy[9], gz[9];
  formatTenths(gx_dps, gx, sizeof(gx));
  formatTenths(gy_dps, gy, sizeof(gy));
  formatTenths(gz_dps, gz, sizeof(gz));
  long battery_cV = (long)(battery_v * 100.0f + 0.5f);
  if (battery_cV < 0) battery_cV = 0;
  if (battery_cV > 9999) battery_cV = 9999;
  long temp_tenths = (long)(temp_c * 10.0f + 0.5f);

  clearFrame();
  snprintf(line, sizeof(line), "ARM:%s RX:%s", armed ? "ON" : "OFF", failsafe ? "FAIL" : "OK");
  drawText(0, line);
  snprintf(line, sizeof(line), "R:%4u P:%4u", rx_us[0], rx_us[1]);
  drawText(1, line);
  snprintf(line, sizeof(line), "T:%4u Y:%4u", rx_us[2], rx_us[3]);
  drawText(2, line);
  snprintf(line, sizeof(line), "A5:%4u A6:%4u", rx_us[4], rx_us[5]);
  drawText(3, line);
  snprintf(line, sizeof(line), "BAT:%ld.%02ldV %s", battery_cV / 100L,
           battery_cV % 100L, battery_low ? "LOW" : "OK");
  drawText(4, line);
  drawText(5, "GYRO DPS");
  snprintf(line, sizeof(line), "X:%s Y:%s", gx, gy);
  drawText(6, line);
  snprintf(line, sizeof(line), "Z:%s T:%ld.%ldC", gz,
           temp_tenths / 10L, temp_tenths % 10L);
  drawText(7, line);

  dirty = true;
  tx_page = 0;
  tx_column = 0;
}

void oledService() {
  if (!available || !dirty) return;
  uint8_t remaining = OLED_WIDTH - tx_column;
  uint8_t count = remaining < OLED_TX_CHUNK ? remaining : OLED_TX_CHUNK;
  if (!sendChunk(tx_page, tx_column, count)) return;
  tx_column += count;
  if (tx_column >= OLED_WIDTH) {
    tx_column = 0;
    tx_page++;
    if (tx_page >= OLED_PAGES) {
      tx_page = 0;
      dirty = false;
    }
  }
}
