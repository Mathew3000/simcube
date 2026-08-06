#include "Lsm6dsox.h"

#include <Wire.h>

namespace {

constexpr uint8_t kWhoAmI = 0x0F;
constexpr uint8_t kCtrl1Xl = 0x10;
constexpr uint8_t kCtrl2G = 0x11;
constexpr uint8_t kCtrl3C = 0x12;
constexpr uint8_t kStatus = 0x1E;
constexpr uint8_t kOutxLG = 0x22;

constexpr uint8_t kWhoAmIValue = 0x6C;  // LSM6DSOX. The DSO/DSR variants answer differently.

// ODR 208 Hz is 0b0101 in the high nibble of both control registers.
constexpr uint8_t kOdr208 = 0x50;
// FS_XL: 00 = +-2g, 01 = +-16g, 10 = +-4g, 11 = +-8g. Note that the encoding is NOT in order.
constexpr uint8_t kFs8g = 0x3 << 2;
// FS_G: 00 = 250, 01 = 500, 10 = 1000, 11 = 2000 dps.
constexpr uint8_t kFs500dps = 0x1 << 2;
// BDU: latch the low/high halves of a sample together, so a read cannot straddle two samples.
// IF_INC: auto-increment the register pointer, which is what makes the 12-byte burst work.
constexpr uint8_t kBdu = 1 << 6;
constexpr uint8_t kIfInc = 1 << 2;

}  // namespace

bool Lsm6dsox::write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(kAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool Lsm6dsox::read(uint8_t reg, uint8_t* dst, size_t n) {
  Wire.beginTransmission(kAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;  // repeated start, no stop
  if (Wire.requestFrom((int)kAddr, (int)n) != (int)n) return false;
  for (size_t i = 0; i < n; ++i) dst[i] = (uint8_t)Wire.read();
  return true;
}

bool Lsm6dsox::begin(int sdaPin, int sclPin, uint32_t hz) {
  present_ = false;
  Wire.begin(sdaPin, sclPin, hz);

  if (!read(kWhoAmI, &who_, 1)) return false;
  if (who_ != kWhoAmIValue) return false;

  if (!write8(kCtrl3C, kBdu | kIfInc)) return false;
  if (!write8(kCtrl1Xl, kOdr208 | kFs8g)) return false;
  if (!write8(kCtrl2G, kOdr208 | kFs500dps)) return false;

  present_ = true;
  return true;
}

bool Lsm6dsox::ready() {
  uint8_t s = 0;
  if (!read(kStatus, &s, 1)) return false;
  return (s & 0x3) == 0x3;  // XLDA and GDA both set
}

bool Lsm6dsox::read(Raw& out) {
  uint8_t b[12];
  if (!read(kOutxLG, b, sizeof(b))) return false;
  auto le16 = [](const uint8_t* p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); };
  out.gx = le16(b + 0);
  out.gy = le16(b + 2);
  out.gz = le16(b + 4);
  out.ax = le16(b + 6);
  out.ay = le16(b + 8);
  out.az = le16(b + 10);
  return true;
}
