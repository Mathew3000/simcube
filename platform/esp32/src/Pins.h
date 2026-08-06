#pragma once

// Pin assignment for the ESP32-S3-WROOM-1 N16R8.
//
// These are PLACEHOLDERS in one specific sense: they are all electrically valid on this module,
// but which pin you actually solder to which HUB75 signal is a fact about the built object.
// Check them against the wiring before the first power-up; a mis-assigned CLK or LAT produces a
// panel full of noise, which is at least obvious.
//
// What is NOT negotiable is the exclusion list. On the N16R8 the following are unavailable or
// unwise:
//
//   IO35, IO36, IO37  -- consumed permanently by the octal PSRAM. Using them looks like it works
//                        until the first PSRAM access, then corrupts memory.
//   IO26..IO32        -- SPI flash.
//   IO19, IO20        -- native USB (the serial console arrives over these on a devkit).
//   IO43, IO44        -- UART0 TX/RX.
//   IO0, IO3, IO45,
//   IO46              -- strapping pins; driven at boot, and IO46 is input-only.
//
// That leaves IO1..IO18, IO21, IO38..IO42, IO47, IO48. Everything below is inside IO4..IO18,
// which keeps a clear margin and leaves the higher pins free for a level shifter or a fan.
namespace pins {

// --- HUB75 ------------------------------------------------------------------------------------
// 32-row panels are 1/16 scan, so there are four address lines and no E. Setting E to -1 is how
// the driver is told that; a 64-row panel would need it wired.
constexpr int kR1 = 4;
constexpr int kG1 = 5;
constexpr int kB1 = 6;
constexpr int kR2 = 7;
constexpr int kG2 = 15;
constexpr int kB2 = 16;
constexpr int kA = 17;
constexpr int kB = 18;
constexpr int kC = 8;
constexpr int kD = 9;
constexpr int kE = -1;  // 1/16 scan: unused
constexpr int kLat = 10;
constexpr int kOe = 11;
constexpr int kClk = 12;

// --- IMU (I2C) --------------------------------------------------------------------------------
// Kept away from the panel bus deliberately: ~1.4m of unterminated ribbon switching at 16MHz is
// a fine antenna, and an I2C line routed alongside it will see the clock.
constexpr int kSda = 13;
constexpr int kScl = 14;
constexpr int kI2cHz = 400000;

}  // namespace pins
