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
// 32x32 panels are 1/16 scan and ignore E; 64x64 are 1/32 scan and REQUIRE it. Wiring it always
// lets one board design serve both -- an unused E on a 1/16 panel is harmless, while a missing E on
// a 1/32 panel shows the top half of the panel duplicated.
constexpr int kE = 21;
constexpr int kLat = 10;
constexpr int kOe = 11;
constexpr int kClk = 12;

// --- role straps ------------------------------------------------------------------------------
// Two pins, pulled up, jumper to ground. Unstrapped (0b11) is the master; see Role.h for why that
// choice is the diagnosable one.
constexpr int kRoleA = 38;
constexpr int kRoleB = 39;

// --- SPI, master to display nodes ---------------------------------------------------------------
// ONE firmware image serves every role, so a signal present on both board types must use the SAME
// pin on both. SCK/MOSI/MISO therefore avoid the HUB75 block entirely -- an earlier draft of this
// file put SCK on 12 and MOSI on 11, which are CLK and OE, and would have had the SPI link fighting
// the panel bus on the display boards.
//
// Signals that exist on only one board type may reuse pins the other spends elsewhere: the master
// has no HUB75, so its three chip selects sit on 4/5/6, which are RGB lines on a display board.
constexpr int kSpiSck = 13;   // both roles
constexpr int kSpiMosi = 14;  // both roles
constexpr int kSpiMiso = 47;  // both roles
constexpr int kSpiCsIn = 48;  // display: this node's chip select, an input
constexpr int kSpiCsOut[3] = {4, 5, 6};  // master: one per display node, asserted together

// --- IMU (I2C) --------------------------------------------------------------------------------
// Kept away from the panel bus deliberately: ~1.4m of unterminated ribbon switching at 16MHz is
// a fine antenna, and an I2C line routed alongside it will see the clock.
// Master only, so these may reuse pins a display board spends on HUB75 addressing.
constexpr int kSda = 8;
constexpr int kScl = 9;
constexpr int kI2cHz = 400000;

}  // namespace pins
