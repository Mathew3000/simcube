#pragma once

// Pin assignment for the plain ESP32 driving the WS2812B chain.
//
// kData confirmed by the user against the built object, not a placeholder like the S3's Pins.h
// entries are: GPIO2, which doubles as the devkit's onboard LED pin on most plain-ESP32 boards --
// the reason this specific pin was already carrying a data line. It is also a boot-mode strapping
// pin (must be low or floating at reset for normal SPI-flash boot); this has not caused a boot
// problem here, and WS2812 data idles low, but it is worth knowing if a future board revision
// ever fails to boot with the strip attached.
//
// There is no IMU wired yet (MINI.md section 2: "IMU: none yet"), so there are no I2C pins here.
// Adding them now would be for a part that does not exist; see main.cpp's motion section for
// where they belong once it does.
namespace pins {

constexpr int kData = 2;  // WS2812B DIN

}  // namespace pins
