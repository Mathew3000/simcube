#pragma once
#include <cstddef>
#include <cstdint>

// Minimal LSM6DSOX driver: raw register reads over I2C, no floating point anywhere.
//
// Deliberately not Adafruit_LSM6DS. Two reasons, and the second is the real one:
//
//  * The dependency chain (Adafruit_LSM6DS + BusIO + Unified Sensor) is more code than this
//    file, to expose a sensors_event_t we immediately take apart again.
//
//  * getEvent() returns floats, and this runs in the task pinned to core 0. FreeRTOS on Xtensa
//    saves FPU context lazily, so keeping one core integer-only avoids paying for FPU context
//    switches between the sensor task and anything else scheduled there. The conversion to
//    g and rad/s happens on core 1, next to the filter that consumes it.
//
// The scale factors are therefore explicit here rather than buried in a vendor driver, which
// also means the +-8g range the plan calls for is visible and checkable: a hand-shaken cube
// clips a +-2g part, and it clips exactly during the interaction that matters.
class Lsm6dsox {
 public:
  struct Raw {
    int16_t gx, gy, gz;  // gyro, LSB
    int16_t ax, ay, az;  // accel, LSB
  };

  // Configures 208 Hz on both sensors at +-8 g / +-500 dps. Returns false if WHO_AM_I is wrong,
  // which is the one failure worth distinguishing: it means the wiring or the address, not the
  // configuration.
  bool begin(int sdaPin, int sclPin, uint32_t hz);

  // True when a fresh sample of both accel and gyro is waiting.
  bool ready();
  // One 12-byte burst from OUTX_L_G: gyro then accel, in one transaction.
  bool read(Raw& out);

  uint8_t whoAmI() const { return who_; }
  bool present() const { return present_; }

  // LSB -> physical units. Constants from the datasheet for the ranges configured above.
  //   accel: +-8 g   -> 0.244 mg/LSB
  //   gyro:  +-500 dps -> 17.50 mdps/LSB, converted straight to rad/s
  static constexpr float kAccelScaleG = 0.000244f;
  static constexpr float kGyroScaleRad = 0.0175f * 0.017453292f;

 private:
  bool write8(uint8_t reg, uint8_t val);
  bool read(uint8_t reg, uint8_t* dst, size_t n);

  static constexpr uint8_t kAddr = 0x6A;  // SDO/SA0 low; 0x6B if the pin is pulled high
  uint8_t who_ = 0;
  bool present_ = false;
};
