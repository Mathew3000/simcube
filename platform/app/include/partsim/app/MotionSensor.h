#pragma once
#include <cstdint>

namespace partsim {
namespace app {

// One raw sample of both sensors, in LSB. Integer on purpose: the producing task is pinned to
// core 0 and does no float work at all, because FreeRTOS on Xtensa saves FPU context lazily and
// keeping one core integer-only means the two cores never contend for it. The conversion to g
// and rad/s happens next to the filter that consumes it. See Lsm6dsox.h.
struct ImuSample {
  int16_t gx, gy, gz;  // gyro
  int16_t ax, ay, az;  // accel
};

// A 6-DOF IMU, reduced to what the application actually asks of one.
//
// The scale factors are read from the driver rather than declared here, which matters more than
// it looks: they are a fact about the range the driver configured in its own control registers,
// and a copy of them in the application layer is a second place for a range change to be missed.
// The one number and the register write that justifies it stay in the same file.
class MotionSensor {
 public:
  virtual ~MotionSensor() = default;

  // False when the part did not answer. Not fatal anywhere: a cube with a dead IMU should still
  // be a lamp, with gravity held at the default and the fluid simply sitting at the bottom.
  virtual bool present() const = 0;
  // The WHO_AM_I byte, reported by the `i` command. It distinguishes "wrong wiring or address"
  // from "wrong configuration", which is the one distinction worth making during bring-up.
  virtual uint8_t whoAmI() const = 0;

  // True when a fresh sample of both sensors is waiting.
  virtual bool ready() = 0;
  // One burst read of gyro then accel. False if nothing was read.
  virtual bool read(ImuSample& out) = 0;

  virtual float accelScaleG() const = 0;    // LSB -> g
  virtual float gyroScaleRad() const = 0;   // LSB -> rad/s
};

// No IMU. Gravity stays wherever the simulation was left, which is the same behaviour as a board
// whose IMU failed to answer -- so the host build exercises the path the device falls back to
// rather than a path of its own.
class NullMotionSensor final : public MotionSensor {
 public:
  bool present() const override { return false; }
  uint8_t whoAmI() const override { return 0; }
  bool ready() override { return false; }
  bool read(ImuSample&) override { return false; }
  float accelScaleG() const override { return 0.0f; }
  float gyroScaleRad() const override { return 0.0f; }
};

}  // namespace app
}  // namespace partsim
