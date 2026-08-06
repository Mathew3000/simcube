#pragma once
#include "partsim/Config.h"
#include "partsim/Types.h"

namespace partsim {

// Device axis -> object axis, as a signed permutation.
//
// The single place the physical mounting of the IMU is described. Every "why is gravity
// sideways" bug on a device like this comes from that knowledge being smeared across the
// firmware; here it is one table, and tests/test_motion.cpp checks it round-trips.
struct AxisMap {
  int8_t axis[3];  // object component k reads device component axis[k]
  int8_t sign[3];  // ...multiplied by this

  static AxisMap identity() { return AxisMap{{0, 1, 2}, {1, 1, 1}}; }

  Vec3 apply(Vec3 v) const {
    const float c[3] = {v.x, v.y, v.z};
    return Vec3{(float)sign[0] * c[axis[0]], (float)sign[1] * c[axis[1]],
                (float)sign[2] * c[axis[2]]};
  }

  // A typo'd table that repeats an axis silently collapses one dimension of motion, which
  // looks like a flaky sensor rather than a config error. Checked at init.
  bool valid() const {
    int seen = 0;
    for (int k = 0; k < 3; ++k) {
      if (axis[k] < 0 || axis[k] > 2) return false;
      if (sign[k] != 1 && sign[k] != -1) return false;
      seen |= 1 << axis[k];
    }
    return seen == 0x7;
  }
};

struct MotionConfig {
  float alphaMax;       // strongest blend toward the accelerometer, per update
  float trustWindowG;   // |accel magnitude - 1g| beyond which the accel is ignored entirely
  float shakeCornerHz;  // high-pass corner separating shake from residual tilt error
  float shakeGain;      // scales shake into container acceleration
  float biasRate;       // gyro-bias tracking rate, per second, while held still
  float stillGyro;      // rad/s below which "still" is plausible
  float stillAccelG;    // and |accel| must be within this of 1g as well
  float gravityMag;     // sim units per g -- the bridge from SI to the solver's scale

  static MotionConfig defaults() {
    // alphaMax is small on purpose. The accelerometer is the only absolute reference for
    // tilt, but during handling it is dominated by linear acceleration; leaning on it hard
    // enough to track a tilt in one frame also means every shake yanks "down" around. 0.02
    // at 208 Hz is a ~0.35 s time constant, which is far faster than anyone can rotate a
    // cube and far slower than a shake.
    return MotionConfig{0.02f, 0.5f, 5.0f, 1.0f, 0.5f, 0.05f, 0.05f, kGravityMag};
  }
};

// Turns raw IMU samples into the two vectors the simulation actually consumes: object-space
// gravity, and the container's acceleration.
//
// This lives in core/, not in platform/esp32/, for one reason: it is the only part of the
// hardware path that is pure math, so it is also the only part that can be tested without
// hardware. The ESP32 layer above it does nothing but read registers and call update().
//
// Deliberately NOT part of the deterministic golden path -- it is driven by wall-clock sensor
// samples, so there is nothing to reproduce. It still avoids libm transcendentals so the
// no-libm guard stays a simple grep over all of core/.
class MotionSource {
 public:
  void init(const MotionConfig& cfg, AxisMap map);

  // Seeds the gravity estimate directly from one sample, skipping the filter's settling
  // time. Called once at boot: without it the cube spends its first second convinced that
  // down is -y no matter which way up it was switched on.
  void seed(Vec3 accelG);

  // accelG in g (specific force, so +1g along the axis pointing UP at rest -- that is what
  // an accelerometer reads), gyroRad in rad/s, both in DEVICE axes.
  void update(Vec3 accelG, Vec3 gyroRad, float dt);

  // Object-space gravity, ready for Simulation::setGravityObject.
  Vec3 gravityObject() const { return down_ * cfg_.gravityMag; }
  // Object-space container acceleration, ready for Simulation::setContainerAccel.
  Vec3 containerAccel() const { return shake_ * (cfg_.gravityMag * cfg_.shakeGain); }

  // Unit "down" in object space, for diagnostics and the serial console.
  Vec3 down() const { return down_; }
  Vec3 gyroBias() const { return bias_; }
  // How much of the accelerometer is being trusted right now: 1 while at rest, 0 mid-shake.
  float trust() const { return trust_; }
  bool seeded() const { return seeded_; }

 private:
  MotionConfig cfg_ = MotionConfig::defaults();
  AxisMap map_ = AxisMap::identity();

  Vec3 down_{0.0f, -1.0f, 0.0f};  // unit, object frame
  Vec3 bias_{0.0f, 0.0f, 0.0f};   // gyro bias, object frame (post-remap)
  Vec3 slow_{0.0f, 0.0f, 0.0f};   // low-passed linear accel, the part shake excludes
  Vec3 shake_{0.0f, 0.0f, 0.0f};  // high-passed linear accel, in g
  float trust_ = 0.0f;
  bool seeded_ = false;
};

}  // namespace partsim
