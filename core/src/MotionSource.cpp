#include "partsim/MotionSource.h"

namespace partsim {

void MotionSource::init(const MotionConfig& cfg, AxisMap map) {
  cfg_ = cfg;
  map_ = map.valid() ? map : AxisMap::identity();
  down_ = Vec3{0.0f, -1.0f, 0.0f};
  bias_ = Vec3{0.0f, 0.0f, 0.0f};
  slow_ = Vec3{0.0f, 0.0f, 0.0f};
  shake_ = Vec3{0.0f, 0.0f, 0.0f};
  trust_ = 0.0f;
  seeded_ = false;
}

void MotionSource::seed(Vec3 accelG) {
  const Vec3 a = map_.apply(accelG);
  const float m2 = length2(a);
  if (m2 < 1e-6f) return;  // free-fall or a dead sensor: keep the default and wait
  // An accelerometer at rest reads the reaction to gravity, i.e. +1g along the axis pointing
  // up. Down is therefore the NEGATION of the reading -- the single sign that, if flipped,
  // makes the water climb the ceiling.
  down_ = normalize(a) * -1.0f;
  slow_ = Vec3{0.0f, 0.0f, 0.0f};
  shake_ = Vec3{0.0f, 0.0f, 0.0f};
  seeded_ = true;
}

void MotionSource::update(Vec3 accelG, Vec3 gyroRad, float dt) {
  if (dt <= 0.0f) return;
  const Vec3 a = map_.apply(accelG);
  const Vec3 wRaw = map_.apply(gyroRad);

  if (!seeded_) {
    seed(accelG);
    if (!seeded_) return;
  }

  // --- gyro bias, estimated only while genuinely still ------------------------------------
  // Gated, not continuous: an ungated tracker slowly absorbs real rotation into the bias, so
  // a cube left tilted on a shelf drifts back to thinking it is level.
  const float aMag = length(a);
  const float still = pabs(aMag - 1.0f);
  if (still < cfg_.stillAccelG && length2(wRaw) < cfg_.stillGyro * cfg_.stillGyro) {
    const float k = pclamp(cfg_.biasRate * dt, 0.0f, 1.0f);
    bias_ += (wRaw - bias_) * k;
  }
  const Vec3 w = wRaw - bias_;

  // --- propagate "down" by the gyro -------------------------------------------------------
  // A vector fixed in the world rotates by -omega*dt when expressed in a frame that is itself
  // rotating by +omega*dt, so d(down)/dt = -w x down == down x w.
  down_ = normalize(down_ + cross(down_, w) * dt);

  // --- correct with the accelerometer, gated on how believable it is -----------------------
  // The gate is the whole reason this works during interaction: mid-shake the reading is
  // mostly linear acceleration and says nothing about which way is down, so it is discarded
  // and the gyro carries the estimate until the cube is calm again.
  trust_ = pclamp(1.0f - still / cfg_.trustWindowG, 0.0f, 1.0f);
  if (trust_ > 0.0f && aMag > 1e-4f) {
    const Vec3 measured = a * (-1.0f / aMag);
    const float alpha = cfg_.alphaMax * trust_;
    down_ = normalize(down_ + (measured - down_) * alpha);
  }

  // --- shake --------------------------------------------------------------------------------
  // The reading is specific force: a = a_device - g. With an estimate of g in hand the
  // device's own acceleration is just a + down. What is left is slow error from the tilt
  // estimate, which a high-pass removes: shake is what changes faster than kShakeCornerHz.
  const Vec3 lin = a + down_;
  const float k = pclamp(kTwoPi * cfg_.shakeCornerHz * dt, 0.0f, 1.0f);
  slow_ += (lin - slow_) * k;
  shake_ = lin - slow_;
}

}  // namespace partsim
