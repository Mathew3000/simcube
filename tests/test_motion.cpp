#include "check.h"
#include "partsim/MotionSource.h"

using namespace partsim;

namespace {

constexpr float kImuDt = 1.0f / 208.0f;  // the LSM6DSOX rate the firmware configures

// Feeds the same sample repeatedly, which is what "held still in this pose" looks like.
void hold(MotionSource& m, Vec3 accelG, Vec3 gyro, float seconds) {
  const int n = (int)(seconds / kImuDt);
  for (int i = 0; i < n; ++i) m.update(accelG, gyro, kImuDt);
}

float angleBetween(Vec3 a, Vec3 b) {
  const float c = pclamp(dot(normalize(a), normalize(b)), -1.0f, 1.0f);
  // acos would be libm; compare the chord instead, which is monotone in the angle.
  return psqrt(pmax(0.0f, 2.0f - 2.0f * c));
}

}  // namespace

TEST(axis_map_validates_permutations) {
  CHECK(AxisMap::identity().valid());
  // Swap y and z, negate the new z: a perfectly ordinary mounting.
  CHECK((AxisMap{{0, 2, 1}, {1, 1, -1}}).valid());
  // A repeated axis silently collapses a dimension of motion, so it must be rejected.
  CHECK(!(AxisMap{{0, 1, 1}, {1, 1, 1}}).valid());
  CHECK(!(AxisMap{{0, 1, 3}, {1, 1, 1}}).valid());
  CHECK(!(AxisMap{{0, 1, 2}, {1, 0, 1}}).valid());
}

TEST(axis_map_applies_sign_and_permutation) {
  const AxisMap m{{2, 0, 1}, {1, -1, 1}};
  const Vec3 v = m.apply(Vec3{1.0f, 2.0f, 3.0f});
  CHECK_NEAR(v.x, 3.0f, 1e-6f);   // object x reads device z
  CHECK_NEAR(v.y, -1.0f, 1e-6f);  // object y reads -device x
  CHECK_NEAR(v.z, 2.0f, 1e-6f);
}

// The sign that matters more than any other in this file: get it backwards and the water
// climbs to the ceiling. An accelerometer at rest reads the reaction to gravity, i.e. +1g
// along whichever axis points UP.
TEST(level_and_still_gives_gravity_straight_down) {
  MotionSource m;
  m.init(MotionConfig::defaults(), AxisMap::identity());
  m.seed(Vec3{0.0f, 1.0f, 0.0f});
  hold(m, Vec3{0.0f, 1.0f, 0.0f}, Vec3{0.0f, 0.0f, 0.0f}, 1.0f);

  const Vec3 g = m.gravityObject();
  CHECK_NEAR(g.x, 0.0f, 1e-4f);
  CHECK_NEAR(g.y, -kGravityMag, 1e-3f);
  CHECK_NEAR(g.z, 0.0f, 1e-4f);
  CHECK_NEAR(length(m.containerAccel()), 0.0f, 1e-3f);
  CHECK_NEAR(m.trust(), 1.0f, 1e-4f);
}

TEST(seeding_uses_the_pose_it_was_switched_on_in) {
  MotionSource m;
  m.init(MotionConfig::defaults(), AxisMap::identity());
  CHECK(!m.seeded());
  // Switched on resting on its +x face: down is +x in object space.
  m.seed(Vec3{-1.0f, 0.0f, 0.0f});
  CHECK(m.seeded());
  CHECK_NEAR(m.down().x, 1.0f, 1e-5f);
  // Without seeding it would take the filter seconds to swing round from the -y default,
  // and the cube would visibly pour its water sideways at boot.
  CHECK(angleBetween(m.down(), Vec3{1.0f, 0.0f, 0.0f}) < 1e-3f);
}

TEST(seed_ignores_free_fall) {
  MotionSource m;
  m.init(MotionConfig::defaults(), AxisMap::identity());
  m.seed(Vec3{0.0f, 0.0f, 0.0f});
  CHECK(!m.seeded());
  CHECK_NEAR(m.down().y, -1.0f, 1e-6f);  // default retained
}

TEST(axis_map_is_honoured_end_to_end) {
  // Sensor mounted with its z pointing along the object's +y and its y along object -z.
  MotionSource m;
  m.init(MotionConfig::defaults(), AxisMap{{0, 2, 1}, {1, 1, -1}});
  // Object is level, so the accelerometer's UP axis (object +y == device +z) reads +1g.
  m.seed(Vec3{0.0f, 0.0f, 1.0f});
  hold(m, Vec3{0.0f, 0.0f, 1.0f}, Vec3{0.0f, 0.0f, 0.0f}, 0.5f);
  CHECK_NEAR(m.gravityObject().y, -kGravityMag, 1e-3f);
  CHECK_NEAR(m.gravityObject().z, 0.0f, 1e-3f);
}

TEST(tilt_is_tracked_within_a_second) {
  MotionSource m;
  m.init(MotionConfig::defaults(), AxisMap::identity());
  m.seed(Vec3{0.0f, 1.0f, 0.0f});

  // Tilted 45 degrees about z and then held there, with no gyro help at all -- the
  // accelerometer alone has to bring the estimate round.
  const float s = 0.70710678f;
  hold(m, Vec3{-s, s, 0.0f}, Vec3{0.0f, 0.0f, 0.0f}, 1.0f);

  const Vec3 want{s, -s, 0.0f};
  CHECK(angleBetween(m.down(), want) < 0.02f);  // ~1.1 degrees of chord
}

TEST(gyro_carries_the_estimate_when_the_accelerometer_is_useless) {
  MotionSource m;
  m.init(MotionConfig::defaults(), AxisMap::identity());
  m.seed(Vec3{0.0f, 1.0f, 0.0f});

  // Rotate 90 degrees about z over half a second while the accelerometer reads 3g of
  // nonsense -- the situation during a real shake. The gate should discard the accel
  // entirely and let the gyro do the work.
  const float rate = (0.5f * kPi) / 0.5f;  // rad/s
  const int n = (int)(0.5f / kImuDt);
  for (int i = 0; i < n; ++i) m.update(Vec3{3.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, rate}, kImuDt);

  CHECK_NEAR(m.trust(), 0.0f, 1e-6f);  // accel fully rejected
  // Object rotated +90 deg about z, so world-down (0,-1,0) now reads as (-1,0,0)... in the
  // object frame it is the object that turned, so down swings to +x... check the axis and
  // magnitude rather than restating the rotation: down must have left -y and be in the x/y
  // plane, unit length.
  CHECK_NEAR(length(m.down()), 1.0f, 1e-4f);
  CHECK_NEAR(m.down().z, 0.0f, 1e-4f);
  CHECK(pabs(m.down().x) > 0.9f);   // essentially all the way round
  CHECK(pabs(m.down().y) < 0.15f);
}

TEST(shake_produces_container_acceleration_and_then_decays) {
  MotionSource m;
  m.init(MotionConfig::defaults(), AxisMap::identity());
  m.seed(Vec3{0.0f, 1.0f, 0.0f});
  hold(m, Vec3{0.0f, 1.0f, 0.0f}, Vec3{0.0f, 0.0f, 0.0f}, 0.5f);

  // A shove along +x for 30 ms: 1g of extra specific force. Note the accelerometer reads the
  // reaction, so a device accelerating in +x reads +1g in x on top of gravity.
  float peak = 0.0f;
  for (int i = 0; i < (int)(0.03f / kImuDt); ++i) {
    m.update(Vec3{1.0f, 1.0f, 0.0f}, Vec3{0.0f, 0.0f, 0.0f}, kImuDt);
    peak = pmax(peak, m.containerAccel().x);
  }
  // Order of magnitude: 1g of shove must land as roughly kGravityMag of container accel.
  CHECK(peak > 0.5f * kGravityMag);

  // Held at a constant offset it must fade: a steady 1g sideways is indistinguishable from
  // the cube being tilted, and the high-pass is what stops a long push from reading as an
  // endless shake.
  hold(m, Vec3{1.0f, 1.0f, 0.0f}, Vec3{0.0f, 0.0f, 0.0f}, 1.0f);
  CHECK(pabs(m.containerAccel().x) < 0.05f * peak);
}

TEST(gyro_bias_is_learned_only_while_still) {
  MotionConfig cfg = MotionConfig::defaults();
  MotionSource m;
  m.init(cfg, AxisMap::identity());
  m.seed(Vec3{0.0f, 1.0f, 0.0f});

  // A small constant offset on the z gyro while resting level: unambiguously bias.
  const float offset = 0.01f;  // rad/s, about 0.6 deg/s -- typical for this part
  hold(m, Vec3{0.0f, 1.0f, 0.0f}, Vec3{0.0f, 0.0f, offset}, 10.0f);
  CHECK_NEAR(m.gyroBias().z, offset, 0.002f);
  // ...and having learned it, the estimate must not have drifted away from level.
  CHECK(angleBetween(m.down(), Vec3{0.0f, -1.0f, 0.0f}) < 0.01f);
}

TEST(real_rotation_is_not_absorbed_into_the_bias) {
  MotionSource m;
  m.init(MotionConfig::defaults(), AxisMap::identity());
  m.seed(Vec3{0.0f, 1.0f, 0.0f});

  // Turning at 1 rad/s is far above the stillness gate, so none of it may be learned as
  // bias. An ungated tracker fails this and the cube slowly forgets which way it is tilted.
  hold(m, Vec3{0.0f, 1.0f, 0.0f}, Vec3{0.0f, 1.0f, 0.0f}, 2.0f);
  CHECK(length(m.gyroBias()) < 1e-4f);
}

TEST(gravity_magnitude_is_always_the_solver_scale) {
  // Whatever the accelerometer reads, the solver must get a vector of exactly kGravityMag --
  // it is tuned against the CFL cap at that magnitude, and a 3g shake arriving as 3x gravity
  // would blow the fluid through the walls.
  MotionSource m;
  m.init(MotionConfig::defaults(), AxisMap::identity());
  m.seed(Vec3{0.0f, 4.0f, 0.0f});
  CHECK_NEAR(length(m.gravityObject()), kGravityMag, 1e-3f);
  hold(m, Vec3{2.0f, -3.0f, 1.5f}, Vec3{0.3f, -0.2f, 0.1f}, 0.5f);
  CHECK_NEAR(length(m.gravityObject()), kGravityMag, 1e-3f);
}
