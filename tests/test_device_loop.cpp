// The firmware's control loop, run on the host against a synthetic IMU.
//
// Every piece of this chain is unit-tested separately -- the filter in test_motion.cpp, the solver
// in test_solver.cpp -- and that is not the same as testing the chain. A sign error between them
// (an accelerometer convention, a gravity direction, a container-acceleration sign) passes both
// sets of unit tests and produces a cube whose water climbs to the ceiling. Milestone 1 shipped
// exactly that class of bug twice.
//
// So: drive the same sequence simTask does, with accelerometer readings this cube would actually
// produce, and check the fluid ends up where physics says it should. This is the closest thing to
// a hardware test that exists before the hardware does.
#include "check.h"
#include "partsim/MotionSource.h"
#include "partsim/Simulation.h"

using namespace partsim;

namespace {

Simulation g_sim;  // ~1.2MB
MotionSource g_motion;

constexpr float kImuDt = 1.0f / 208.0f;
constexpr float kFrameDt = 1.0f / 60.0f;

Vec3 centroid() {
  const Particles& p = g_sim.particles();
  if (p.n == 0) return Vec3{0.0f, 0.0f, 0.0f};
  double x = 0.0, y = 0.0, z = 0.0;
  for (int i = 0; i < p.n; ++i) { x += p.x[i]; y += p.y[i]; z += p.z[i]; }
  return Vec3{(float)(x / p.n), (float)(y / p.n), (float)(z / p.n)};
}

float meanSpeed() {
  const Particles& p = g_sim.particles();
  if (p.n == 0) return 0.0f;
  double s = 0.0;
  for (int i = 0; i < p.n; ++i) s += length(p.vel(i));
  return (float)(s / p.n);
}

// One firmware frame: drain the IMU samples that would have arrived, then step the simulation.
// Same order and the same two setters simTask uses.
void frame(Vec3 accelG, Vec3 gyro) {
  const int samples = (int)(kFrameDt / kImuDt);  // 3 at 60fps
  for (int i = 0; i < samples; ++i) {
    if (!g_motion.seeded()) g_motion.seed(accelG);
    g_motion.update(accelG, gyro, kImuDt);
  }
  g_sim.setGravityObject(g_motion.gravityObject());
  g_sim.setContainerAccel(g_motion.containerAccel());
  g_sim.stepFixed();
}

void run(Vec3 accelG, Vec3 gyro, float seconds) {
  const int n = (int)(seconds / kFrameDt);
  for (int i = 0; i < n; ++i) frame(accelG, gyro);
}

// Resting level, an accelerometer reads +1g on the axis pointing up.
const Vec3 kLevel{0.0f, 1.0f, 0.0f};

void settleLevel() {
  CHECK(g_sim.initScene(Simulation::kCube, 0, 7));  // water tank
  g_motion.init(MotionConfig::defaults(), AxisMap::identity());
  g_sim.setAutoCycle(false);
  run(kLevel, Vec3{0.0f, 0.0f, 0.0f}, 4.0f);
}

}  // namespace

TEST(device_loop_water_settles_at_the_bottom_when_level) {
  settleLevel();
  const Vec3 c = centroid();
  const Aabb b = g_sim.volume().box();
  // Below the middle of the box, and not moving.
  CHECK(c.y < b.center().y);
  CHECK(meanSpeed() < 1.0f);
  std::printf("       level: centroid y %.2f (box %.1f..%.1f), mean|v| %.3f\n", c.y, b.lo.y,
              b.hi.y, meanSpeed());
}

TEST(device_loop_water_follows_gravity_when_the_object_is_tilted) {
  settleLevel();
  const Vec3 before = centroid();

  // Rolled 90 degrees so the object's +x face is now underneath. In that pose the axis pointing
  // up is object -x, so the accelerometer reads -1g in x. This is the whole reason the test
  // exists: if that sign is wrong anywhere in the chain, the water goes to -x instead.
  run(Vec3{-1.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, 0.0f}, 6.0f);

  const Vec3 after = centroid();
  const Aabb b = g_sim.volume().box();
  std::printf("       tilt: centroid x %.2f -> %.2f, y %.2f -> %.2f\n", before.x, after.x,
              before.y, after.y);

  CHECK(after.x > before.x + 2.0f);  // moved toward +x, the new low side
  CHECK(after.x > b.center().x);     // and past the middle
  CHECK(after.y > before.y);         // no longer piled at the old floor
}

TEST(device_loop_gravity_direction_matches_the_pose_on_every_face) {
  // Six poses, six expected low axes. Cheap, and it covers all the sign combinations at once --
  // a permuted axis map or a flipped sign shows up here rather than during bring-up.
  struct Case { Vec3 accel; int axis; float sign; const char* name; };
  const Case cases[6] = {
      {{0.0f, 1.0f, 0.0f}, 1, -1.0f, "level"},
      {{0.0f, -1.0f, 0.0f}, 1, 1.0f, "upside down"},
      {{-1.0f, 0.0f, 0.0f}, 0, 1.0f, "on +x face"},
      {{1.0f, 0.0f, 0.0f}, 0, -1.0f, "on -x face"},
      {{0.0f, 0.0f, -1.0f}, 2, 1.0f, "on +z face"},
      {{0.0f, 0.0f, 1.0f}, 2, -1.0f, "on -z face"},
  };

  for (const Case& c : cases) {
    MotionSource m;
    m.init(MotionConfig::defaults(), AxisMap::identity());
    m.seed(c.accel);
    const Vec3 g = m.gravityObject();
    const float comp[3] = {g.x, g.y, g.z};
    // Gravity points along the expected axis, with the expected sign, at full magnitude.
    CHECK_NEAR(comp[c.axis], c.sign * kGravityMag, 1e-3f);
    for (int k = 0; k < 3; ++k)
      if (k != c.axis) CHECK_NEAR(comp[k], 0.0f, 1e-3f);
  }
}

TEST(device_loop_shaking_disturbs_the_fluid_and_it_recovers) {
  settleLevel();
  const float restSpeed = meanSpeed();

  // A 6 Hz shake along x at 1.5g, held for a second and a half: a hand shaking a cube. The
  // accelerometer sees gravity plus the container's own acceleration.
  //
  // The threshold is relative, and measured rather than guessed. A sweep over 3/6/12 Hz at
  // 1/1.5/3 g gives 1.9x to 7.3x the resting mean speed: it rises with amplitude, and FALLS with
  // frequency at fixed amplitude, because a faster oscillation reverses before the fluid builds
  // any speed. An absolute "+2.0 units" threshold looked reasonable and was simply wrong at the
  // top of that range.
  float peak = 0.0f, peakAccel = 0.0f;
  const int n = (int)(1.5f / kFrameDt);
  for (int i = 0; i < n; ++i) {
    const float t = (float)i * kFrameDt;
    const float a = 1.5f * fsin(kTwoPi * 6.0f * t);
    frame(Vec3{a, 1.0f, 0.0f}, Vec3{0.0f, 0.0f, 0.0f});
    peak = pmax(peak, meanSpeed());
    peakAccel = pmax(peakAccel, length(g_sim.containerAccel()));
  }
  std::printf("       shake: mean|v| rest %.3f -> peak %.3f (%.1fx), container accel %.1f\n",
              restSpeed, peak, peak / restSpeed, peakAccel);
  CHECK(peak > 2.0f * restSpeed);  // it actually moved
  // ...and the shake reached the solver with authority, not as a rounding error. Asserted
  // separately so that a regression which silently zeroes the coupling still fails even if the
  // fluid happens to be sloshing from something else.
  CHECK(peakAccel > 0.5f * kGravityMag);

  // And it must come back to rest rather than staying agitated -- a shake that pumps energy in
  // without it draining is how a PBF solver ends up boiling.
  run(kLevel, Vec3{0.0f, 0.0f, 0.0f}, 6.0f);
  const float settled = meanSpeed();
  std::printf("       shake: settled back to mean|v| %.3f\n", settled);
  CHECK(settled < 1.0f);

  // Still inside the box, which is the property a shake is most likely to break.
  const Aabb b = g_sim.volume().box();
  const Particles& p = g_sim.particles();
  for (int i = 0; i < p.n; ++i) CHECK(b.contains(p.pos(i)));
}

TEST(device_loop_survives_a_dead_imu) {
  // The firmware treats a missing sensor as non-fatal: a cube with a dead IMU should still be a
  // lamp. That means the simulation runs with whatever gravity it was initialised with, and must
  // not be fed a zero vector -- normalize(0) is (0,0,0), and a solver with no gravity at all
  // looks like a hang.
  CHECK(g_sim.initScene(Simulation::kCube, 0, 11));
  MotionSource m;
  m.init(MotionConfig::defaults(), AxisMap::identity());
  CHECK(!m.seeded());
  // Never seeded, never updated: the firmware skips both setters in this case.
  for (int i = 0; i < 200; ++i) g_sim.stepFixed();

  CHECK_NEAR(length(g_sim.gravityObject()), kGravityMag, 1e-3f);
  const Vec3 c = centroid();
  CHECK(c.y < g_sim.volume().box().center().y);  // still falls downward
  CHECK(meanSpeed() < 2.0f);                     // and still settles
}

TEST(device_loop_free_fall_does_not_destroy_the_gravity_estimate) {
  settleLevel();
  // Thrown, or dropped. The accelerometer reads ~0 and its direction is meaningless; the gate
  // must reject it outright rather than normalising noise into a new "down".
  const Vec3 downBefore = g_motion.down();
  run(Vec3{0.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, 0.0f}, 0.5f);
  const Vec3 downAfter = g_motion.down();
  CHECK_NEAR(length(downAfter), 1.0f, 1e-4f);
  CHECK_NEAR(dot(downBefore, downAfter), 1.0f, 1e-3f);  // unchanged
  CHECK_NEAR(g_motion.trust(), 0.0f, 1e-6f);
}
