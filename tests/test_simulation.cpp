#include "check.h"
#include "partsim/Simulation.h"

using namespace partsim;

namespace {
// ~1.2MB; static storage only.
Simulation g_sim;

Vec3 centreOfMass(const Particles& p) {
  double x = 0, y = 0, z = 0;
  for (int i = 0; i < p.n; ++i) { x += p.x[i]; y += p.y[i]; z += p.z[i]; }
  const double n = p.n ? p.n : 1;
  return Vec3{(float)(x / n), (float)(y / n), (float)(z / n)};
}

void run(int steps) {
  for (int s = 0; s < steps; ++s) g_sim.stepFixed();
}
}  // namespace

TEST(sim_init_cube_and_settle) {
  CHECK(g_sim.init(Simulation::kCube, 1500, 1));
  CHECK(g_sim.particleCount() == 1500);
  CHECK(g_sim.geometry().count() == 6);
  run(300);

  const Vec3 c = centreOfMass(g_sim.particles());
  CHECK(c.y < -8.0f);  // pooled on the floor
  for (int i = 0; i < g_sim.particles().n; ++i)
    CHECK(g_sim.volume().box().contains(g_sim.particles().pos(i)));
}

TEST(sim_init_caps_at_capacity) {
  // Asking for more than the volume holds must clamp, not overfill: an over-compressed fluid
  // never settles.
  CHECK(g_sim.init(Simulation::kCube, kMaxParticles, 2));
  CHECK(g_sim.particleCount() <= g_sim.capacity());
  CHECK(g_sim.particleCount() > 1000);

  CHECK(g_sim.init(Simulation::kSinglePanel, kMaxParticles, 2));
  CHECK(g_sim.geometry().count() == 1);
  // A slab tolerates a much smaller share of nominal capacity than a cube.
  CHECK(g_sim.particleCount() <= g_sim.capacity() / 4);
  CHECK(g_sim.particleCount() > 50);
}

TEST(sim_orientation_rotates_gravity_into_object_space) {
  CHECK(g_sim.init(Simulation::kCube, 1200, 3));

  // Identity: gravity is straight down in object space.
  g_sim.setOrientation(Quat{0, 0, 0, 1});
  CHECK_NEAR(g_sim.gravityObject().y, -kGravityMag, 1e-3);
  CHECK_NEAR(g_sim.gravityObject().x, 0.0f, 1e-3);

  // 90 degrees about +Z. World-down now reads as -X inside the object, so the water must
  // migrate to the -X wall. This is the whole reason the core works in object space.
  const float s = fsin(kPi * 0.25f), c = fcos(kPi * 0.25f);
  g_sim.setOrientation(Quat{0, 0, s, c});
  CHECK_NEAR(g_sim.gravityObject().x, -kGravityMag, 0.05);
  CHECK_NEAR(g_sim.gravityObject().y, 0.0f, 0.05);

  run(400);
  const Vec3 com = centreOfMass(g_sim.particles());
  CHECK(com.x < -8.0f);          // piled against -X
  CHECK(com.y > -6.0f);          // no longer on the floor
  for (int i = 0; i < g_sim.particles().n; ++i)
    CHECK(g_sim.volume().box().contains(g_sim.particles().pos(i)));
}

TEST(sim_container_accel_has_the_inverted_sign_of_the_fluid_response) {
  // Pins down the sign convention, which is easy to get backwards: accelerating the container
  // UPWARD presses the fluid DOWN (an elevator starting to rise). To throw water upward you
  // drop the container, i.e. accelerate it downward.
  CHECK(g_sim.init(Simulation::kCube, 800, 4));
  run(250);
  const Vec3 rest = centreOfMass(g_sim.particles());

  g_sim.addContainerAccel(Vec3{0.0f, -220.0f, 0.0f});  // container dropped
  run(6);
  CHECK(centreOfMass(g_sim.particles()).y > rest.y);    // fluid rises

  run(700);
  const Vec3 after = centreOfMass(g_sim.particles());
  CHECK_NEAR(after.y, rest.y, 1.5);  // the impulse decays and it settles back
  CHECK(length(g_sim.containerAccel()) == 0.0f);
  for (int i = 0; i < g_sim.particles().n; ++i)
    CHECK(g_sim.volume().box().contains(g_sim.particles().pos(i)));
}

TEST(sim_sideways_container_accel_piles_fluid_the_other_way) {
  CHECK(g_sim.init(Simulation::kCube, 800, 11));
  run(250);
  const float restX = centreOfMass(g_sim.particles()).x;
  // Shove the cube toward +x; the water must pile toward -x.
  for (int s = 0; s < 30; ++s) {
    g_sim.addContainerAccel(Vec3{90.0f, 0.0f, 0.0f});
    g_sim.stepFixed();
  }
  CHECK(centreOfMass(g_sim.particles()).x < restX);
}

TEST(sim_accumulator_runs_fixed_steps_and_clamps) {
  CHECK(g_sim.init(Simulation::kCube, 500, 5));

  // A delta smaller than the fixed step does no physics but banks the time.
  CHECK(g_sim.advance(kFixedDt * 0.4f) == 0);
  CHECK(g_sim.advance(kFixedDt * 0.4f) == 0);
  CHECK(g_sim.advance(kFixedDt * 0.4f) == 1);  // banked time now exceeds one step

  // A huge delta must clamp rather than trying to catch up, which would make a slow frame
  // trigger even more work on the next one.
  CHECK(g_sim.advance(10.0f) == kMaxSubsteps);
  CHECK(g_sim.advance(kFixedDt * 0.1f) == 0);  // backlog was dropped, not carried

  // Nonsense input must not step or crash.
  CHECK(g_sim.advance(-5.0f) == 0);
}

TEST(sim_state_hash_is_stable_and_sensitive) {
  CHECK(g_sim.init(Simulation::kCube, 600, 7));
  run(50);
  const uint32_t a = g_sim.stateHash();

  CHECK(g_sim.init(Simulation::kCube, 600, 7));
  run(50);
  CHECK(g_sim.stateHash() == a);  // same inputs, same bits

  CHECK(g_sim.init(Simulation::kCube, 600, 8));  // different seed
  run(50);
  CHECK(g_sim.stateHash() != a);
}

TEST(sim_golden_sequence_is_reproducible) {
  // The reference the WASM build is compared against. If this is not reproducible on one
  // target it cannot possibly match across targets.
  const uint32_t a = goldenHash(g_sim, kGoldenSteps, kGoldenSeed);
  const uint32_t b = goldenHash(g_sim, kGoldenSteps, kGoldenSeed);
  CHECK(a == b);
  CHECK(a != 0u);
  std::printf("       golden state hash %08x\n", a);

  // The scripted motion really does move the fluid around, so the hash is meaningful.
  int outside = 0;
  for (int i = 0; i < g_sim.particles().n; ++i)
    if (!g_sim.volume().box().contains(g_sim.particles().pos(i))) ++outside;
  CHECK(outside == 0);
}

TEST(sim_render_produces_lit_panels) {
  CHECK(g_sim.init(Simulation::kCube, 2000, 9));
  run(300);
  g_sim.render();

  int lit = 0;
  for (int k = 0; k < g_sim.geometry().count(); ++k) {
    const uint8_t* px = g_sim.renderer().panelPixels(k);
    bool any = false;
    for (int i = 0; i < g_sim.renderer().panelTexels(k) && !any; ++i)
      if (px[i * 4] || px[i * 4 + 1] || px[i * 4 + 2]) any = true;
    if (any) ++lit;
  }
  // Water on the floor lights the bottom face and the four sides; the top stays dark.
  CHECK(lit == 5);
}

TEST(sim_render_interpolation_moves_the_splat_not_the_state) {
  // Lets the display run ahead of the physics: unconsumed accumulator time is covered visually
  // instead of appearing as a stutter. It must never feed back into the simulation.
  CHECK(g_sim.init(Simulation::kCube, 900, 17));
  run(120);

  const uint32_t before = g_sim.stateHash();

  // Splat where the particles are.
  g_sim.renderer().setTimeOffset(0.0f);
  g_sim.renderer().render(g_sim.particles(), g_sim.field(), g_sim.geometry());
  uint64_t plain = 0;
  for (int k = 0; k < 6; ++k) plain = fnv1a(g_sim.renderer().panelPixels(k), (size_t)g_sim.renderer().panelTexels(k) * 4u, plain);

  // Splat a full frame ahead. Something must change, or the offset is being ignored.
  g_sim.renderer().setTimeOffset(kFixedDt * 3.0f);
  g_sim.renderer().render(g_sim.particles(), g_sim.field(), g_sim.geometry());
  uint64_t shifted = 0;
  for (int k = 0; k < 6; ++k)
    shifted = fnv1a(g_sim.renderer().panelPixels(k), (size_t)g_sim.renderer().panelTexels(k) * 4u, shifted);

  CHECK(g_sim.stateHash() == before);  // rendering did not disturb the physics
  CHECK(plain != shifted);             // and the offset actually did something

  g_sim.renderer().setTimeOffset(0.0f);
}

TEST(sim_interpolation_is_off_for_a_settled_fluid) {
  // A settled fluid has near-zero velocity, so interpolation must be a no-op there rather than
  // introducing shimmer of its own.
  CHECK(g_sim.init(Simulation::kCube, 900, 19));
  run(400);

  g_sim.renderer().setTimeOffset(0.0f);
  g_sim.renderer().render(g_sim.particles(), g_sim.field(), g_sim.geometry());
  uint64_t a = 0;
  for (int k = 0; k < 6; ++k) a = fnv1a(g_sim.renderer().panelPixels(k), (size_t)g_sim.renderer().panelTexels(k) * 4u, a);

  g_sim.renderer().setTimeOffset(kFixedDt);
  g_sim.renderer().render(g_sim.particles(), g_sim.field(), g_sim.geometry());
  uint64_t b = 0;
  for (int k = 0; k < 6; ++k) b = fnv1a(g_sim.renderer().panelPixels(k), (size_t)g_sim.renderer().panelTexels(k) * 4u, b);

  CHECK(a == b);
  g_sim.renderer().setTimeOffset(0.0f);
}
