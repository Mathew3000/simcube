#include "check.h"
#include "partsim/Rng.h"
#include "partsim/Solver.h"

using namespace partsim;

namespace {
// Far too big for the stack.
Particles g_p;
SpatialHash g_h;
Solver g_solver;
float g_scratch[kMaxParticles];

// A compact block resting ON the floor, which then slumps under its own weight.
//
// Deliberately not a tall dropped column: that makes the final spread a function of splash
// energy rather than of the material, and friction can only resist motion, never gather
// material back in. This measurement is worthless if the deposition is violent.
void blockOnFloor(const SimVolume& v, int count, uint8_t material, uint32_t seed) {
  Rng r(seed);
  g_p.clear();
  const float sp = kRestSpacing * (material == kSand ? 0.8f : 1.0f);
  const int side = 9;
  const float lo = v.box().lo.y;
  for (int gy = 0; gy < 40 && g_p.n < count; ++gy)
    for (int gz = 0; gz < side && g_p.n < count; ++gz)
      for (int gx = 0; gx < side && g_p.n < count; ++gx)
        g_p.add(Vec3{((float)gx - (float)side * 0.5f + 0.5f) * sp + r.nextSigned() * 0.05f,
                     lo + (0.5f + (float)gy) * sp,
                     ((float)gz - (float)side * 0.5f + 0.5f) * sp + r.nextSigned() * 0.05f},
                Vec3{0, 0, 0}, material);
}

struct Pile {
  float peak;      // highest particle, above the floor
  float meanSpeed;
  int outside;
};

Pile slump(const SimVolume& v, int count, uint8_t material, int steps, uint32_t seed) {
  blockOnFloor(v, count, material, seed);
  for (int s = 0; s < steps; ++s)
    g_solver.step(g_p, v, g_h, g_scratch, defaultMaterials(),
                  Vec3{0.0f, -kGravityMag, 0.0f}, kFixedDt);

  Pile r{-1e30f, 0.0f, 0};
  double speed = 0.0;
  for (int i = 0; i < g_p.n; ++i) {
    if (g_p.y[i] > r.peak) r.peak = g_p.y[i];
    speed += length(g_p.vel(i));
    if (!v.box().contains(g_p.pos(i))) ++r.outside;
  }
  r.peak -= v.box().lo.y;
  r.meanSpeed = (float)(speed / g_p.n);
  return r;
}

SimVolume cubeVolume() {
  SimVolume v;
  v.build(Geometry::cube(32, 1.0f), kSlabDepth, kCellSize);
  g_solver.init();
  return v;
}
}  // namespace

TEST(sand_holds_a_heap_where_water_spreads_flat) {
  // The defining difference between a granular material and a heavy liquid. Same count, same
  // gentle deposition, same gravity -- only the material parameters differ.
  const SimVolume v = cubeVolume();
  const Pile water = slump(v, 600, kWater, 900, 3);
  const Pile sand = slump(v, 600, kSand, 900, 3);

  std::printf("       water peak %.2f (|v| %.3f)   sand peak %.2f (|v| %.3f)\n",
              water.peak, water.meanSpeed, sand.peak, sand.meanSpeed);

  CHECK(water.outside == 0);
  CHECK(sand.outside == 0);
  CHECK(sand.peak > water.peak * 1.8f);  // measured ~5.6 vs ~2.6
  CHECK(sand.peak > 3.5f);
}

TEST(sand_comes_to_rest_and_does_not_creep) {
  // Without the static-friction deadband a pile creeps indefinitely and slowly flattens, which
  // reads as the sand melting. So it must actually stop, not merely slow down.
  const SimVolume v = cubeVolume();
  const Pile settled = slump(v, 600, kSand, 900, 11);
  CHECK(settled.meanSpeed < 0.3f);

  const float peakBefore = settled.peak;
  for (int s = 0; s < 600; ++s)
    g_solver.step(g_p, v, g_h, g_scratch, defaultMaterials(),
                  Vec3{0.0f, -kGravityMag, 0.0f}, kFixedDt);
  float peakAfter = -1e30f;
  for (int i = 0; i < g_p.n; ++i) peakAfter = pmax(peakAfter, g_p.y[i]);
  peakAfter -= v.box().lo.y;

  // The pile must not slowly sag over another 10 seconds of simulated time.
  std::printf("       peak %.2f -> %.2f after 600 more steps\n", peakBefore, peakAfter);
  CHECK(peakAfter > peakBefore - 0.6f);
}

TEST(sand_sinks_through_water) {
  // Emerges from the density constraint (sand's rest density is 2x water's), not from a special
  // case in the gravity path.
  const SimVolume v = cubeVolume();
  g_p.clear();
  Rng r(5);
  for (int i = 0; i < 1800; ++i)
    g_p.add(Vec3{r.nextSigned() * 15.0f, -16.0f + r.nextFloat() * 14.0f,
                 r.nextSigned() * 15.0f}, Vec3{0, 0, 0}, kWater);
  for (int i = 0; i < 400; ++i)
    g_p.add(Vec3{r.nextSigned() * 8.0f, 2.0f + r.nextFloat() * 8.0f, r.nextSigned() * 8.0f},
            Vec3{0, 0, 0}, kSand);

  for (int s = 0; s < 1200; ++s)
    g_solver.step(g_p, v, g_h, g_scratch, defaultMaterials(),
                  Vec3{0.0f, -kGravityMag, 0.0f}, kFixedDt);

  double wy = 0, sy = 0;
  int wn = 0, sn = 0, outside = 0;
  for (int i = 0; i < g_p.n; ++i) {
    if (g_p.mat[i] == kSand) { sy += g_p.y[i]; ++sn; } else { wy += g_p.y[i]; ++wn; }
    if (!v.box().contains(g_p.pos(i))) ++outside;
  }
  const double waterY = wy / wn, sandY = sy / sn;
  std::printf("       water meanY %.2f   sand meanY %.2f\n", waterY, sandY);
  CHECK(outside == 0);
  CHECK(sandY < waterY - 0.5);  // sand settled underneath
}

TEST(sand_friction_is_stable_at_the_default_coefficient) {
  // Friction is bounded by mu * contact overlap (Coulomb) and projected sequentially per
  // contact. Accumulating bounded corrections and applying the sum instead over-corrects by
  // roughly the neighbour count and blows up -- this guards that regression.
  const SimVolume v = cubeVolume();
  const Pile pile = slump(v, 900, kSand, 700, 21);
  CHECK(pile.outside == 0);
  CHECK(pile.meanSpeed < 1.0f);
  for (int i = 0; i < g_p.n; ++i) {
    CHECK(g_p.x[i] == g_p.x[i]);  // no NaN
    CHECK(length(g_p.vel(i)) < 100.0f);
  }
}

TEST(sand_survives_shaking_without_leaking) {
  const SimVolume v = cubeVolume();
  blockOnFloor(v, 700, kSand, 33);
  Rng r(9);
  Vec3 g{0, -kGravityMag, 0};
  for (int s = 0; s < 500; ++s) {
    if (s % 12 == 0)
      g = normalize(Vec3{r.nextSigned(), r.nextSigned(), r.nextSigned()}) *
          (kGravityMag * 2.5f);
    g_solver.step(g_p, v, g_h, g_scratch, defaultMaterials(), g, kFixedDt);
  }
  int outside = 0;
  for (int i = 0; i < g_p.n; ++i)
    if (!v.box().contains(g_p.pos(i))) ++outside;
  CHECK(outside == 0);
}

TEST(water_is_unaffected_by_the_friction_pass) {
  // Water has friction 0, so pass D must skip it entirely -- both for cost and because any
  // friction on water would visibly stiffen it.
  CHECK(defaultMaterials()[kWater].friction == 0.0f);
  CHECK(defaultMaterials()[kSand].friction > 0.0f);
  CHECK(defaultMaterials()[kSand].restDensityScale >
        defaultMaterials()[kWater].restDensityScale);
}
