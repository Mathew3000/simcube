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

int fillBottom(Particles& p, const Aabb& box, int want, uint8_t mat, uint32_t seed) {
  Rng r(seed);
  const float d = kRestSpacing;
  const int nx = (int)(box.size().x / d);
  const int nz = (int)(box.size().z / d);
  for (int ly = 0; p.n < want; ++ly) {
    for (int iz = 0; iz < nz && p.n < want; ++iz) {
      for (int ix = 0; ix < nx && p.n < want; ++ix) {
        p.add(Vec3{box.lo.x + (0.5f + (float)ix) * d + r.nextSigned() * 0.1f * d,
                   box.lo.y + (0.5f + (float)ly) * d + r.nextSigned() * 0.1f * d,
                   box.lo.z + (0.5f + (float)iz) * d + r.nextSigned() * 0.1f * d},
              Vec3{0, 0, 0}, mat);
      }
    }
    if (ly > 64) break;
  }
  return p.n;
}

// Settles `count` water particles in a 32^3 cube and leaves the state in the globals.
SimVolume settle(int count, int steps, Vec3 gravity) {
  SimVolume v;
  v.build(Geometry::cube(32, 1.0f), kSlabDepth, kCellSize);
  g_solver.init();
  g_p.clear();
  fillBottom(g_p, v.box(), count, kWater, 0xA11CE);
  for (int s = 0; s < steps; ++s)
    g_solver.step(g_p, v, g_h, g_scratch, defaultMaterials(), gravity, kFixedDt);
  return v;
}

bool finite(float f) { return f == f && f < 1e30f && f > -1e30f; }

}  // namespace

TEST(solver_wall_fraction_is_a_half_at_the_wall_and_zero_at_h) {
  g_solver.init();
  // The whole hydrostatic result hinges on this: a particle sitting on a wall is missing
  // exactly half its neighbourhood, so the compensation must be 0.5 rest densities.
  CHECK_NEAR(g_solver.wallFraction(0.0f), 0.5f, 1e-3);
  CHECK_NEAR(g_solver.wallFraction(kSmoothRadius), 0.0f, 1e-6);
  CHECK(g_solver.wallFraction(kSmoothRadius * 2.0f) == 0.0f);
  // Monotonically decreasing.
  float prev = 1.0f;
  for (int i = 0; i <= 32; ++i) {
    const float f = g_solver.wallFraction((float)i * kSmoothRadius / 32.0f);
    CHECK(f <= prev + 1e-6f);
    prev = f;
  }
}

TEST(solver_rest_density_normalisation) {
  g_solver.init();
  // mass is chosen so a rest lattice has density exactly 1, which keeps epsilon and
  // s_corr in this project O(1) and independent of the kernel's normalisation constant.
  CHECK_NEAR(g_solver.restDensity(), 1.0f, 1e-6);
  CHECK(g_solver.mass() > 0.0f && g_solver.mass() < 100.0f);
}

TEST(solver_hydrostatic_rest) {
  const SimVolume v = settle(1500, 400, Vec3{0.0f, -kGravityMag, 0.0f});
  const Aabb& b = v.box();

  int outside = 0, moving = 0, bad = 0;
  double sumSpeed = 0.0, sumRho = 0.0, sumY = 0.0;
  for (int i = 0; i < g_p.n; ++i) {
    const Vec3 q = g_p.pos(i), vel = g_p.vel(i);
    if (!finite(q.x) || !finite(q.y) || !finite(q.z) || !finite(vel.x)) ++bad;
    if (!b.contains(q)) ++outside;
    const float sp = length(vel);
    sumSpeed += sp;
    if (sp > 1.0f) ++moving;
    sumY += q.y;
    sumRho += g_solver.densityAt(g_p, v, g_h, i);
  }
  const double meanSpeed = sumSpeed / g_p.n;
  const double meanRho = sumRho / g_p.n;
  // Centre of mass gives the column height: meanY == lo + H/2 for a settled column.
  const double fill = 2.0 * (sumY / g_p.n - b.lo.y);
  const double expected =
      (double)g_p.n * g_solver.mass() / (b.size().x * b.size().z);

  std::printf("       mean|v| %.4f  rho %.4f  fill %.2f (want %.2f)  moving %d/%d\n",
              meanSpeed, meanRho, fill, expected, moving, g_p.n);

  CHECK(bad == 0);                        // no NaN or runaway
  CHECK(outside == 0);                    // nothing escapes the container
  CHECK(meanSpeed < 0.5);                 // genuinely at rest, not shimmering
  CHECK(moving * 20 < g_p.n);             // under 5% still in motion
  CHECK(meanRho > 0.95 && meanRho < 1.06);  // incompressible to within ~5%
  CHECK(fill > expected * 0.90 && fill < expected * 1.05);  // right amount of volume
}

TEST(solver_column_density_is_uniform_with_depth) {
  // The bug this guards against: an under-compensated wall term let the fluid over-pack
  // against the floor by 1.5x while the measured density still read 1.0. A uniform
  // profile is the signature of a correct boundary.
  const SimVolume v = settle(1500, 400, Vec3{0.0f, -kGravityMag, 0.0f});
  const Aabb& b = v.box();

  const float band = 2.0f;
  int bands = 0;
  for (int k = 0; k < 4; ++k) {
    const float y0 = b.lo.y + (float)k * band;
    int cnt = 0;
    double rho = 0.0;
    for (int i = 0; i < g_p.n; ++i)
      if (g_p.y[i] >= y0 && g_p.y[i] < y0 + band) {
        ++cnt;
        rho += g_solver.densityAt(g_p, v, g_h, i);
      }
    if (cnt < 50) continue;
    ++bands;
    CHECK(rho / cnt > 0.95 && rho / cnt < 1.06);
  }
  CHECK(bands >= 3);  // the column really is several bands deep
}

TEST(solver_gravity_direction_is_respected) {
  // Tilt gravity onto +x and the fluid must pile against the +x wall instead of the floor.
  const SimVolume v = settle(1200, 400, Vec3{kGravityMag, 0.0f, 0.0f});
  double sumX = 0.0;
  for (int i = 0; i < g_p.n; ++i) sumX += g_p.x[i];
  const double meanX = sumX / g_p.n;
  std::printf("       gravity +x: mean x %.2f (box hi %.1f)\n", meanX, v.box().hi.x);
  CHECK(meanX > 8.0);  // clearly pushed into the +x half
  for (int i = 0; i < g_p.n; ++i) CHECK(v.box().contains(g_p.pos(i)));
}

TEST(solver_survives_a_violent_shake_without_leaking) {
  SimVolume v;
  v.build(Geometry::cube(32, 1.0f), kSlabDepth, kCellSize);
  g_solver.init();
  g_p.clear();
  fillBottom(g_p, v.box(), 1500, kWater, 99);

  // Slam gravity around at full strength in a different direction every 10 steps -- far
  // harsher than a hand shake, and the case where a solver typically vents particles
  // through a wall.
  Rng r(5);
  Vec3 g{0, -kGravityMag, 0};
  for (int s = 0; s < 400; ++s) {
    if (s % 10 == 0)
      g = normalize(Vec3{r.nextSigned(), r.nextSigned(), r.nextSigned()}) *
          (kGravityMag * 3.0f);
    g_solver.step(g_p, v, g_h, g_scratch, defaultMaterials(), g, kFixedDt);
  }

  int outside = 0, bad = 0;
  for (int i = 0; i < g_p.n; ++i) {
    const Vec3 q = g_p.pos(i);
    if (!finite(q.x) || !finite(q.y) || !finite(q.z)) ++bad;
    if (!v.box().contains(q)) ++outside;
  }
  CHECK(bad == 0);
  CHECK(outside == 0);
}

TEST(solver_thin_slab_behaves_like_a_2d_tank) {
  // Single-panel mode: same 3D code path, 2 cells deep.
  SimVolume v;
  CHECK(v.build(Geometry::slab(32, 32, 1.0f), kSlabDepth, kCellSize));
  g_solver.init();
  g_p.clear();
  // A quarter of capacity. A slab tolerates a much smaller fraction of its nominal
  // capacity than a cube does: the wall compensation is 0.5 right at each face and ~0 at
  // mid-depth, so the z distribution ends up non-uniform and over-filling churns.
  // Measured at depth 1.5h: 400 particles settle, 700 do not. Tightening this is deferred
  // until single-panel mode is actually built -- it is not part of Milestone 1.
  const int want = g_solver.capacity(v) / 4;
  fillBottom(g_p, v.box(), want, kWater, 1234);
  CHECK(g_p.n > 100);
  for (int s = 0; s < 300; ++s)
    g_solver.step(g_p, v, g_h, g_scratch, defaultMaterials(),
                  Vec3{0.0f, -kGravityMag, 0.0f}, kFixedDt);

  int outside = 0;
  double sumSpeed = 0.0;
  for (int i = 0; i < g_p.n; ++i) {
    if (!v.box().contains(g_p.pos(i))) ++outside;
    sumSpeed += length(g_p.vel(i));
  }
  CHECK(outside == 0);
  CHECK(sumSpeed / g_p.n < 1.0);
  // Depth stays within the 2-unit slab.
  for (int i = 0; i < g_p.n; ++i)
    CHECK(g_p.z[i] >= v.box().lo.z && g_p.z[i] <= v.box().hi.z);
}

TEST(solver_is_deterministic) {
  const SimVolume v = settle(800, 200, Vec3{0.0f, -kGravityMag, 0.0f});
  const size_t bytes = sizeof(float) * (size_t)g_p.n;
  uint64_t a = fnv1a(g_p.x, bytes);
  a = fnv1a(g_p.y, bytes, a);
  a = fnv1a(g_p.vx, bytes, a);

  settle(800, 200, Vec3{0.0f, -kGravityMag, 0.0f});
  uint64_t b = fnv1a(g_p.x, bytes);
  b = fnv1a(g_p.y, bytes, b);
  b = fnv1a(g_p.vx, bytes, b);

  CHECK(a == b);
  std::printf("       state hash %016llx\n", (unsigned long long)a);
  (void)v;
}

TEST(solver_empty_and_single_particle_are_safe) {
  SimVolume v;
  v.build(Geometry::cube(32, 1.0f), kSlabDepth, kCellSize);
  g_solver.init();

  g_p.clear();
  for (int s = 0; s < 5; ++s)
    g_solver.step(g_p, v, g_h, g_scratch, defaultMaterials(),
                  Vec3{0.0f, -kGravityMag, 0.0f}, kFixedDt);

  g_p.clear();
  g_p.add(Vec3{0, 0, 0}, Vec3{0, 0, 0}, kWater);
  for (int s = 0; s < 200; ++s)
    g_solver.step(g_p, v, g_h, g_scratch, defaultMaterials(),
                  Vec3{0.0f, -kGravityMag, 0.0f}, kFixedDt);
  // A lone particle just falls to the floor and stops there.
  CHECK(g_p.y[0] < -14.0f);
  CHECK(v.box().contains(g_p.pos(0)));
}
