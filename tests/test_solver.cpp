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

// Settles water in a 32^3 cube and leaves the state in the globals.
//
// `refCount` is a FILL LEVEL expressed at the reference rest spacing, not a literal particle
// count -- particlesForFill rescales it by 1/d^3. Passing the literal through would make every
// fixture here a function of kRestSpacing: at spacing 3.0 a 32-unit box holds ~1213 particles at
// rest, so the 1500 these tests ask for would be an OVERFULL box that cannot settle by
// construction, and "solver does not reach hydrostatic rest" would look like a physics
// regression instead of a stale constant.
SimVolume settle(int refCount, int steps, Vec3 gravity);

// Particles needed to pool a 32^3 box to `depth` world units: the floor area times the depth,
// divided by the volume one particle occupies at rest. Stated as a DEPTH because that is what the
// density tests actually need -- bulk density is only meaningful with an interior, and "interior"
// means further than one smoothing radius from a free surface. kSmoothRadius is 2*kRestSpacing,
// so a fixture written as a particle count silently stops having an interior when the particles
// are coarsened: at spacing 3.0 the old settle(1500) pools 4.6 units against an h of 6.0, which is
// all surface and legitimately reads 0.94 rather than 1.00.
// Depth is capped at three quarters of the box. Four smoothing radii is 24 units at spacing 3.0
// and fits, but h is 2*d, so at spacing 4.0 the same expression asks for 32 units -- the entire
// container, with no free surface at all, and the density bands then read the compression of a
// sealed column. Identical to the uncapped value at every spacing at or below 3.0.
float cappedDepth(const Aabb& box, float want) { return pmin(want, 0.75f * box.size().y); }

int countForDepth(const Aabb& box, float depth) {
  const float d = kRestSpacing;
  return (int)(box.size().x * box.size().z * depth / (d * d * d));
}

SimVolume settleToDepth(float depth, int steps, Vec3 gravity) {
  SimVolume v;
  v.build(Geometry::cube(32, 1.0f), kSlabDepth, kCellSize);
  g_solver.init();
  g_p.clear();
  fillBottom(g_p, v.box(), countForDepth(v.box(), cappedDepth(v.box(), depth)), kWater, 0xA11CE);
  for (int s = 0; s < steps; ++s)
    g_solver.step(g_p, v, g_h, g_scratch, defaultMaterials(), gravity, kFixedDt);
  return v;
}

SimVolume settle(int refCount, int steps, Vec3 gravity) {
  SimVolume v;
  v.build(Geometry::cube(32, 1.0f), kSlabDepth, kCellSize);
  g_solver.init();
  g_p.clear();
  fillBottom(g_p, v.box(), particlesForFill(refCount), kWater, 0xA11CE);
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
  // Four smoothing radii deep, so most of the fluid is interior rather than free surface.
  // 2500 steps. A pool four smoothing radii deep has far more momentum to shed than the shallow
  // one this fixture used to build, and the approach to rest is long and shallow: measured at
  // spacing 3.0, mean|v| 1.09 at 400 steps, 0.52 at 900, 0.000 at 2500.
  //
  // It sat at 1500 for a while and that was luck. At 1500 the reading is anywhere between 0.05 and
  // 0.85 depending on the trajectory -- a last-ulp change (hoisting one reciprocal out of a loop)
  // moved it from 0.053 to 0.568 without touching the physics. A threshold that a rounding change
  // can cross is measuring chaos, not convergence. 2500 is past the knee for every variant tried.
  const SimVolume v = settleToDepth(4.0f * kSmoothRadius, settleSteps(2500), Vec3{0.0f, -kGravityMag, 0.0f});
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
  const SimVolume v = settleToDepth(4.0f * kSmoothRadius, settleSteps(2500), Vec3{0.0f, -kGravityMag, 0.0f});
  const Aabb& b = v.box();
  const float depth = cappedDepth(b, 4.0f * kSmoothRadius);

  // One band per smoothing radius, and only the bands BELOW the top one -- the surface band is
  // free surface, where a density below rest is correct physics rather than a boundary bug.
  const float band = kSmoothRadius;
  int bands = 0;
  double lo_rho = 1e9, hi_rho = -1e9;
  for (int k = 0; k < (int)(depth / band) - 1; ++k) {
    const float y0 = b.lo.y + (float)k * band;
    int cnt = 0;
    double rho = 0.0;
    for (int i = 0; i < g_p.n; ++i)
      if (g_p.y[i] >= y0 && g_p.y[i] < y0 + band) {
        ++cnt;
        rho += g_solver.densityAt(g_p, v, g_h, i);
      }
    if (cnt < 20) continue;
    ++bands;
    const double band_rho = rho / cnt;
    std::printf("       band %d: %d particles, rho %.4f\n", k, cnt, band_rho);
    // A gross bound only. The property this test is named for is UNIFORMITY, asserted on the
    // spread below -- a single band's absolute value carries legitimate hydrostatic compression
    // that grows as the particles coarsen, because the same column is borne by fewer layers.
    // Measured at the floor: 1.0495 at spacing 3.0, 1.0675 at 4.0. Pinning each band under 1.06
    // was pinning the test to one spacing, while the bug it guards against -- an under-compensated
    // wall term over-packing the floor by 1.5x -- is nowhere near that line.
    CHECK(band_rho > 0.95 && band_rho < 1.10);
    if (band_rho < lo_rho) lo_rho = band_rho;
    if (band_rho > hi_rho) hi_rho = band_rho;
  }
  // How many interior bands exist is a function of the spacing: the band is one smoothing radius
  // and the pool is capped at three quarters of the box, so a coarse build simply has fewer. At
  // spacing 3.0 that is 3; at 4.0 the box only holds 2. Requiring 3 unconditionally asserted a
  // property of the container, not of the solver.
  // Uniform with depth: the floor band and the topmost interior band must agree. Measured spread
  // is 3.9% at spacing 3.0 and 5.4% at 4.0; a boundary bug shows up as tens of percent.
  std::printf("       rho spread %.4f .. %.4f (%.1f%%)\n", lo_rho, hi_rho,
              100.0 * (hi_rho - lo_rho));
  CHECK(hi_rho - lo_rho < 0.08);
  const int expectBands = imax(2, (int)(depth / band) - 1);
  std::printf("       %d interior bands of %.1f units (depth %.1f)\n", bands, (double)band,
              (double)depth);
  CHECK(bands >= imin(3, expectBands));
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
  fillBottom(g_p, v.box(), particlesForFill(1500), kWater, 99);

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
  CHECK(g_p.n > 25);  // scales with 1/d^3; the point is that the slab got a real fill
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

#if PARTSIM_NEIGHBOUR_CACHE
TEST(neighbour_cache_agrees_with_a_live_gather) {
  // The cache must contain exactly what a scan would find, in the same order -- that ordering is
  // what the cross-target determinism rests on, so "same set" is not good enough.
  const SimVolume v = settleToDepth(2.0f * kSmoothRadius, 200, Vec3{0.0f, -kGravityMag, 0.0f});
  g_h.build(v, g_p, g_scratch);

  const float r2Max = kNeighbourRadius * kNeighbourRadius;
  long long listed = 0;
  int checked = 0;
  for (int i = 0; i < g_p.n; ++i) {
    const Vec3 pi = g_p.pred(i);
    int k = 0;
    bool ok = true;
    forEachNeighbour(v, g_h, pi, [&](int j) {
      if (j == i) return;
      if (length2(g_p.pred(j) - pi) >= r2Max) return;
      if (k >= g_h.neighbourCount(i)) { ok = false; return; }
      if ((int)g_h.neighbours(i)[k] != j) ok = false;
      ++k;
    });
    CHECK(ok);
    CHECK(k == g_h.neighbourCount(i));
    listed += g_h.neighbourCount(i);
    ++checked;
  }
  // No particle may have lost neighbours to the cap, or the lists above would not have matched.
  CHECK(g_h.truncated() == 0);
  std::printf("       %d particles, mean %.1f cached neighbours, cap %d, truncated %d\n", checked,
              (double)listed / checked, kMaxNeighbours, g_h.truncated());
}

TEST(neighbour_cache_margin_covers_what_the_kernel_needs) {
  // The margin exists so the frozen list still holds every particle the correction pass can pull
  // inside the smoothing radius. If it ever drops below 1.0 the cache is missing neighbours that
  // are in range at build time, which is a silently wrong density rather than a slow one.
  CHECK(kNeighbourRadius >= kSmoothRadius);
  CHECK(kMaxNeighbours <= 255);
}
#endif

// --- the Parallel contract ----------------------------------------------------------------
namespace {
// Splits [0,n) into `parts` chunks and runs them on this thread in REVERSE order.
//
// Reverse is the whole design, and the first version of this test got it wrong. Running chunks
// back to back in ascending order reproduces the serial order exactly, so a chunk boundary never
// changes what an element can see -- verified by injecting a genuine order dependency into the
// density pass and watching all four split counts still agree. That test proved nothing.
//
// Running the last chunk first means element i is computed before element i-1, which is precisely
// what a second core would do to some pair of elements. A pass that is truly split-independent
// does not care; a Gauss-Seidel pass handed here changes its answer immediately. No threads, no
// scheduler, no flakiness -- and it fails on the injected dependency, which is how this version is
// known to work.
class ChunkParallel : public Parallel {
 public:
  explicit ChunkParallel(int parts) : parts_(parts) {}
  void forRange(int n, void* ctx, RangeFn fn) override {
    if (n <= 0) return;
    const int step = (n + parts_ - 1) / parts_;
    int starts[64];
    int count = 0;
    for (int b = 0; b < n && count < 64; b += step) starts[count++] = b;
    for (int k = count - 1; k >= 0; --k) fn(ctx, starts[k], imin(n, starts[k] + step));
  }
  int workers() const override { return parts_; }

 private:
  int parts_;
};
}  // namespace

TEST(parallel_split_does_not_change_the_answer) {
  // Everything handed to Parallel::forRange must be split-independent. This is the guard on that:
  // the same settle, run with 1, 2, 3 and 7 chunks, must produce bit-identical state. An uneven
  // count (3, 7) is deliberate -- a bug that only shows at a particular boundary hides behind
  // powers of two.
  uint32_t reference = 0;
  const int kSplits[] = {1, 2, 3, 7};
  for (int parts : kSplits) {
    ChunkParallel par(parts);
    g_solver.init();
    g_solver.setParallel(&par);
    SimVolume v;
    v.build(Geometry::cube(32, 1.0f), kSlabDepth, kCellSize);
    g_p.clear();
    fillBottom(g_p, v.box(), particlesForFill(1500), kWater, 0xA11CE);
    for (int s = 0; s < 120; ++s)
      g_solver.step(g_p, v, g_h, g_scratch, defaultMaterials(),
                    Vec3{0.0f, -kGravityMag, 0.0f}, kFixedDt);
    // Hash positions and velocities directly: this is about the particle state, not a Simulation.
    uint64_t h64 = fnv1a(g_p.x, (size_t)g_p.n * sizeof(float));
    h64 = fnv1a(g_p.y, (size_t)g_p.n * sizeof(float), h64);
    h64 = fnv1a(g_p.z, (size_t)g_p.n * sizeof(float), h64);
    h64 = fnv1a(g_p.vx, (size_t)g_p.n * sizeof(float), h64);
    h64 = fnv1a(g_p.vy, (size_t)g_p.n * sizeof(float), h64);
    h64 = fnv1a(g_p.vz, (size_t)g_p.n * sizeof(float), h64);
    const uint32_t h = (uint32_t)(h64 ^ (h64 >> 32));
    std::printf("       %d chunk(s): %d particles, state %08x\n", parts, g_p.n, h);
    if (parts == 1) reference = h;
    CHECK(h == reference);
  }
  g_solver.setParallel(nullptr);  // back to serial for every test after this one
}
