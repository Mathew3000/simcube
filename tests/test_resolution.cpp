// Resolution must be a DISPLAY concern only.
//
// 32 texels at pitch 1.0 and 64 texels at pitch 0.5 describe the same 32-unit world. If that is
// true, the physics cannot tell them apart -- and it needs to be true, because the plan is to run
// the same firmware on both panel sizes and the golden STATE hash is a shared reference. Only the
// pixel hash may differ.
//
// This is the guard that stops resolution leaking into the solver. It would have caught the
// kSlabDepth unit bug, where a world-units depth was multiplied by the pitch and quietly halved
// the container at pitch 0.5.
#include "check.h"
#include "partsim/Renderer.h"
#include "partsim/Solver.h"

using namespace partsim;

namespace {

Particles g_p;
SpatialHash g_h;
Solver g_solv;
FieldGrid g_f;
float g_scratch[kMaxParticles];

// The same body of fluid, described in world coordinates so it is identical at either pitch.
void fill() {
  g_p.clear();
  for (int i = 0; i < 900; ++i) {
    const int x = i % 12, y = (i / 12) % 10, z = i / 120;
    g_p.add(Vec3{-9.0f + (float)x * 1.5f, -14.0f + (float)y * 1.5f, -6.0f + (float)z * 1.5f},
            Vec3{0.0f, 0.0f, 0.0f}, (i % 5 == 0) ? kSand : kWater);
  }
}

uint32_t stateOf(const Particles& p) {
  uint64_t h = 1469598103934665603ull;
  const size_t n = sizeof(float) * (size_t)p.n;
  h = fnv1a(p.x, n, h); h = fnv1a(p.y, n, h); h = fnv1a(p.z, n, h);
  h = fnv1a(p.vx, n, h); h = fnv1a(p.vy, n, h); h = fnv1a(p.vz, n, h);
  return (uint32_t)(h ^ (h >> 32));
}

// Runs a scripted sequence with tilt, so the container walls are exercised on every axis rather
// than only the floor.
uint32_t runPhysics(int res, float pitch, SimVolume& v) {
  const Geometry g = Geometry::cube(res, pitch);
  CHECK(v.build(g, kSlabDepth, kCellSize));
  g_solv.init();
  fill();
  CHECK(g_f.init(v));
  Rng rng(0x5EEDu);
  for (int s = 0; s < 300; ++s) {
    const float t = (float)s * 0.017f;
    const Vec3 grav =
        normalize(Vec3{fsin(t) * 0.6f, -1.0f, fcos(t * 0.73f) * 0.6f}) * kGravityMag;
    g_solv.step(g_p, v, g_h, g_scratch, defaultMaterials(), grav, kFixedDt);
    g_f.step(v, grav, Vec3{0.0f, 0.0f, 0.0f}, nullptr, 0, kFixedDt, rng);
  }
  return stateOf(g_p);
}

}  // namespace

TEST(resolution_describes_the_same_container) {
  SimVolume a, b;
  const Geometry ga = Geometry::cube(32, 1.0f);
  const Geometry gb = Geometry::cube(64, 0.5f);
  CHECK(a.build(ga, kSlabDepth, kCellSize));
  CHECK(b.build(gb, kSlabDepth, kCellSize));

  // Bit-exact, not merely close: 0.5*32*1.0 and 0.5*64*0.5 are both exactly 16.0, and the inward
  // push is in world units, so there is no rounding anywhere to be tolerant of.
  CHECK(a.box().lo.x == b.box().lo.x);
  CHECK(a.box().hi.x == b.box().hi.x);
  CHECK(a.box().lo.y == b.box().lo.y);
  CHECK(a.box().hi.y == b.box().hi.y);
  CHECK(a.box().lo.z == b.box().lo.z);
  CHECK(a.box().hi.z == b.box().hi.z);
  // ...and therefore the same neighbour grid, which is derived from the box and the cell size.
  CHECK(a.dim().x == b.dim().x);
  CHECK(a.dim().y == b.dim().y);
  CHECK(a.dim().z == b.dim().z);
  CHECK(a.cellCount() == b.cellCount());
  std::printf("       both give a %.1f-unit box on a %dx%dx%d grid\n", a.box().size().x,
              a.dim().x, a.dim().y, a.dim().z);
}

TEST(resolution_does_not_reach_the_physics) {
  SimVolume va, vb;
  const uint32_t coarse = runPhysics(32, 1.0f, va);
  const uint32_t fine = runPhysics(64, 0.5f, vb);
  std::printf("       state hash  32x32 @1.0 = %08x   64x64 @0.5 = %08x\n", coarse, fine);
  CHECK(coarse == fine);
  CHECK(coarse != 0u);
}

TEST(resolution_changes_only_the_picture) {
  // The complement of the above: the same fluid must NOT produce the same pixels, or the finer
  // panel is not actually resolving anything extra.
  static Renderer r;
  int lit[2] = {0, 0};
  const int res[2] = {32, 64};
  const float pitch[2] = {1.0f, 0.5f};
  for (int k = 0; k < 2; ++k) {
    const Geometry g = Geometry::cube(res[k], pitch[k]);
    CHECK(r.init(g));
    fill();
    r.clear();
    r.splat(g_p, g);
    for (int j = 0; j < res[k]; ++j)
      for (int i = 0; i < res[k]; ++i)
        if (r.accumAt(4, i, j, res[k], kChWater) > 0) ++lit[k];
  }
  std::printf("       bottom face lit texels: %d at 32x32, %d at 64x64 (%.2fx)\n", lit[0], lit[1],
              (float)lit[1] / (float)lit[0]);
  CHECK(lit[0] > 0);
  CHECK(lit[1] > lit[0] * 2);  // genuinely more detail, not a scaled copy
}
