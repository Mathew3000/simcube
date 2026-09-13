#include "check.h"
#include "partsim/Geometry.h"
#include "partsim/InkField.h"
#include "partsim/Rng.h"
#include "partsim/SimVolume.h"

using namespace partsim;

namespace {

// Large enough that these must not be stack locals.
InkField g_a;
InkField g_b;

SimVolume makeVolume() {
  SimVolume v;
  const bool ok = v.build(Geometry::cube(32, 1.0f), 2.0f, kCellSize);
  CHECK(ok);
  return v;
}

int totalDye(const InkField& f, int channel) {
  int t = 0;
  const uint8_t* c = f.channel(channel);
  for (int i = 0; i < f.cellCount(); ++i) t += c[i];
  return t;
}

// Centre of mass along one axis, in cells, over every channel. -1 when the field is empty.
float centroid(const InkField& f, int axis) {
  double sum = 0.0, wsum = 0.0;
  for (int z = 0; z < kInkDim; ++z)
    for (int y = 0; y < kInkDim; ++y)
      for (int x = 0; x < kInkDim; ++x) {
        double w = 0.0;
        for (int c = 0; c < kInkChannels; ++c) w += f.at(c, x, y, z);
        const int coord = (axis == 0) ? x : (axis == 1 ? y : z);
        sum += w * (double)coord;
        wsum += w;
      }
  return wsum > 0.0 ? (float)(sum / wsum) : -1.0f;
}

uint64_t hashField(const InkField& f) {
  uint64_t h = 1469598103934665603ull;
  for (int c = 0; c < kInkChannels; ++c) h = fnv1a(f.channel(c), (size_t)f.cellCount(), h);
  return h;
}

// The container runs -16..+16 on every axis, so its centre is the ORIGIN. Injecting at
// (16,16,16) puts the plume in a corner, where boundary behaviour dominates whatever the test
// meant to measure.
const Vec3 kCentre{0.0f, 0.0f, 0.0f};

}  // namespace

TEST(ink_empty_field_stays_exactly_empty) {
  const SimVolume v = makeVolume();
  CHECK(g_a.init(v));
  CHECK(g_a.empty());

  // A clear volume must cost nothing and produce nothing. This also pins the early-out: if the
  // peak tracking ever stopped working, a water-only scene would start paying for the whole grid.
  for (int i = 0; i < 20; ++i) g_a.step(Vec3{0.0f, -1.0f, 0.0f}, Vec3{0, 0, 0}, 50);

  for (int c = 0; c < kInkChannels; ++c) CHECK(totalDye(g_a, c) == 0);
  CHECK(g_a.empty());
}

TEST(ink_injection_touches_only_its_own_channel) {
  const SimVolume v = makeVolume();
  CHECK(g_a.init(v));
  g_a.inject(kCentre, 4.0f, 0, 200);

  CHECK(totalDye(g_a, 0) > 0);
  CHECK(!g_a.empty());
#if PARTSIM_INK_CHANNELS > 1
  // Dye is a MASS per channel, not a hue: injecting red must not imply anything about blue.
  CHECK(totalDye(g_a, 1) == 0);
#endif
}

TEST(ink_injection_saturates_rather_than_wrapping) {
  const SimVolume v = makeVolume();
  CHECK(g_a.init(v));
  // Four pours into the same place. A wrap would punch a transparent hole in the densest part of
  // the plume, which reads as a bug in the renderer rather than in the field.
  for (int i = 0; i < 4; ++i) g_a.inject(kCentre, 4.0f, 0, 200);
  CHECK(g_a.peak() == 255);
  const uint8_t* c = g_a.channel(0);
  for (int i = 0; i < g_a.cellCount(); ++i) CHECK(c[i] <= 255);
}

TEST(ink_settles_along_gravity_on_every_axis) {
  const SimVolume v = makeVolume();
  // Six directions, because a sign error on one axis is invisible while the cube sits upright.
  const Vec3 dirs[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  const int axisOf[6] = {0, 0, 1, 1, 2, 2};
  const float signOf[6] = {1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f};

  for (int d = 0; d < 6; ++d) {
    CHECK(g_a.init(v));
    g_a.inject(kCentre, 6.0f, 0, 220);
    const float before = centroid(g_a, axisOf[d]);

    for (int i = 0; i < 10; ++i) g_a.step(dirs[d] * 9.81f, Vec3{0, 0, 0}, 50);

    const float after = centroid(g_a, axisOf[d]);
    CHECK(after >= 0.0f);
    // Moved the right way, and by a visible amount rather than a rounding wobble.
    CHECK((after - before) * signOf[d] > 0.3f);
  }
}

TEST(ink_active_box_matches_a_forced_full_update) {
  const SimVolume v = makeVolume();
  CHECK(g_a.init(v));
  CHECK(g_b.init(v));

  Rng ra(0x1234), rb(0x1234);
  g_a.seedFlow(ra);
  g_b.seedFlow(rb);

  g_a.inject(kCentre, 5.0f, 0, 240);
  g_b.inject(kCentre, 5.0f, 0, 240);
#if PARTSIM_INK_CHANNELS > 1
  const Vec3 off{4.0f, 2.0f, -2.0f};
  g_a.inject(off, 4.0f, 1, 180);
  g_b.inject(off, 4.0f, 1, 180);
#endif
  g_a.spawnVortonPair(kCentre, Vec3{0.0f, 0.0f, 1.0f}, 8.0f, 900, 400);
  g_b.spawnVortonPair(kCentre, Vec3{0.0f, 0.0f, 1.0f}, 8.0f, 900, 400);

  g_b.setForceFullUpdate(true);

  // The active box is an optimisation, so it must be invisible. Anything else means a tendril is
  // being silently clipped at the boundary -- which looks like diffusion, not like a bug, and so
  // would never be found by eye.
  for (int s = 0; s < 40; ++s) {
    g_a.step(Vec3{0.0f, -9.81f, 0.0f}, Vec3{0.4f, 0.0f, -0.2f}, 50);
    g_b.step(Vec3{0.0f, -9.81f, 0.0f}, Vec3{0.4f, 0.0f, -0.2f}, 50);
    for (int c = 0; c < kInkChannels; ++c) {
      const uint8_t* pa = g_a.channel(c);
      const uint8_t* pb = g_b.channel(c);
      for (int i = 0; i < g_a.cellCount(); ++i) {
        if (pa[i] != pb[i]) {
          std::printf("    step %d channel %d cell %d: box %d, full %d\n", s, c, i, pa[i], pb[i]);
          CHECK(pa[i] == pb[i]);
          return;
        }
      }
    }
  }
}

TEST(ink_is_deterministic_for_the_same_seed_and_motion) {
  const SimVolume v = makeVolume();
  uint64_t first = 0;
  // Twice through, from scratch. The field is part of the deterministic path: the master and a
  // display node must agree byte for byte about what they are drawing.
  for (int pass = 0; pass < 2; ++pass) {
    CHECK(g_a.init(v));
    Rng r(0xC0FFEE);
    g_a.seedFlow(r);
    g_a.inject(kCentre, 5.0f, 0, 200);
    g_a.spawnVortonPair(kCentre, Vec3{1.0f, 0.0f, 0.0f}, 7.0f, 800, 300);
    for (int i = 0; i < 30; ++i)
      g_a.step(Vec3{0.2f, -9.7f, 0.1f}, Vec3{0.3f, 0.1f, 0.0f}, 50);
    const uint64_t h = hashField(g_a);
    if (pass == 0) first = h; else CHECK(h == first);
  }
}

TEST(ink_stays_inside_the_grid_under_extreme_flow) {
  const SimVolume v = makeVolume();
  CHECK(g_a.init(v));
  Rng r(0x5EED);
  g_a.seedFlow(r);
  g_a.inject(kCentre, 6.0f, 0, 255);

  // A deliberately absurd vorton, to prove the CFL clamp and the trilinear index clamp hold. If
  // either is wrong this reads out of bounds, which the sanitizer build turns into a failure.
  g_a.spawnVortonPair(kCentre, Vec3{0.0f, 1.0f, 0.0f}, 3.0f, 30000, 5000);
  const int before = totalDye(g_a, 0);

  for (int i = 0; i < 200; ++i) g_a.step(Vec3{0.0f, -9.81f, 0.0f}, Vec3{9.0f, -9.0f, 9.0f}, 50);

  // Advection may lose mass to the boundary and to rounding; it must never CREATE it.
  CHECK(totalDye(g_a, 0) <= before);
  const IVec3 lo = g_a.activeLo(), hi = g_a.activeHi();
  if (hi.x >= lo.x) {
    CHECK(lo.x >= 0 && hi.x < kInkDim);
    CHECK(lo.y >= 0 && hi.y < kInkDim);
    CHECK(lo.z >= 0 && hi.z < kInkDim);
  }
}

#if PARTSIM_INK_CHANNELS > 1
TEST(ink_channels_advect_independently) {
  const SimVolume v = makeVolume();
  CHECK(g_a.init(v));
  // Red low, blue high, both descending. They must stay distinguishable while they travel --
  // if the channels were coupled, this would resolve to one colour immediately and the whole
  // point of carrying two masses would be lost.
  g_a.inject(Vec3{0.0f, -6.0f, 0.0f}, 4.0f, 0, 220);
  g_a.inject(Vec3{0.0f, 6.0f, 0.0f}, 4.0f, 1, 220);

  const int red0 = totalDye(g_a, 0), blue0 = totalDye(g_a, 1);
  CHECK(red0 > 0 && blue0 > 0);

  for (int i = 0; i < 8; ++i) g_a.step(Vec3{0.0f, -9.81f, 0.0f}, Vec3{0, 0, 0}, 50);

  // Both still present, and the blue is still above the red.
  CHECK(totalDye(g_a, 0) > 0);
  CHECK(totalDye(g_a, 1) > 0);

  double redY = 0.0, redW = 0.0, blueY = 0.0, blueW = 0.0;
  for (int z = 0; z < kInkDim; ++z)
    for (int y = 0; y < kInkDim; ++y)
      for (int x = 0; x < kInkDim; ++x) {
        const double a = g_a.at(0, x, y, z), b = g_a.at(1, x, y, z);
        redY += a * y; redW += a;
        blueY += b * y; blueW += b;
      }
  CHECK(redW > 0.0 && blueW > 0.0);
  CHECK(blueY / blueW > redY / redW);
}
#endif
