// The single most valuable early test in the project: a broken neighbour search does not
// look like a search bug, it looks like the solver inexplicably exploding several steps
// later. So we check the grid gather against brute-force O(N^2) exactly.
#include <algorithm>
#include <array>
#include <vector>

#include "check.h"
#include "partsim/Rng.h"
#include "partsim/SpatialHash.h"

using namespace partsim;

namespace {

// Far too big for the stack.
Particles g_p;
SpatialHash g_h;
float g_scratch[kMaxParticles];

void fillRandom(Particles& p, const Aabb& box, int count, uint32_t seed) {
  Rng r(seed);
  p.clear();
  const Vec3 s = box.size();
  for (int i = 0; i < count; ++i) {
    const Vec3 q{box.lo.x + r.nextFloat() * s.x, box.lo.y + r.nextFloat() * s.y,
                 box.lo.z + r.nextFloat() * s.z};
    p.add(q, Vec3{0, 0, 0}, (uint8_t)(i & 1));
  }
}

std::vector<int> gridNeighbours(const SimVolume& v, const SpatialHash& h,
                                const Particles& p, int i, float radius) {
  std::vector<int> out;
  const Vec3 pi = p.pred(i);
  const float r2 = radius * radius;
  forEachNeighbour(v, h, pi, [&](int j) {
    if (j == i) return;
    if (length2(p.pred(j) - pi) <= r2) out.push_back(j);
  });
  std::sort(out.begin(), out.end());
  return out;
}

std::vector<int> bruteNeighbours(const Particles& p, int i, float radius) {
  std::vector<int> out;
  const Vec3 pi = p.pred(i);
  const float r2 = radius * radius;
  for (int j = 0; j < p.n; ++j) {
    if (j == i) continue;
    if (length2(p.pred(j) - pi) <= r2) out.push_back(j);
  }
  return out;  // already ascending
}

}  // namespace

TEST(hash_matches_brute_force_in_cube) {
  SimVolume v;
  CHECK(v.build(Geometry::cube(32, 1.0f), 2.0f, kCellSize));
  fillRandom(g_p, v.box(), 2000, 0xBEEF);
  CHECK(g_h.build(v, g_p, g_scratch));

  int mismatches = 0, pairs = 0;
  for (int i = 0; i < g_p.n; ++i) {
    const std::vector<int> a = gridNeighbours(v, g_h, g_p, i, kSmoothRadius);
    const std::vector<int> b = bruteNeighbours(g_p, i, kSmoothRadius);
    pairs += (int)b.size();
    if (a != b) ++mismatches;
  }
  CHECK(mismatches == 0);
  CHECK(pairs > 0);
  std::printf("       %d particles, %d neighbour pairs, %.1f avg neighbours\n", g_p.n,
              pairs, (double)pairs / (double)g_p.n);
}

TEST(hash_matches_brute_force_in_thin_slab) {
  // The slab is one cell deep in z, so the 27-cell walk degenerates to 9 cells. This is
  // exactly where an off-by-one in the clamping would hide.
  SimVolume v;
  CHECK(v.build(Geometry::slab(32, 32, 1.0f), 2.0f, kCellSize));
  fillRandom(g_p, v.box(), 1500, 0xC0DE);
  CHECK(g_h.build(v, g_p, g_scratch));

  int mismatches = 0;
  for (int i = 0; i < g_p.n; ++i) {
    if (gridNeighbours(v, g_h, g_p, i, kSmoothRadius) !=
        bruteNeighbours(g_p, i, kSmoothRadius))
      ++mismatches;
  }
  CHECK(mismatches == 0);
}

TEST(hash_handles_clustered_and_coincident_points) {
  // Degenerate input: everything piled into two corners, many exactly coincident.
  SimVolume v;
  CHECK(v.build(Geometry::cube(32, 1.0f), 2.0f, kCellSize));
  g_p.clear();
  for (int i = 0; i < 300; ++i) g_p.add(Vec3{-16.0f, -16.0f, -16.0f}, Vec3{0, 0, 0}, kWater);
  for (int i = 0; i < 300; ++i) g_p.add(Vec3{16.0f, 16.0f, 16.0f}, Vec3{0, 0, 0}, kWater);
  CHECK(g_h.build(v, g_p, g_scratch));

  int mismatches = 0;
  for (int i = 0; i < g_p.n; ++i) {
    if (gridNeighbours(v, g_h, g_p, i, kSmoothRadius) !=
        bruteNeighbours(g_p, i, kSmoothRadius))
      ++mismatches;
  }
  CHECK(mismatches == 0);
}

TEST(hash_partitions_every_particle_exactly_once) {
  SimVolume v;
  CHECK(v.build(Geometry::cube(32, 1.0f), 2.0f, kCellSize));
  fillRandom(g_p, v.box(), 3000, 7);
  CHECK(g_h.build(v, g_p, g_scratch));

  int total = 0;
  for (int c = 0; c < g_h.cellCount(); ++c) {
    const int b = g_h.cellBegin(c), e = g_h.cellEnd(c);
    CHECK(b <= e);
    total += e - b;
    // Every particle in a cell's range must actually belong to that cell -- this is what
    // makes the contiguous x-run shortcut in forEachNeighbour legal.
    for (int i = b; i < e; ++i) CHECK(v.cellIndexOf(g_p.pred(i)) == c);
  }
  CHECK(total == g_p.n);
  CHECK(g_h.cellBegin(0) == 0);
  CHECK(g_h.cellEnd(g_h.cellCount() - 1) == g_p.n);
}

TEST(hash_reorder_preserves_the_particle_set) {
  SimVolume v;
  CHECK(v.build(Geometry::cube(32, 1.0f), 2.0f, kCellSize));
  fillRandom(g_p, v.box(), 1000, 4242);

  // Give velocities a distinguishable tag so a mis-permuted array is detectable, then
  // snapshot every particle as a (pos, vel, material) tuple.
  std::vector<std::array<float, 7>> before;
  for (int i = 0; i < g_p.n; ++i) {
    g_p.vx[i] = (float)i;
    before.push_back({g_p.x[i], g_p.y[i], g_p.z[i], g_p.vx[i], g_p.vy[i], g_p.vz[i],
                      (float)g_p.mat[i]});
  }

  CHECK(g_h.build(v, g_p, g_scratch));

  std::vector<std::array<float, 7>> after;
  for (int i = 0; i < g_p.n; ++i)
    after.push_back({g_p.x[i], g_p.y[i], g_p.z[i], g_p.vx[i], g_p.vy[i], g_p.vz[i],
                     (float)g_p.mat[i]});

  std::sort(before.begin(), before.end());
  std::sort(after.begin(), after.end());
  CHECK(before == after);  // same set, permuted -- all 11 arrays moved together
}

TEST(hash_is_deterministic_across_rebuilds) {
  SimVolume v;
  CHECK(v.build(Geometry::cube(32, 1.0f), 2.0f, kCellSize));

  fillRandom(g_p, v.box(), 800, 31337);
  CHECK(g_h.build(v, g_p, g_scratch));
  const uint64_t h1 = fnv1a(g_p.x, sizeof(float) * (size_t)g_p.n,
                            fnv1a(g_p.y, sizeof(float) * (size_t)g_p.n));

  fillRandom(g_p, v.box(), 800, 31337);
  CHECK(g_h.build(v, g_p, g_scratch));
  const uint64_t h2 = fnv1a(g_p.x, sizeof(float) * (size_t)g_p.n,
                            fnv1a(g_p.y, sizeof(float) * (size_t)g_p.n));
  CHECK(h1 == h2);
}

TEST(hash_empty_particle_set_is_safe) {
  SimVolume v;
  CHECK(v.build(Geometry::cube(32, 1.0f), 2.0f, kCellSize));
  g_p.clear();
  CHECK(g_h.build(v, g_p, g_scratch));
  for (int c = 0; c < g_h.cellCount(); ++c) CHECK(g_h.cellBegin(c) == g_h.cellEnd(c));

  int visited = 0;
  forEachNeighbour(v, g_h, Vec3{0, 0, 0}, [&](int) { ++visited; });
  CHECK(visited == 0);
}
