// Host-side inspector / benchmark. Dumps the baked panel table (step-2 verification) and
// runs a hydrostatic settling experiment with solver diagnostics, which is the tuning loop
// for the PBF constants.
#include <chrono>
#include <cstdio>
#include <cstdlib>

#include "partsim/Rng.h"
#include "partsim/Solver.h"

using namespace partsim;

namespace {

// Far too big for the stack.
Particles g_p;
SpatialHash g_h;
Solver g_solver;
float g_scratch[kMaxParticles];

const char* faceName(int i) {
  static const char* names[6] = {"-Z", "+Z", "-X", "+X", "-Y bot", "+Y top"};
  return i < 6 ? names[i] : "?";
}

void dumpGeometry(const char* label, const Geometry& g, float slabDepth) {
  std::printf("\n=== %s: %d panel(s), %d texels ===\n", label, g.count(), g.totalTexels());
  std::printf("%-8s %-22s %-16s %-16s %-16s\n", "face", "origin", "u (right)", "v (up)",
              "n (inward)");
  for (int i = 0; i < g.count(); ++i) {
    const Panel& p = g.at(i);
    std::printf("%-8s (%6.1f %6.1f %6.1f) (%4.1f %4.1f %4.1f) (%4.1f %4.1f %4.1f) "
                "(%4.1f %4.1f %4.1f)  %dx%d\n",
                faceName(i), p.origin.x, p.origin.y, p.origin.z, p.u.x, p.u.y, p.u.z,
                p.v.x, p.v.y, p.v.z, p.n.x, p.n.y, p.n.z, (int)p.w, (int)p.h);
  }

  const Aabb b = g.bounds(slabDepth);
  std::printf("bounds  size (%.1f %.1f %.1f)\n", b.size().x, b.size().y, b.size().z);

  SimVolume v;
  if (!v.build(g, slabDepth, kCellSize)) {
    std::printf("volume  BUILD FAILED\n");
    return;
  }
  std::printf("volume  grid %dx%dx%d = %d cells @ cell %.1f\n", v.dim().x, v.dim().y,
              v.dim().z, v.cellCount(), v.cellSize());
}

// Fill the bottom of the box with a jittered rest lattice. Jitter matters: a perfect
// lattice is a metastable configuration and hides whether the solver actually holds
// together.
int fillBottom(Particles& p, const Aabb& box, int want, uint8_t mat, uint32_t seed) {
  Rng r(seed);
  const float d = kRestSpacing;
  const int nx = (int)(box.size().x / d);
  const int nz = (int)(box.size().z / d);
  const int layers = (nx * nz > 0) ? (want + nx * nz - 1) / (nx * nz) : 0;

  for (int ly = 0; ly < layers; ++ly) {
    for (int iz = 0; iz < nz; ++iz) {
      for (int ix = 0; ix < nx; ++ix) {
        if (p.n >= want) return p.n;
        const Vec3 q{box.lo.x + (0.5f + (float)ix) * d + r.nextSigned() * 0.1f * d,
                     box.lo.y + (0.5f + (float)ly) * d + r.nextSigned() * 0.1f * d,
                     box.lo.z + (0.5f + (float)iz) * d + r.nextSigned() * 0.1f * d};
        p.add(q, Vec3{0, 0, 0}, mat);
      }
    }
  }
  return p.n;
}

struct Diag {
  float meanSpeed, maxSpeed, meanDensityInterior, meanDensityAll, surfaceY, minY, maxY;
  float fastFraction;  // share of particles moving faster than 1 unit/s
  int outside, interiorCount;
};

Diag diagnose(const Particles& p, const SimVolume& v, const SpatialHash& h,
              const Solver& s) {
  Diag d{0, 0, 0, 0, 0.0f, 1e30f, -1e30f, 0.0f, 0, 0};
  const Aabb& b = v.box();
  double sumSpeed = 0.0, sumRho = 0.0, sumRhoAll = 0.0, sumY = 0.0;
  int fast = 0;

  for (int i = 0; i < p.n; ++i) {
    const float sp = length(p.vel(i));
    sumSpeed += sp;
    if (sp > d.maxSpeed) d.maxSpeed = sp;

    if (sp > 1.0f) ++fast;
    const Vec3 q = p.pos(i);
    sumY += q.y;
    sumRhoAll += s.densityAt(p, v, h, i);
    if (q.y < d.minY) d.minY = q.y;
    if (q.y > d.maxY) d.maxY = q.y;
    if (!b.contains(q)) ++d.outside;

    // "Interior" == at least h from every wall, so the wall term is not involved.
    const float m = kSmoothRadius;
    if (q.x - b.lo.x > m && b.hi.x - q.x > m && q.y - b.lo.y > m && b.hi.y - q.y > m &&
        q.z - b.lo.z > m && b.hi.z - q.z > m) {
      sumRho += s.densityAt(p, v, h, i);
      ++d.interiorCount;
    }
  }
  d.meanSpeed = p.n ? (float)(sumSpeed / p.n) : 0.0f;
  d.meanDensityInterior = d.interiorCount ? (float)(sumRho / d.interiorCount) : 0.0f;
  d.meanDensityAll = p.n ? (float)(sumRhoAll / p.n) : 0.0f;

  // Fill height from the centre of mass: for a settled column of uniform density resting
  // on the floor, meanY = lo + H/2. Far more robust than a top-percentile, which mostly
  // measures how much the surface is splashing.
  d.surfaceY = 2.0f * ((float)(sumY / (p.n ? p.n : 1)) - b.lo.y);
  d.fastFraction = p.n ? (float)fast / (float)p.n : 0.0f;
  return d;
}

// Decisive probe: for the particle nearest the centre of the settled mass, compare the
// grid density, a brute-force O(N) density, and the raw neighbour count. A cubic rest
// lattice at spacing d with h = 2d has 26 neighbours within h, so the count tells us the
// true local packing independently of any kernel bookkeeping.
void probeDensity(const Particles& p, const SimVolume& v, const SpatialHash& h,
                  const Solver& s) {
  if (p.n == 0) return;
  const Aabb& b = v.box();
  double sy = 0.0;
  for (int i = 0; i < p.n; ++i) sy += p.y[i];
  const Vec3 target{b.center().x, (float)(sy / p.n), b.center().z};

  int best = 0;
  float bestD = 1e30f;
  for (int i = 0; i < p.n; ++i) {
    const float d2 = length2(p.pos(i) - target);
    if (d2 < bestD) { bestD = d2; best = i; }
  }

  const Vec3 pi = p.pos(best);
  float brute = s.mass() * s.kernels().poly6(0.0f);
  int within = 0;
  for (int j = 0; j < p.n; ++j) {
    if (j == best) continue;
    const float r2 = length2(p.pos(j) - pi);
    if (r2 < s.kernels().h2) { brute += s.mass() * s.kernels().poly6(r2); ++within; }
  }

  std::printf("probe   at (%.1f %.1f %.1f): grid rho %.4f  brute rho %.4f  "
              "neighbours %d (rest lattice = 26)\n",
              pi.x, pi.y, pi.z, s.densityAt(p, v, h, best), brute, within);
}

// Vertical profile: particle count and mean density per 2-unit band. Settles any argument
// about whether the column is uniform.
void profile(const Particles& p, const SimVolume& v, const SpatialHash& h,
             const Solver& s) {
  const Aabb& b = v.box();
  const float band = 2.0f;
  const int bands = (int)(b.size().y / band);
  std::printf("profile  band      n   rho   (expected n per band for a full layer: "
              "%d)\n",
              (int)(band * b.size().x * b.size().z / s.mass()));
  for (int k = 0; k < bands; ++k) {
    const float y0 = b.lo.y + (float)k * band;
    int cnt = 0;
    double rho = 0.0;
    for (int i = 0; i < p.n; ++i)
      if (p.y[i] >= y0 && p.y[i] < y0 + band) {
        ++cnt;
        rho += s.densityAt(p, v, h, i);
      }
    if (cnt == 0 && k > bands / 2) break;
    std::printf("       %6.1f %6d %6.3f\n", y0, cnt, cnt ? rho / cnt : 0.0);
  }
}

void hydrostatic(int count, int steps, int iters, float damping, float eps, int substeps,
                 bool quiet) {
  const Geometry g = Geometry::cube(32, 1.0f);
  SimVolume v;
  v.build(g, kSlabDepth, kCellSize);
  g_solver.init();
  g_solver.setIterations(iters);
  g_solver.setDamping(damping);
  g_solver.setEpsilon(eps);

  g_p.clear();
  fillBottom(g_p, v.box(), count, kWater, 0xA11CE);

  const float expectedHeight =
      (float)g_p.n * kRestSpacing * kRestSpacing * kRestSpacing /
      (v.box().size().x * v.box().size().z);

  std::printf("\n=== hydrostatic: %d water particles, %d steps, iters %d, damping %.2f, eps %.4f ===\n",
              g_p.n, steps, iters, damping, (double)eps);
  std::printf("mass %.4f  rest density %.3f  expected fill height %.1f units\n",
              g_solver.mass(), g_solver.restDensity(), expectedHeight);
  std::printf("%6s %9s %9s %9s %9s %8s %8s\n", "step", "mean|v|", "max|v|", "rho_int",
              "surfaceY", "minY", "outside");

  const Vec3 gravity{0.0f, -kGravityMag, 0.0f};
  double totalMs = 0.0;

  for (int s = 0; s <= steps; ++s) {
    if (!quiet && s % 100 == 0) {
      const Diag d = diagnose(g_p, v, g_h, g_solver);
      std::printf("%6d %9.3f %9.3f %9.4f %9.2f %8.2f %8d\n", s, d.meanSpeed, d.maxSpeed,
                  d.meanDensityInterior, d.surfaceY, d.minY, d.outside);
    }
    if (s == steps) break;
    const auto t0 = std::chrono::steady_clock::now();
    for (int sub = 0; sub < substeps; ++sub)
      g_solver.step(g_p, v, g_h, g_scratch, defaultMaterials(), gravity,
                    kFixedDt / (float)substeps);
    totalMs += std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - t0)
                   .count();
  }

  probeDensity(g_p, v, g_h, g_solver);
  if (!quiet) profile(g_p, v, g_h, g_solver);
  const Diag d = diagnose(g_p, v, g_h, g_solver);
  std::printf("FINAL mean|v| %6.3f  max|v| %6.2f  rho %.4f  moving %5.1f%%  "
              "fill %4.1f (want %.1f)  outside %d  %6.3f ms/step\n",
              d.meanSpeed, d.maxSpeed, d.meanDensityAll, 100.0 * d.fastFraction,
              d.surfaceY, expectedHeight, d.outside, totalMs / steps);
}

}  // namespace

int main(int argc, char** argv) {
  std::printf("partsim bench -- capacities: %d particles, %d panels, %d grid cells\n",
              kMaxParticles, kMaxPanels, kMaxGridCells);
  dumpGeometry("cube 32", Geometry::cube(32, 1.0f), kSlabDepth);
  dumpGeometry("single panel 32x32 (thin slab)", Geometry::slab(32, 32, 1.0f), kSlabDepth);

  const int count = (argc > 1) ? atoi(argv[1]) : 3000;
  const int steps = (argc > 2) ? atoi(argv[2]) : 600;
  if (argc > 3 && argv[3][0] == 's') {
    // sweep mode: bench N STEPS sweep
    const int iterList[3] = {2, 3, 4};
    const float dampList[2] = {1.0f, 0.96f};
    const int subList[3] = {1, 2, 3};
    for (int a = 0; a < 3; ++a)
      for (int bb = 0; bb < 2; ++bb)
        for (int c = 0; c < 3; ++c)
          hydrostatic(count, steps, iterList[a], dampList[bb], 0.05f, subList[c], true);
    return 0;
  }
  const int iters = (argc > 3) ? atoi(argv[3]) : kSolverIterations;
  const float damping = (argc > 4) ? (float)atof(argv[4]) : kVelocityDamping;
  const float eps = (argc > 5) ? (float)atof(argv[5]) : kCfmEpsilon;
  const int substeps = (argc > 6) ? atoi(argv[6]) : 1;
  hydrostatic(count, steps, iters, damping, eps, substeps, false);
  return 0;
}
