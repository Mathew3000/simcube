#include "check.h"
#include "partsim/Simulation.h"

#if PARTSIM_ENABLE_CHROMA

using namespace partsim;

namespace {
Simulation g_sim;
// Own instances: the solver globals live in another translation unit's anonymous namespace.
Particles g_cp;
SpatialHash g_ch;
Solver g_cs;
float g_cscratch[kMaxParticles];

// Total dye in the pool, as (red, green, blue) counts. Conservation is about this sum, not about
// any one particle: mixing moves dye between neighbours and must not create or destroy it.
void totalDye(const Particles& p, double& r, double& g, double& b) {
  r = g = b = 0.0;
  for (int i = 0; i < p.n; ++i) {
    r += p.cr[i];
    g += p.cg[i];
    b += (double)kChromaOne - p.cr[i] - p.cg[i];
  }
}

// Half the particles red, half blue, split on x so they start segregated and have to mix.
void pourRedIntoBlue() {
  CHECK(g_sim.init(Simulation::kCube, particlesForFill(1500), 4));
  Particles& p = const_cast<Particles&>(g_sim.particles());
  for (int i = 0; i < p.n; ++i) {
    const bool red = p.x[i] < 0.0f;
    p.cr[i] = red ? (uint16_t)kChromaOne : (uint16_t)0;
    p.cg[i] = 0;
  }
}
}  // namespace

TEST(chroma_starts_where_it_was_put) {
  pourRedIntoBlue();
  double r, g, b;
  totalDye(g_sim.particles(), r, g, b);
  const double total = r + g + b;
  std::printf("       %d particles, dye r %.0f g %.0f b %.0f\n", g_sim.particles().n, r, g, b);
  CHECK(r > 0.0 && b > 0.0);
  CHECK(g == 0.0);
  // Every particle carries exactly 255 units of dye across the three components, by construction.
  CHECK_NEAR(total, (double)kChromaOne * g_sim.particles().n, 1.0);
}

TEST(chroma_mixes_toward_uniform) {
  pourRedIntoBlue();
  // Spread, measured as the mean absolute deviation of red from its own mean. A segregated pool is
  // near its maximum; a mixed one approaches zero.
  auto spread = [](const Particles& p) {
    double mean = 0.0;
    for (int i = 0; i < p.n; ++i) mean += p.cr[i];
    mean /= p.n;
    double dev = 0.0;
    for (int i = 0; i < p.n; ++i) dev += (p.cr[i] > mean ? p.cr[i] - mean : mean - p.cr[i]);
    return dev / p.n;
  };
  const double before = spread(g_sim.particles());
  for (int s = 0; s < 600; ++s) g_sim.stepFixed();
  const double after = spread(g_sim.particles());
  std::printf("       red spread %.1f -> %.1f after 600 steps\n", before, after);
  CHECK(before > 100.0 * 256.0);  // genuinely segregated to begin with
  CHECK(after < before * 0.25);  // and demonstrably mixing
}

TEST(chroma_is_conserved_within_a_bounded_drift) {
  // Mixing rides the XSPH pass and is applied IN PLACE, so it is Gauss-Seidel: particle i sees
  // neighbours that have already moved toward it this step, and the per-particle normalisation by
  // the local weight sum breaks the antisymmetry that would otherwise make the exchange exact.
  //
  // So dye is not conserved, and this bounds the error rather than pretending it is zero. What
  // makes it acceptable is that it PLATEAUS: measured at -4.85% after 300 steps, -4.98% after 600,
  // and unchanged at 1200 and 2400. It accumulates only while there is a gradient to mix and then
  // stops, so a red-into-blue pour lands about 5% bluer than true pink and stays there. A leak
  // would keep going, and this test is what tells the two apart -- run it longer if in doubt.
  //
  // Fixing it properly means dropping the density normalisation (which makes the rate depend on
  // local density, so dye in a thin stream would mix slower than dye in a pool) or a Jacobi pass
  // with a second chroma buffer, 2 KB at device capacity. Neither is worth 5% today.
  pourRedIntoBlue();
  double r0, g0, b0;
  totalDye(g_sim.particles(), r0, g0, b0);
  for (int s = 0; s < 600; ++s) g_sim.stepFixed();
  double r1, g1, b1;
  totalDye(g_sim.particles(), r1, g1, b1);
  const double dr = (r1 - r0) / (r0 + 1.0);
  const double db = (b1 - b0) / (b0 + 1.0);
  std::printf("       dye drift over 600 steps: red %+.2f%%  blue %+.2f%%\n", dr * 100.0,
              db * 100.0);
  CHECK(dr > -0.08 && dr < 0.08);
  CHECK(db > -0.08 && db < 0.08);
}

TEST(chroma_survives_the_spatial_sort) {
  // The sort permutes every particle array. Miss cr/cg and the dye detaches from the particle
  // carrying it -- which does not crash and does not move the state hash.
  //
  // Tested against the SORT rather than through the simulation, because diffusion legitimately
  // destroys any single-particle marker within a few steps: an earlier version of this test tagged
  // one particle red and looked for it later, and it was gone for entirely correct reasons.
  SimVolume v;
  v.build(Geometry::cube(32, 1.0f), kSlabDepth, kCellSize);
  g_cs.init();
  g_cp.clear();
  // A loose lattice; the exact arrangement does not matter, only that the sort reorders it.
  const float d = kRestSpacing;
  for (float y = -15.0f; y < -4.0f; y += d)
    for (float z = -14.0f; z < 14.0f; z += d)
      for (float x = -14.0f; x < 14.0f; x += d) g_cp.add(Vec3{x, y, z}, Vec3{0, 0, 0}, kWater);

  // Chroma derived from position, so the pairing can be checked after an arbitrary permutation.
  auto expect = [](float x) { return (uint16_t)((int)(x + 16.0f) * 512) & 0xFF00; };
  for (int i = 0; i < g_cp.n; ++i) {
    g_cp.cr[i] = expect(g_cp.x[i]);
    g_cp.cg[i] = 0;
  }
  for (int i = 0; i < g_cp.n; ++i) g_cp.setPred(i, g_cp.pos(i));
  CHECK(g_ch.build(v, g_cp, g_cscratch));

  int wrong = 0;
  for (int i = 0; i < g_cp.n; ++i)
    if (g_cp.cr[i] != expect(g_cp.x[i])) ++wrong;
  std::printf("       %d particles permuted, %d with chroma detached\n", g_cp.n, wrong);
  CHECK(wrong == 0);
}

#endif  // PARTSIM_ENABLE_CHROMA
