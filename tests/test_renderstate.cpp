// A draw-only node must produce EXACTLY the pixels a full simulation would.
//
// This is the property the multi-node cube rests on. RenderState.h exists so a display node can
// drop the solver's working set -- 55.7 KB of predicted positions, Lagrange multipliers, a
// neighbour grid it never builds and a heat buffer it never advects -- and the risk of that is a
// second splat path that quietly diverges. So the test copies a settled simulation into the
// draw-only containers and demands bit-identical output.
#include "check.h"
#include "partsim/RenderState.h"
#include "partsim/Simulation.h"

using namespace partsim;

namespace {

Simulation g_sim;          // ~1.2MB
RenderParticles g_rp;      // the draw-only pool
HeatBuffer g_hb;
Renderer g_r2;             // a second renderer, fed from the views

// Hash the accumulation buffers of every face, which is stricter than hashing resolved pixels:
// resolve() quantises to 8 bits per channel and could mask a small divergence.
uint64_t accumHash(const Renderer& r, const Geometry& g) {
  uint64_t h = 1469598103934665603ull;
  for (int k = 0; k < g.count(); ++k) {
    const Panel& p = g.at(k);
    for (int j = 0; j < (int)p.h; ++j)
      for (int i = 0; i < (int)p.w; ++i)
        for (int c = 0; c < kChannelCount; ++c) {
          const uint16_t v = r.accumAt(k, i, j, c);
          h = fnv1a(&v, sizeof(v), h);
        }
  }
  return h;
}

// Copies simulation state into the draw-only containers, exactly as a display node's decoder
// would after receiving a frame -- but without quantisation, so any difference is the renderer's.
void mirrorInto(const Simulation& sim) {
  const Particles& p = sim.particles();
  g_rp.clear();
  for (int i = 0; i < p.n; ++i) CHECK(g_rp.add(p.pos(i), p.vel(i), p.mat[i]));

  CHECK(g_hb.init(sim.volume()));
  const FieldGrid& f = sim.field();
  CHECK(g_hb.cellCount() == f.cellCount());
  uint8_t peak = 0;
  for (int i = 0; i < f.cellCount(); ++i) {
    const uint8_t v = f.at(i);
    g_hb.cells()[i] = v;
    if (v > peak) peak = v;
  }
  g_hb.setPeak(peak);
}

}  // namespace

TEST(renderstate_grid_derivation_is_shared) {
  // FieldGrid and HeatBuffer must agree on the grid exactly. Two independent copies of the
  // derivation would fail silently -- as a plume drawn in the wrong place, not as an error.
  CHECK(g_sim.initScene(Simulation::kCube, 4, 1));  // kettle: has an active field
  CHECK(g_hb.init(g_sim.volume()));
  const FieldGrid& f = g_sim.field();
  CHECK(g_hb.dim().x == f.dim().x);
  CHECK(g_hb.dim().y == f.dim().y);
  CHECK(g_hb.dim().z == f.dim().z);
  CHECK(g_hb.cellCount() == f.cellCount());
  std::printf("       both derive a %dx%dx%d grid, %d cells\n", g_hb.dim().x, g_hb.dim().y,
              g_hb.dim().z, g_hb.cellCount());
}

TEST(renderstate_draws_bit_identically_to_the_full_simulation) {
  // The kettle, because it exercises water, sand and an active heat field at once. A water-only
  // scene would leave the whole splatField path unchecked.
  CHECK(g_sim.initScene(Simulation::kCube, 4, 7));
  for (int i = 0; i < 400; ++i) g_sim.stepFixed();

  const Geometry& g = g_sim.geometry();

  // Reference: the full types.
  g_sim.accumulate();
  const uint64_t want = accumHash(g_sim.renderer(), g);

  // Draw-only: same state, no solver pools.
  mirrorInto(g_sim);
  CHECK(g_r2.init(g));
  g_r2.setExposure(kSplatExposure);
  g_r2.setTimeOffset(g_sim.renderer().timeOffset());
  g_r2.accumulate(g_rp.view(), g_hb.view(), g);
  const uint64_t got = accumHash(g_r2, g);

  std::printf("       %d particles, %d heat cells: accum hash %016llx vs %016llx\n",
              g_rp.count(), g_hb.cellCount(), (unsigned long long)want, (unsigned long long)got);
  CHECK(want == got);
  CHECK(want != 1469598103934665603ull);  // and something was actually drawn
}

TEST(renderstate_interpolation_matches_too) {
  // The velocity extrapolation path reads different fields, so it needs its own check.
  CHECK(g_sim.initScene(Simulation::kCube, 0, 3));
  for (int i = 0; i < 60; ++i) g_sim.stepFixed();  // still moving, so velocity matters
  const Geometry& g = g_sim.geometry();

  g_sim.renderer().setTimeOffset(0.011f);
  g_sim.renderer().accumulate(g_sim.particles(), g_sim.field(), g);
  const uint64_t want = accumHash(g_sim.renderer(), g);

  mirrorInto(g_sim);
  CHECK(g_r2.init(g));
  g_r2.setExposure(kSplatExposure);
  g_r2.setTimeOffset(0.011f);
  g_r2.accumulate(g_rp.view(), g_hb.view(), g);
  CHECK(want == accumHash(g_r2, g));
}

TEST(renderstate_is_smaller_than_the_solver_pools) {
  // The whole point, asserted so a future field added to RenderParticles cannot quietly undo it.
  const size_t draw = sizeof(RenderParticles) + sizeof(HeatBuffer);
  const size_t full = sizeof(Particles) + sizeof(FieldGrid) + sizeof(SpatialHash);
  std::printf("       draw-only %.1f KB vs solver pools %.1f KB (%.0f%% saved)\n",
              draw / 1024.0, full / 1024.0, 100.0 * (1.0 - (double)draw / (double)full));
  CHECK(draw < full);
  // Per particle: 25 bytes against Particles' 41.
  CHECK(sizeof(RenderParticles) / kMaxParticles < sizeof(Particles) / kMaxParticles);
}
