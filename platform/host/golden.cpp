// Prints the golden state hash and the rendered-pixel hash. The node harness prints the same
// two numbers from the WASM build; a mismatch is a real cross-target divergence, not noise.
#include <cstdio>
#include <cstring>

#include "partsim/Simulation.h"

using namespace partsim;

namespace {
Simulation g_sim;  // ~1.2MB
}

int main(int argc, char** argv) {
  const bool quiet = (argc > 1) && std::strcmp(argv[1], "-q") == 0;
  const uint32_t state = goldenHash(g_sim, kGoldenSteps, kGoldenParticles, kGoldenSeed);
  g_sim.render();
  const uint32_t pixels = [] {
    uint64_t h = 1469598103934665603ull;
    for (int k = 0; k < g_sim.geometry().count(); ++k) {
      const Panel& p = g_sim.geometry().at(k);
      h = fnv1a(g_sim.renderer().panelPixels(k), (size_t)p.w * (size_t)p.h * 4u, h);
    }
    return (uint32_t)(h ^ (h >> 32));
  }();

  if (quiet) {
    std::printf("%08x %08x\n", state, pixels);
  } else {
    std::printf("golden steps=%d particles=%d seed=%08x\n", kGoldenSteps,
                g_sim.particleCount(), kGoldenSeed);
    std::printf("state  %08x\n", state);
    std::printf("pixels %08x\n", pixels);
  }
  return 0;
}
