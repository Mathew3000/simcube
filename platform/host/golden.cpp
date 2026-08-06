// Prints the golden state hash and the rendered-pixel hash. The node harness prints the same
// two numbers from the WASM build; a mismatch is a real cross-target divergence, not noise.
#include <cstdio>
#include <cstring>

#include "partsim/Simulation.h"

using namespace partsim;

namespace {
Simulation g_sim;  // ~1.2MB
// One face at a time, which is also how the ESP32 does it -- so this tool exercises the
// firmware's render path rather than a host-only shortcut. Resolving into a staging buffer is
// bit-identical to resolving into Renderer's internal copy, so the hash is unaffected.
uint8_t g_face[kMaxPanelTexels * 4];
}  // namespace

int main(int argc, char** argv) {
  const bool quiet = (argc > 1) && std::strcmp(argv[1], "-q") == 0;
  const uint32_t state = goldenHash(g_sim, kGoldenSteps, kGoldenSeed);
  g_sim.accumulate();
  const uint32_t pixels = [] {
    uint64_t h = 1469598103934665603ull;
    for (int k = 0; k < g_sim.geometry().count(); ++k) {
      const Panel& p = g_sim.geometry().at(k);
      g_sim.renderer().resolve(k, g_face, 4);
      h = fnv1a(g_face, (size_t)p.w * (size_t)p.h * 4u, h);
    }
    return (uint32_t)(h ^ (h >> 32));
  }();

  if (quiet) {
    std::printf("%08x %08x\n", state, pixels);
  } else {
    std::printf("golden: scenes %d+%d, %d steps each, seed %08x, %d particles\n",
                kGoldenSceneA, kGoldenSceneB, kGoldenSteps, kGoldenSeed,
                g_sim.particleCount());
    std::printf("state  %08x\n", state);
    std::printf("pixels %08x\n", pixels);
  }
  return 0;
}
