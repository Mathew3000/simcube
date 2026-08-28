// Static memory footprint of the simulation at the capacities this build was configured with.
//
// The point is the ESP32: every pool is fixed-size and lives in .bss, so the whole budget is
// knowable at compile time -- and the plan's "~165KB of the ~230KB internal heap" was an
// estimate nobody had ever printed. Built at PARTSIM_PROFILE=esp32 this is the actual number.
//
// Usage: partsim_memreport [budget_kb]
// Exits non-zero if the total exceeds budget_kb, so it can be a ctest.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

#include "partsim/RenderState.h"
#include "partsim/Simulation.h"

using namespace partsim;

namespace {

// The HUB75 DMA descriptor buffer, which is not part of Simulation but comes out of the same
// internal SRAM and is by far the largest single consumer. Formula rather than a remembered
// figure: one 16-bit word per pixel per bitplane, over h/2 rows because the panels are
// two-scan-lines-at-a-time, doubled for the flicker-free back buffer.
size_t hub75Bytes(int chainW, int chainH, int depthBits, bool doubleBuffer) {
  const size_t rows = (size_t)chainH / 2;
  const size_t perBuffer = rows * (size_t)chainW * sizeof(uint16_t) * (size_t)depthBits;
  return perBuffer * (doubleBuffer ? 2 : 1);
}

void row(const char* name, size_t bytes, size_t total) {
  std::printf("  %-26s %8zu B  %6.1f KB  %5.1f%%\n", name, bytes, (double)bytes / 1024.0,
              100.0 * (double)bytes / (double)total);
}

}  // namespace

int main(int argc, char** argv) {
  const double budgetKb = (argc > 1) ? std::atof(argv[1]) : 0.0;

  // Sub-pool sizes, recomputed from the same constants the pools are declared with rather than
  // read out of the object -- a member-by-member sizeof would need the fields to be public.
  const size_t particles = sizeof(Particles);
  const size_t hash = sizeof(SpatialHash);
  const size_t field = sizeof(FieldGrid);
  const size_t renderer = sizeof(Renderer);
  const size_t geometry = sizeof(Geometry);
  const size_t solver = sizeof(Solver);
  const size_t sim = sizeof(Simulation);
  const size_t accounted = particles + hash + field + renderer + geometry + solver;
  const size_t other = (sim > accounted) ? sim - accounted : 0;

  // Panel side from the capacity, not a hardcoded 32: 1024 texels is a 32x32 tile, 4096 is 64x64.
  // Everything below used to assume 32-row panels, which would have silently under-reported the
  // DMA buffer by 4x on a 64x64 build -- the single largest number in this report.
  int side = 1;
  while ((side + 1) * (side + 1) <= kMaxPanelTexels) ++side;

  // The chain THIS ROLE drives, which is the render-panel count -- not the number of panels that
  // exist. A display node holding a six-panel table still owns a two-tile chain.
  const int chainW = side * kMaxRenderPanels;
  // Only charge for a DMA buffer if this role actually has a HUB75 connector. The master does not,
  // and charging it 48KB for a chain it never drives overstated it by nearly 40%.
  const size_t dma = PARTSIM_DRIVES_PANELS ? hub75Bytes(chainW, side, 6, true) : 0u;
  const size_t staging = PARTSIM_DRIVES_PANELS ? (size_t)kMaxPanelTexels * 3u : 0u;

  // A role that does not run the solver carries RenderState's draw-only containers instead of a
  // whole Simulation. Reporting sizeof(Simulation) for it would measure 55.7KB of pools the node
  // never touches -- which is precisely the mistake that made a display node look 14.6KB over
  // budget when it is in fact comfortably inside.
  const size_t drawOnly = sizeof(RenderParticles) + sizeof(HeatBuffer) + sizeof(Renderer)
                        + sizeof(Geometry);
  const size_t stateTotal = PARTSIM_RUNS_SOLVER ? sim : drawOnly;
  const size_t total = stateTotal + dma + staging;

  std::printf("partsim static memory report\n");
  std::printf("  profile              : %s\n",
#ifdef PARTSIM_PROFILE_ESP32
              "esp32 (one node, six 32x32)"
#elif defined(PARTSIM_PROFILE_ESP32_MASTER)
              "esp32-master (physics, no panels)"
#elif defined(PARTSIM_PROFILE_ESP32_DISPLAY)
              "esp32-display (two 64x64 faces)"
#else
              "host/wasm"
#endif
  );
  std::printf("  max particles        : %d\n", kMaxParticles);
  std::printf("  max panels           : %d\n", kMaxPanels);
  std::printf("  max texels per panel : %d  (%dx%d)\n", kMaxPanelTexels, side, side);
  std::printf("  panels rendered here : %d of %d\n", kMaxRenderPanels, kMaxPanels);
  if (PARTSIM_DRIVES_PANELS) std::printf("  hub75 chain          : %dx%d\n", chainW, side);
  else                       std::printf("  hub75 chain          : none (drives no panels)\n");
  std::printf("  max grid cells       : %d\n", kMaxGridCells);
  std::printf("  max field cells      : %d\n", kMaxFieldCells);
  std::printf("  internal RGBA copy   : %s\n", PARTSIM_INTERNAL_PIXELS ? "yes" : "no");
  std::printf("  runs the solver      : %s\n", PARTSIM_RUNS_SOLVER ? "yes" : "no (draws only)");
  std::printf("\n");

  if (PARTSIM_RUNS_SOLVER) {
    row("Particles (SoA pool)", particles, total);
    row("SpatialHash (sort+grid)", hash, total);
    row("FieldGrid (heat, x2)", field, total);
    row("Renderer (accum+pixels)", renderer, total);
    row("Geometry", geometry, total);
    row("Solver", solver, total);
    row("Simulation, other", other, total);
    std::printf("  %-26s %8zu B  %6.1f KB\n", "-- Simulation total", sim, (double)sim / 1024.0);
  } else {
    row("RenderParticles (draw-only)", sizeof(RenderParticles), total);
    row("HeatBuffer (single)", sizeof(HeatBuffer), total);
    row("Renderer (accum+pixels)", renderer, total);
    row("Geometry", geometry, total);
    std::printf("  %-26s %8zu B  %6.1f KB\n", "-- draw-only total", drawOnly,
                (double)drawOnly / 1024.0);
    std::printf("  %-26s %8zu B  %6.1f KB  (never instantiated here)\n", "   a full Simulation",
                sim, (double)sim / 1024.0);
  }
  std::printf("\n");
  row("HUB75 DMA (6bit, double)", dma, total);
  row("one-face RGB staging", staging, total);
  std::printf("\n  %-26s %8zu B  %6.1f KB\n", "== TOTAL internal SRAM", total,
              (double)total / 1024.0);

  // Per-particle cost, because it is the number to reason with when trading particles for
  // anything else.
  std::printf("  %-26s %8.1f B/particle\n", "   particle pools",
              (double)(particles + hash + other) / (double)kMaxParticles);

  if (budgetKb > 0.0) {
    const double kb = (double)total / 1024.0;
    std::printf("\n  budget %.1f KB, using %.1f KB, %.1f KB %s\n", budgetKb, kb,
                budgetKb - kb >= 0.0 ? budgetKb - kb : kb - budgetKb,
                budgetKb - kb >= 0.0 ? "free" : "OVER");
    if (kb > budgetKb) {
      std::printf("  FAIL: static footprint exceeds the internal SRAM budget\n");
      return 1;
    }
    std::printf("  OK\n");
  }

  // --- topology matrix ---------------------------------------------------------------------
  // What a DISPLAY node costs per face count, with the DMA buffer internal or in PSRAM. This is
  // the table the board decision turns on: how many HUB75 connectors one display board needs.
  //
  // Everything except the accumulator and the DMA buffer is shared regardless of face count --
  // the particle state arrives whole over SPI because any particle can light any face.
  std::printf("\n  display-node topology (64x64 faces, DMA 6-bit double-buffered)\n");
  const size_t accumPerFace = (size_t)kMaxPanelTexels * kChannelCount * sizeof(uint16_t);
  const size_t dmaPerFace = hub75Bytes(side, side, 6, true);
  // Real sizeofs now that RenderState exists, rather than the 16 B/particle estimate this used
  // to carry. Floats, not the wire's int16/int8: dequantising once at decode beats doing it per
  // panel inside the splat loop, and it keeps one splat implementation for both roles.
  const size_t sharedRender = (size_t)kMaxPanelTexels * 3u        // staging, one face at a time
                            + sizeof(RenderParticles)             // draw-only particle state
                            + sizeof(HeatBuffer)                  // heat, single buffer
                            + sizeof(Geometry);
  std::printf("    %-6s %10s %10s %12s %12s\n", "faces", "accum", "dma", "internal", "internal");
  std::printf("    %-6s %10s %10s %12s %12s\n", "", "", "", "dma inside", "dma in psram");
  for (int f : {1, 2, 3, 6}) {
    const double acc = (double)(accumPerFace * (size_t)f) / 1024.0;
    const double dm = (double)(dmaPerFace * (size_t)f) / 1024.0;
    const double sh = (double)sharedRender / 1024.0;
    std::printf("    %-6d %8.1f K %8.1f K %10.1f K %10.1f K\n", f, acc, dm, acc + dm + sh, acc + sh);
  }
  std::printf("    shared regardless of face count: %.1f KB", (double)sharedRender / 1024.0);
  std::printf("  (RenderParticles %.1f + HeatBuffer %.1f + staging %.1f + geometry %.1f)\n",
              sizeof(RenderParticles) / 1024.0, sizeof(HeatBuffer) / 1024.0,
              kMaxPanelTexels * 3 / 1024.0, sizeof(Geometry) / 1024.0);
  std::printf("    %.1f B/particle draw-only, against %.1f B/particle for the solver pools\n",
              (double)sizeof(RenderParticles) / kMaxParticles,
              (double)(sizeof(Particles) + sizeof(SpatialHash)) / kMaxParticles);
  return 0;
}
