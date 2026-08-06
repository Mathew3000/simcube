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

  const int chainW = 32 * kMaxPanels;
  const size_t dma = hub75Bytes(chainW, 32, 6, true);
  const size_t staging = 32 * 32 * 3;  // one face of RGB, reused across faces
  const size_t total = sim + dma + staging;

  std::printf("partsim static memory report\n");
  std::printf("  profile              : %s\n",
#ifdef PARTSIM_PROFILE_ESP32
              "esp32"
#else
              "host/wasm"
#endif
  );
  std::printf("  max particles        : %d\n", kMaxParticles);
  std::printf("  max panels           : %d\n", kMaxPanels);
  std::printf("  max texels per panel : %d\n", kMaxPanelTexels);
  std::printf("  max grid cells       : %d\n", kMaxGridCells);
  std::printf("  max field cells      : %d\n", kMaxFieldCells);
  std::printf("  internal RGBA copy   : %s\n", PARTSIM_INTERNAL_PIXELS ? "yes" : "no");
  std::printf("\n");

  row("Particles (SoA pool)", particles, total);
  row("SpatialHash (sort+grid)", hash, total);
  row("FieldGrid (heat, x2)", field, total);
  row("Renderer (accum+pixels)", renderer, total);
  row("Geometry", geometry, total);
  row("Solver", solver, total);
  row("Simulation, other", other, total);
  std::printf("  %-26s %8zu B  %6.1f KB\n", "-- Simulation total", sim, (double)sim / 1024.0);
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
  return 0;
}
