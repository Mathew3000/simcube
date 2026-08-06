// Enforces architecture invariant 3: no dynamic allocation after init(), on every target.
//
// This is not a style rule. The ESP32-S3 has ~230KB of usable internal SRAM and every pool is
// sized to fit it; a stray allocation in the step or render path is a heap fragmentation bug
// waiting to strand the device hours in. The invariant was previously documented and unchecked,
// which is the same as not having it.
//
// Global operator new is replaced with a trap that can be armed around the region under test, so
// the rest of the suite (which uses std::vector freely) is unaffected.
#include <cstdio>
#include <cstdlib>
#include <new>

#include "check.h"
#include "partsim/Simulation.h"

using namespace partsim;

namespace {
bool g_trap = false;
long g_count = 0;
size_t g_bytes = 0;
}  // namespace

void* operator new(std::size_t n) {
  if (g_trap) {
    // Not CHECK: by the time we get here the invariant is already broken, and aborting gives a
    // stack trace pointing at the offending call.
    std::fprintf(stderr, "\nFATAL: %zu-byte allocation after init()\n", n);
    std::abort();
  }
  ++g_count;
  g_bytes += n;
  void* p = std::malloc(n);
  if (!p) throw std::bad_alloc();
  return p;
}
void* operator new[](std::size_t n) { return operator new(n); }
void* operator new(std::size_t n, const std::nothrow_t&) noexcept {
  if (g_trap) { std::fprintf(stderr, "\nFATAL: %zu-byte nothrow alloc after init()\n", n); std::abort(); }
  return std::malloc(n);
}
void* operator new[](std::size_t n, const std::nothrow_t& t) noexcept {
  return operator new(n, t);
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void operator delete(void* p, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { std::free(p); }

namespace {
// Static storage, as every platform layer uses.
Simulation g_sim;
}  // namespace

TEST(noalloc_simulation_never_allocates_after_init) {
  // Touch stdio first: its buffers are allocated lazily on the first call, and that would trip
  // the trap for a reason that has nothing to do with the simulation.
  std::printf("       arming allocation trap\n");
  std::fflush(stdout);

  // Measure init on its own. g_count is program-wide, so reporting it as "allocations during
  // init" was simply wrong -- it was dominated by whatever earlier test cases had allocated,
  // and it visibly moved when unrelated tests were added. Init runs on the device too, so it
  // is worth its own assertion rather than a mislabelled number.
  const long beforeInit = g_count;
  CHECK(g_sim.initScene(Simulation::kCube, 3, 1));  // water and sand
  const long initAllocs = g_count - beforeInit;
  CHECK(initAllocs == 0);
  const long before = g_count;

  g_trap = true;
  // Everything the hot path does, plus every state change a running device performs.
  for (int i = 0; i < 200; ++i) g_sim.stepFixed();
  g_sim.render();
  g_sim.advance(0.05f);
  g_sim.setOrientation(Quat{0.0f, 0.2f, 0.0f, 0.98f});
  g_sim.addContainerAccel(Vec3{20.0f, 0.0f, 0.0f});
  for (int i = 0; i < 50; ++i) g_sim.stepFixed();
  g_sim.render();

  g_sim.setScene(1);  // switching to the campfire: emitters and the heat field
  for (int i = 0; i < 100; ++i) g_sim.stepFixed();
  g_sim.render();

  g_sim.setAutoCycle(true);
  for (int i = 0; i < 200; ++i) g_sim.advance(kFixedDt);
  g_sim.render();
  g_trap = false;

  CHECK(g_count == before);  // reached only if nothing aborted
  std::printf("       %ld allocations during init, 0 after\n", initAllocs);
}

TEST(noalloc_every_scene_loads_without_allocating) {
  g_sim.initScene(Simulation::kCube, 0, 2);
  const long before = g_count;
  g_trap = true;
  for (int s = 0; s < sceneCount(); ++s) {
    g_sim.setScene(s);
    for (int i = 0; i < 20; ++i) g_sim.stepFixed();
    g_sim.render();
  }
  g_trap = false;
  CHECK(g_count == before);
}

TEST(noalloc_single_panel_mode_too) {
  g_sim.initScene(Simulation::kSinglePanel, 0, 3);
  const long before = g_count;
  g_trap = true;
  for (int i = 0; i < 100; ++i) g_sim.stepFixed();
  g_sim.render();
  g_trap = false;
  CHECK(g_count == before);
}
