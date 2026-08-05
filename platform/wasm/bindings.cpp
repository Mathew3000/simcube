// C ABI for the browser. Deliberately raw extern "C" plus typed-array views over the WASM
// heap rather than embind.
//
// What crosses the boundary each frame is six 4KB pixel buffers. embind would add ~50KB of
// glue and its natural idioms either copy or rebuild a view per call. With the C ABI the
// panel buffers live at STABLE ADDRESSES for the module's lifetime, so JS constructs each
// Uint8Array once at init and the browser uploads straight out of linear memory -- genuinely
// zero copies per frame.
//
// Consequence, and it is the subtle one: -sALLOW_MEMORY_GROWTH must stay OFF. Growing the
// heap allocates a NEW ArrayBuffer and detaches the old one, silently zero-lengthing every
// cached view -- black textures with no error. Nothing is allocated after init, so growth
// would buy nothing anyway.
#include <cstdint>

#include "partsim/Simulation.h"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define PS_EXPORT EMSCRIPTEN_KEEPALIVE
#else
#define PS_EXPORT
#endif

using namespace partsim;

namespace {
// ~1.2MB of pools; static storage, never the stack.
Simulation g_sim;
float g_basis[12];
float g_stats[4];
bool g_ready = false;
}  // namespace

extern "C" {

PS_EXPORT int ps_init(int mode, int particleCount, uint32_t seed) {
  g_ready = g_sim.init(mode, particleCount, seed);
  return g_ready ? g_sim.particleCount() : 0;
}

PS_EXPORT int ps_particle_count() { return g_ready ? g_sim.particleCount() : 0; }
PS_EXPORT int ps_capacity() { return g_ready ? g_sim.capacity() : 0; }

// Object orientation as a quaternion. The binding converts it to object-space gravity, which
// is the only form the core understands.
PS_EXPORT void ps_set_orientation(float x, float y, float z, float w) {
  if (g_ready) g_sim.setOrientation(Quat{x, y, z, w});
}

// A flick: the acceleration applied TO THE CONTAINER, in world space, with the object's
// current orientation so it can be rotated into object space. Shoving the cube right piles the
// water left -- see Simulation::addContainerAccel for why that is the correct sign.
PS_EXPORT void ps_add_jerk(float qx, float qy, float qz, float qw, float ax, float ay,
                           float az) {
  if (g_ready) g_sim.addContainerAccelWorld(Quat{qx, qy, qz, qw}, Vec3{ax, ay, az});
}

PS_EXPORT int ps_advance(float dtSeconds) { return g_ready ? g_sim.advance(dtSeconds) : 0; }
PS_EXPORT void ps_render() { if (g_ready) g_sim.render(); }

PS_EXPORT int ps_panel_count() { return g_ready ? g_sim.geometry().count() : 0; }
PS_EXPORT int ps_panel_w(int i) { return g_ready ? (int)g_sim.geometry().at(i).w : 0; }
PS_EXPORT int ps_panel_h(int i) { return g_ready ? (int)g_sim.geometry().at(i).h : 0; }

// RGBA8, stable for the module's lifetime once ps_init has run.
PS_EXPORT const uint8_t* ps_panel_ptr(int i) {
  return g_ready ? g_sim.renderer().panelPixels(i) : nullptr;
}

// origin, u, v, n -- 12 floats. The frontend places its quads from THIS rather than
// hardcoding a cube, so the physics and the visuals cannot disagree about panel orientation.
PS_EXPORT const float* ps_panel_basis(int i) {
  if (!g_ready) return g_basis;
  const Panel& p = g_sim.geometry().at(i);
  const float v[12] = {p.origin.x, p.origin.y, p.origin.z, p.u.x, p.u.y, p.u.z,
                       p.v.x,      p.v.y,      p.v.z,      p.n.x, p.n.y, p.n.z};
  for (int k = 0; k < 12; ++k) g_basis[k] = v[k];
  return g_basis;
}

// [simMs, renderMs, particles, substeps]
PS_EXPORT const float* ps_stats() {
  g_stats[0] = g_sim.stats.simMs;
  g_stats[1] = g_sim.stats.renderMs;
  g_stats[2] = (float)g_sim.stats.particles;
  g_stats[3] = (float)g_sim.stats.substeps;
  return g_stats;
}

PS_EXPORT void ps_set_stats_timing(float simMs, float renderMs) {
  g_sim.stats.simMs = simMs;
  g_sim.stats.renderMs = renderMs;
}

PS_EXPORT void ps_set_palette(int index) {
  if (g_ready) g_sim.setPalette(&paletteAt(index));
}
PS_EXPORT int ps_palette_count() { return paletteCount(); }

PS_EXPORT uint32_t ps_state_hash() { return g_ready ? g_sim.stateHash() : 0u; }

// Runs the scripted golden sequence and returns the state hash. The host build runs the
// identical core function, so a mismatch is a genuine cross-target divergence.
PS_EXPORT uint32_t ps_golden_hash(int steps, int particleCount, uint32_t seed) {
  const uint32_t h = goldenHash(g_sim, steps, particleCount, seed);
  g_ready = true;
  return h;
}

// FNV-1a over every panel's rendered RGBA, folded to 32 bits. Lets the node harness prove the
// WASM build produces pixel-identical output to the host, rather than merely similar.
PS_EXPORT uint32_t ps_pixel_hash() {
  if (!g_ready) return 0u;
  uint64_t h = 1469598103934665603ull;
  for (int k = 0; k < g_sim.geometry().count(); ++k) {
    const Panel& p = g_sim.geometry().at(k);
    h = fnv1a(g_sim.renderer().panelPixels(k), (size_t)p.w * (size_t)p.h * 4u, h);
  }
  return (uint32_t)(h ^ (h >> 32));
}

}  // extern "C"
