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

#include "partsim/SimFrame.h"
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

// --- multi-node preview ------------------------------------------------------------------------
// One master simulation plus up to three display nodes, so the browser shows what the real
// four-board cube does -- including what happens when a frame goes missing.
//
// JS carries the frame bytes between them, and that placement is the point: fault injection becomes
// honest rather than simulated, because JavaScript physically holds the buffer and can drop it,
// delay it or corrupt a byte. The protocol is the same SimFrame the firmware uses.
constexpr int kMaxNodes = 3;
struct DisplayNode {
  RenderParticles particles;
  HeatBuffer heat;
  Renderer renderer;
  uint32_t lastStep = 0;
  bool everReceived = false;
  int faces[2] = {0, 0};
  int faceCount = 0;
};
DisplayNode g_nodes[kMaxNodes];
int g_nodeCount = 0;
uint32_t g_masterStep = 0;
uint8_t g_frame[frameMaxBytes()];
int g_frameLen = 0;
bool g_nodesReady = false;
float g_basis[12];
float g_stats[4];
bool g_ready = false;
}  // namespace

extern "C" {

PS_EXPORT int ps_init(int mode, int particleCount, uint32_t seed, int panelRes) {
  g_ready = g_sim.init(mode, particleCount, seed, panelRes);
  return g_ready ? 1 : 0;
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

// Panel resolution and world-units-per-texel. The frontend needs the pitch to size anything in
// world space -- cubeview.js used to multiply a texel count by 1.0 and call it a world size, which
// was right only while the pitch happened to be 1.0.
PS_EXPORT int ps_panel_res() { return g_ready ? g_sim.panelRes() : 0; }
PS_EXPORT float ps_panel_pitch() { return g_ready ? g_sim.pitch() : 1.0f; }

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

PS_EXPORT int ps_scene_count() { return sceneCount(); }
PS_EXPORT int ps_scene() { return g_ready ? g_sim.scene() : 0; }
PS_EXPORT void ps_set_scene(int id) { if (g_ready) g_sim.setScene(id); }

// Returns a pointer to a NUL-terminated string in the WASM heap; JS reads it out of HEAPU8.
// Cheaper and simpler than marshalling strings, and the table lives in rodata anyway.
PS_EXPORT const char* ps_scene_name(int id) { return sceneAt(id).name; }

PS_EXPORT void ps_set_auto_cycle(int on) { if (g_ready) g_sim.setAutoCycle(on != 0); }
PS_EXPORT int ps_auto_cycle() { return (g_ready && g_sim.autoCycle()) ? 1 : 0; }

// Load a scene as part of init, rather than a bare particle count.
PS_EXPORT int ps_init_scene(int mode, int sceneId, uint32_t seed, int panelRes) {
  g_ready = g_sim.initScene(mode, sceneId, seed, panelRes);
  return g_ready ? 1 : 0;
}

PS_EXPORT void ps_set_palette(int index) {
  if (g_ready) g_sim.setPalette(&paletteAt(index));
}
PS_EXPORT int ps_palette_count() { return paletteCount(); }

PS_EXPORT uint32_t ps_state_hash() { return g_ready ? g_sim.stateHash() : 0u; }

// Runs the scripted golden sequence and returns the state hash. The host build runs the
// identical core function, so a mismatch is a genuine cross-target divergence.
PS_EXPORT uint32_t ps_golden_hash(int steps, uint32_t seed) {
  const uint32_t h = goldenHash(g_sim, steps, seed);
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

// --- multi-node preview ------------------------------------------------------------------------

// Master plus `nodes` display nodes. Faces are paired the way the real cube pairs them: adjacent,
// never opposite, so each board's HUB75 ribbons stay short (docs/CUBE-PCB.md section 2.1).
PS_EXPORT int ps_node_init(int nodes, int panelRes) {
  if (nodes < 1 || nodes > kMaxNodes) return 0;
  if (!g_sim.initScene(Simulation::kCube, 0, 0xC0FFEEu, panelRes)) return 0;
  g_ready = true;

  // (-Z,-X) (+Z,+Y) (+X,-Y): an all-adjacent perfect matching of the cube's faces.
  static const int kPairs[3][2] = {{0, 2}, {1, 5}, {3, 4}};
  const int perNode = 6 / nodes;
  int next = 0;
  for (int n = 0; n < nodes; ++n) {
    DisplayNode& d = g_nodes[n];
    d.faceCount = 0;
    for (int k = 0; k < perNode && k < 2; ++k) {
      d.faces[k] = (nodes == 3) ? kPairs[n][k] : next++;
      ++d.faceCount;
    }
    if (!d.renderer.init(g_sim.geometry(), d.faces, d.faceCount)) return 0;
    d.renderer.setExposure(kSplatExposure);
    if (!d.heat.init(g_sim.volume())) return 0;
    d.particles.clear();
    d.lastStep = 0;
    d.everReceived = false;
  }
  g_nodeCount = nodes;
  g_masterStep = 0;
  g_frameLen = 0;
  g_nodesReady = true;
  return 1;
}

PS_EXPORT int ps_node_count() { return g_nodesReady ? g_nodeCount : 0; }
PS_EXPORT int ps_node_face_count(int n) {
  return (g_nodesReady && n >= 0 && n < g_nodeCount) ? g_nodes[n].faceCount : 0;
}
PS_EXPORT int ps_node_face(int n, int k) {
  if (!g_nodesReady || n < 0 || n >= g_nodeCount) return -1;
  return (k >= 0 && k < g_nodes[n].faceCount) ? g_nodes[n].faces[k] : -1;
}

// The master advances and encodes. Returns the frame length, or 0 on failure.
PS_EXPORT int ps_node_step(int substeps) {
  if (!g_nodesReady) return 0;
  for (int i = 0; i < substeps; ++i) {
    g_sim.stepFixed();
    ++g_masterStep;
  }
  FrameHeader h;
  h.step = g_masterStep;
  h.geomHash = geometryHash(g_sim.geometry());
  g_frameLen = encodeFrame(h, g_sim.particles().view(), g_sim.field().view(),
                           g_sim.volume().box(), g_frame, frameMaxBytes());
  return g_frameLen;
}

// The encoded frame, for JS to copy, delay, corrupt or discard before delivering it.
PS_EXPORT uint8_t* ps_node_frame_ptr() { return g_frame; }
PS_EXPORT int ps_node_frame_len() { return g_frameLen; }
PS_EXPORT uint32_t ps_node_master_step() { return g_masterStep; }

// Hands a frame to one node. 1 if accepted, 0 if rejected -- and a rejected frame leaves the
// node untouched, so it keeps drawing its previous picture.
PS_EXPORT int ps_node_deliver(int n, const uint8_t* frame, int len) {
  if (!g_nodesReady || n < 0 || n >= g_nodeCount) return 0;
  DisplayNode& d = g_nodes[n];
  FrameHeader h;
  if (!decodeFrame(frame, len, g_sim.volume().box(), geometryHash(g_sim.geometry()), d.particles,
                   d.heat, h)) {
    return 0;
  }
  d.lastStep = h.step;
  d.everReceived = true;
  return 1;
}

// The failure mode that matters. With one authoritative simulation divergence is impossible, but a
// node that misses frames goes STALE: its faces freeze while the others keep moving.
PS_EXPORT uint32_t ps_node_last_step(int n) {
  return (g_nodesReady && n >= 0 && n < g_nodeCount) ? g_nodes[n].lastStep : 0u;
}
PS_EXPORT int ps_node_ever_received(int n) {
  return (g_nodesReady && n >= 0 && n < g_nodeCount && g_nodes[n].everReceived) ? 1 : 0;
}

PS_EXPORT void ps_node_render(int n) {
  if (!g_nodesReady || n < 0 || n >= g_nodeCount) return;
  DisplayNode& d = g_nodes[n];
  d.renderer.render(d.particles.view(), d.heat.view(), g_sim.geometry());
}

PS_EXPORT const uint8_t* ps_node_panel_ptr(int n, int face) {
  if (!g_nodesReady || n < 0 || n >= g_nodeCount) return nullptr;
  return g_nodes[n].renderer.panelPixels(face);
}

// Drops a node to its boot state, so the page can show a restart recovering.
PS_EXPORT void ps_node_reset(int n) {
  if (!g_nodesReady || n < 0 || n >= g_nodeCount) return;
  g_nodes[n].particles.clear();
  g_nodes[n].heat.clear();
  g_nodes[n].lastStep = 0;
  g_nodes[n].everReceived = false;
}

}  // extern "C"
