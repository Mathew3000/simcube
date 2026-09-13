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
#include "partsim/SpillChain.h"

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

// --- beaker chain ------------------------------------------------------------------------------
// A third set of state alongside g_sim and g_nodes, never a repurposing of either: the multi-node
// preview is ONE simulation drawn by several nodes, and a beaker chain is SEVERAL simulations that
// exchange fluid. Sharing state between them would mean one of the two stops meaning what it says.
//
// Six, with one beaker of headroom under a HARD ceiling of seven.
//
// The brief put the limit at four and said six would not fit. Measured instead of assumed, by
// building the module at successive counts: seven links and runs, and eight fails at link time with
//
//     wasm-ld: error: initial memory too small, 34740704 bytes needed
//
// against INITIAL_MEMORY of 33554432 -- so a beaker costs 3.41MB here (Simulation 3.25MB with
// chroma, plus this file's outbound and inbound packet buffers) and the budget is not what the
// arithmetic in the brief assumed. A display node is not a Simulation; it carries RenderState's
// draw-only containers, which is where the missing room was.
//
// Six rather than seven because a ceiling with nothing under it breaks on the next member anybody
// adds, and because this is a static-data limit: exceeding it is a LINK error, not a runtime one,
// so there is no graceful degradation to fall back on. ALLOW_MEMORY_GROWTH stays off regardless --
// growth swaps in a new ArrayBuffer and detaches every panel view JS holds.
#ifndef PARTSIM_MAX_BEAKERS
#define PARTSIM_MAX_BEAKERS 6
#endif
constexpr int kMaxBeakers = PARTSIM_MAX_BEAKERS;

// The carrier, with JS on the other side of it.
//
// send() appends to a byte buffer JS reads out of the heap; poll() drains a queue JS pushes into.
// That placement is the entire argument for doing this in the browser at all: dropping a packet in
// JS is a packet that never arrives, not a flag the C++ side agrees to honour. The same honesty
// nodes.html gets for frames, applied to the thing whose failure mode is arithmetic rather than
// visible -- a lost spill packet does not freeze a face, it quietly drains a ring.
class JsSpillTransport final : public SpillTransport {
 public:
  // Each packet is written length-prefixed (1 byte: kSpillMaxPayload is 250, so it fits) so JS can
  // split a frame's worth back into individual packets. The prefix is framing for JS only -- it is
  // never handed to decodeSpill, and ps_beaker_deliver takes an explicit length.
  static constexpr int kOutCap = 8192;
  static constexpr int kInSlots = 48;

  bool send(const uint8_t* bytes, int len) override {
    if (len <= 0 || len > kSpillMaxPayload) return false;
    if (outLen_ + 1 + len > kOutCap) {
      ++refusedOut_;  // the carrier refused it: a real "the radio said no", counted not swallowed
      return false;
    }
    out_[outLen_++] = (uint8_t)len;
    for (int i = 0; i < len; ++i) out_[outLen_ + i] = bytes[i];
    outLen_ += len;
    return true;
  }

  int poll(uint8_t* buf, int cap) override {
    if (inCount_ == 0) return 0;
    const int len = inLen_[inHead_];
    if (len > cap) return 0;
    for (int i = 0; i < len; ++i) buf[i] = in_[inHead_][i];
    inHead_ = (inHead_ + 1) % kInSlots;
    --inCount_;
    return len;
  }

  const char* name() const override { return "js"; }

  // Queue an arrival. False means the inbound ring was full -- which is a carrier fault of exactly
  // the kind ESP-NOW has, so it is counted here rather than reported as a decode failure.
  bool deliver(const uint8_t* bytes, int len) {
    if (len <= 0 || len > kSpillMaxPayload) return false;
    if (inCount_ >= kInSlots) { ++overrunIn_; return false; }
    const int s = (inHead_ + inCount_) % kInSlots;
    for (int i = 0; i < len; ++i) in_[s][i] = bytes[i];
    inLen_[s] = len;
    ++inCount_;
    return true;
  }

  void beginFrame() { outLen_ = 0; }
  const uint8_t* outPtr() const { return out_; }
  int outLen() const { return outLen_; }
  uint32_t refusedOut() const { return refusedOut_; }
  uint32_t overrunIn() const { return overrunIn_; }
  void reset() { outLen_ = 0; inHead_ = 0; inCount_ = 0; refusedOut_ = 0; overrunIn_ = 0; }

 private:
  uint8_t out_[kOutCap];
  int outLen_ = 0;
  uint8_t in_[kInSlots][kSpillMaxPayload];
  int inLen_[kInSlots] = {};
  int inHead_ = 0, inCount_ = 0;
  uint32_t refusedOut_ = 0, overrunIn_ = 0;
};

struct Beaker {
  Simulation sim;
  SpillChain chain;
  JsSpillTransport wire;
  uint16_t dyeR = 0, dyeG = 0;  // the beaker's IDENTITY, restored by a reset -- D64
};
Beaker g_beakers[kMaxBeakers];
int g_beakerCount = 0;
bool g_beakersReady = false;
constexpr int kBeakerStatCount = 13;
uint32_t g_beakerStats[kBeakerStatCount];
float g_beakerChroma[4];
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

// --- ink -----------------------------------------------------------------------------------
// Exported unconditionally so a page can ASK whether this module has ink, rather than failing to
// find the symbol and dying. ps_ink_enabled() is the whole protocol.
PS_EXPORT int ps_ink_enabled() {
#if PARTSIM_ENABLE_INK
  return 1;
#else
  return 0;
#endif
}

PS_EXPORT int ps_ink_dim() {
#if PARTSIM_ENABLE_INK
  return kInkDim;
#else
  return 0;
#endif
}

PS_EXPORT int ps_ink_channels() {
#if PARTSIM_ENABLE_INK
  return kInkChannels;
#else
  return 0;
#endif
}

// Object-space position, world units. The browser rotates the OBJECT and leaves gravity pointing
// world-down, so a drop aimed at the top of the screen has to arrive here already in the cube's
// own frame -- the caller passes object coordinates, same as every other entry point.
PS_EXPORT void ps_ink_inject(float x, float y, float z, float radius, int channel, int amount) {
#if PARTSIM_ENABLE_INK
  if (g_ready) g_sim.injectInk(Vec3{x, y, z}, radius, channel, amount);
#else
  (void)x; (void)y; (void)z; (void)radius; (void)channel; (void)amount;
#endif
}

PS_EXPORT void ps_ink_vortons(float x, float y, float z, float ax, float ay, float az,
                              float radius, int strength, int life) {
#if PARTSIM_ENABLE_INK
  if (g_ready) g_sim.spawnInkVortons(Vec3{x, y, z}, Vec3{ax, ay, az}, radius, strength, life);
#else
  (void)x; (void)y; (void)z; (void)ax; (void)ay; (void)az;
  (void)radius; (void)strength; (void)life;
#endif
}

PS_EXPORT void ps_ink_clear() {
#if PARTSIM_ENABLE_INK
  if (g_ready) g_sim.clearInk();
#endif
}

// Peak concentration, 0 when the volume is clear. The page uses it to show whether anything is
// actually in there, which is the first question when a render comes up black.
PS_EXPORT int ps_ink_peak() {
#if PARTSIM_ENABLE_INK
  return g_ready ? (int)g_sim.ink().peak() : 0;
#else
  return 0;
#endif
}

PS_EXPORT int ps_ink_vorton_count() {
#if PARTSIM_ENABLE_INK
  return g_ready ? g_sim.ink().vortonCount() : 0;
#else
  return 0;
#endif
}

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


// --- beaker chain --------------------------------------------------------------------------
//
// N independent simulations, each an open-topped vessel with its own dye, chained by JS. The
// module never knows the chain ORDER: it hands out one beaker's packets and accepts another's,
// and which is which is a JS array. That is deliberate -- chain order is user configuration on a
// real cube, and putting it here would make the browser page prove something the firmware does
// not do.

#if PARTSIM_ENABLE_CHROMA
#define PS_BEAKER_CHROMA 1
#else
#define PS_BEAKER_CHROMA 0
#endif

// 1 when this artifact was built with the beaker tier. beakers.html refuses to run against the
// default module rather than showing a colourless ring that looks like a broken mix -- the
// failure is otherwise indistinguishable from dye that will not diffuse.
PS_EXPORT int ps_beaker_has_chroma() { return PS_BEAKER_CHROMA; }
PS_EXPORT int ps_beaker_max() { return kMaxBeakers; }
PS_EXPORT int ps_beaker_stat_count() { return kBeakerStatCount; }

namespace {
// 60% of what the VOLUME holds, not 60% of the particle pool.
//
// D63 states the rule as (kMaxParticles * 3) / 5, which is 60% on the device because the device
// pool (512) is smaller than the volume's capacity. In a host/WASM build the pool is 16384 and the
// volume holds ~2100, so that same expression asks for 9830, Simulation::init clamps it to
// capacity * 9/10, and the beaker comes up 90% full -- which is the exact condition D63 exists to
// prevent. Taking the fraction of whichever is binding keeps the RULE rather than its arithmetic.
int beakerFillFor(Simulation& sim) {
  const int byPool = (kMaxParticles * 3) / 5;
  const int byVolume = (sim.capacity() * 3) / 5;
  return byPool < byVolume ? byPool : byVolume;
}

bool beakerFill(Beaker& b, int panelRes) {
  // Once to build the geometry and learn the capacity, again to fill to a fraction of it. init()
  // deliberately leaves the open face alone, so the second call refills rather than re-seals.
  if (!b.sim.init(Simulation::kCube, 0, 0xBEA6u, panelRes)) return false;
  b.sim.setOpenFace(kOpenPosY);
  if (!b.sim.init(Simulation::kCube, beakerFillFor(b.sim), 0xBEA6u, panelRes)) return false;
  // No scene drift: auto-cycle would refill a beaker mid-pour, and the fill level is the thing
  // being watched. init() clears it anyway; stated because it is a requirement, not a default.
  b.sim.setAutoCycle(false);
#if PARTSIM_ENABLE_CHROMA
  b.sim.setDye(b.dyeR, b.dyeG);  // a refill gets the beaker's own dye back, not its mixture
#endif
  return true;
}
}  // namespace

PS_EXPORT int ps_beaker_init(int count, int panelRes) {
  if (count < 1 || count > kMaxBeakers) return 0;
  for (int i = 0; i < count; ++i) {
    Beaker& b = g_beakers[i];
    b.chain = SpillChain{};
    b.wire.reset();
    b.dyeR = 0;
    b.dyeG = 0;
    // Every beaker is addressed, even though this page delivers point-to-point and so could not
    // duplicate anything. The hardware has no such option -- the radio is a broadcast and an
    // unaddressed ring of three injects what BOTH neighbours poured (D66) -- and the browser is
    // where that behaviour is supposed to be debuggable. Addressing it here costs nothing and
    // means `foreign` is a live counter rather than a field that is structurally always zero.
    b.chain.setChainPosition(i, count);
    if (!beakerFill(b, panelRes)) return 0;
  }
  g_beakerCount = count;
  g_beakersReady = true;
  return 1;
}

PS_EXPORT int ps_beaker_count() { return g_beakersReady ? g_beakerCount : 0; }

namespace {
inline Beaker* beakerAt(int i) {
  if (!g_beakersReady || i < 0 || i >= g_beakerCount) return nullptr;
  return &g_beakers[i];
}
}  // namespace

// 0-255 per channel, scaled into the 8.8 fixed point core stores dye in. Blue is what is left
// over: (0,0) is blue, (255,0) red, (0,255) green -- see Particles::cr.
PS_EXPORT void ps_beaker_set_dye(int i, int r, int g) {
  Beaker* b = beakerAt(i);
  if (!b) return;
  const int rr = r < 0 ? 0 : (r > 255 ? 255 : r);
  const int gg = g < 0 ? 0 : (g > 255 ? 255 : g);
  // Clamped as a PAIR: cr + cg must stay within kChromaOne or the implied blue goes negative.
  const int sum = rr + gg;
  b->dyeR = (uint16_t)((sum > 255 ? rr * 255 / sum : rr) * 256);
  b->dyeG = (uint16_t)((sum > 255 ? gg * 255 / sum : gg) * 256);
#if PARTSIM_ENABLE_CHROMA
  b->sim.setDye(b->dyeR, b->dyeG);
#else
  (void)0;
#endif
}

PS_EXPORT int ps_beaker_dye_r(int i) { Beaker* b = beakerAt(i); return b ? b->dyeR / 256 : 0; }
PS_EXPORT int ps_beaker_dye_g(int i) { Beaker* b = beakerAt(i); return b ? b->dyeG / 256 : 0; }

// Refill and restore the configured dye. Deliberately does NOT reset the chain counters: a
// receiver downstream sees totalOut restart at zero and has to cope, exactly as it must when a
// board reboots (D60), and hiding that here would remove the only place it can be watched.
PS_EXPORT int ps_beaker_reset(int i) {
  Beaker* b = beakerAt(i);
  if (!b) return 0;
  return beakerFill(*b, b->sim.panelRes()) ? 1 : 0;
}

// Where a beaker stands in the ring, and how long the ring is. Exposed rather than fixed at init
// because chain order is the thing a user configures on a real cube -- and because a page that
// cannot express a WRONG order cannot show what a wrong one does.
//
// A length below 2 is unaddressed: the beaker accepts packets from anyone, which is the pre-D66
// behaviour and the bench-with-two-boards case.
PS_EXPORT void ps_beaker_set_chain_position(int i, int id, int length) {
  Beaker* b = beakerAt(i);
  if (b) b->chain.setChainPosition(id, length);
}
PS_EXPORT int ps_beaker_chain_id(int i) { Beaker* b = beakerAt(i); return b ? b->chain.chainId() : 0; }
PS_EXPORT int ps_beaker_chain_len(int i) { Beaker* b = beakerAt(i); return b ? b->chain.chainLength() : 0; }
PS_EXPORT int ps_beaker_upstream(int i) { Beaker* b = beakerAt(i); return b ? b->chain.upstream() : -1; }

PS_EXPORT void ps_beaker_orient(int i, float x, float y, float z, float w) {
  Beaker* b = beakerAt(i);
  if (b) b->sim.setOrientation(Quat{x, y, z, w});
}

PS_EXPORT void ps_beaker_jerk(int i, float qx, float qy, float qz, float qw, float ax, float ay,
                              float az) {
  Beaker* b = beakerAt(i);
  if (b) b->sim.addContainerAccelWorld(Quat{qx, qy, qz, qw}, Vec3{ax, ay, az});
}

// Advance one displayed frame and pump. Returns the bytes now waiting in the outbox.
//
// advance() clears the spill queue once per FRAME and accumulates across every substep it runs, so
// one pump after it sees the whole pour. stepFixed() clears per STEP, and a pump after a loop of
// those sees only the last one -- half the pour crossing the air as copies, with every counter at
// both ends agreeing it was fine (D62). SpillChain::Stats::unsent is what names it, and
// ps_beaker_stats_ptr puts it on the page.
PS_EXPORT int ps_beaker_step(int i, float dtSeconds) {
  Beaker* b = beakerAt(i);
  if (!b) return 0;
  b->wire.beginFrame();
  b->sim.advance(dtSeconds);
  b->chain.pump(b->sim, b->wire);
  return b->wire.outLen();
}

// This frame's packets, length-prefixed, for JS to split, drop, delay or corrupt.
PS_EXPORT const uint8_t* ps_beaker_out_ptr(int i) {
  Beaker* b = beakerAt(i);
  return b ? b->wire.outPtr() : nullptr;
}
PS_EXPORT int ps_beaker_out_len(int i) { Beaker* b = beakerAt(i); return b ? b->wire.outLen() : 0; }

// Hand one packet over. It is queued, not decoded: the next pump polls it, so a corrupted packet
// fails validation inside SpillChain exactly where a corrupted radio frame would.
PS_EXPORT int ps_beaker_deliver(int i, const uint8_t* bytes, int len) {
  Beaker* b = beakerAt(i);
  if (!b) return 0;
  return b->wire.deliver(bytes, len) ? 1 : 0;
}

PS_EXPORT int ps_beaker_fill(int i) { Beaker* b = beakerAt(i); return b ? b->sim.particleCount() : 0; }
PS_EXPORT int ps_beaker_capacity(int i) { Beaker* b = beakerAt(i); return b ? b->sim.capacity() : 0; }

// Everything a chain can be wrong about, in one read:
// [0] packetsOut [1] packetsIn [2] particlesOut [3] particlesIn [4] bad [5] rejected
// [6] madeUp [7] unsent [8] shortfall [9] owed [10] queueDropped [11] carrierRefused [12] foreign
PS_EXPORT const uint32_t* ps_beaker_stats_ptr(int i) {
  Beaker* b = beakerAt(i);
  for (int k = 0; k < kBeakerStatCount; ++k) g_beakerStats[k] = 0u;
  if (!b) return g_beakerStats;
  const SpillChain::Stats& st = b->chain.stats();
  g_beakerStats[0] = st.packetsOut;
  g_beakerStats[1] = st.packetsIn;
  g_beakerStats[2] = st.particlesOut;
  g_beakerStats[3] = st.particlesIn;
  g_beakerStats[4] = st.bad;
  g_beakerStats[5] = st.rejected;
  g_beakerStats[6] = st.madeUp;
  g_beakerStats[7] = st.unsent;
  g_beakerStats[8] = b->chain.shortfall();
  g_beakerStats[9] = b->chain.owed();
  g_beakerStats[10] = b->sim.spill().dropped;
  g_beakerStats[11] = b->wire.refusedOut() + b->wire.overrunIn();
  // Packets heard from a cube that is not this one's upstream. Expected on a broadcast, and zero
  // on a point-to-point delivery -- which is exactly why the page can switch between the two.
  g_beakerStats[12] = st.foreign;
  return g_beakerStats;
}

// The beaker's CURRENT mixed colour and the dye weight behind it: [r, g, b] each 0..1, then the
// total dye weight in particle-units. The last one is the conservation check -- mixing is a convex
// lerp, so summing cr and cg over every beaker in a ring must stay put while colour moves.
PS_EXPORT const float* ps_beaker_chroma_ptr(int i) {
  for (int k = 0; k < 4; ++k) g_beakerChroma[k] = 0.0f;
  Beaker* b = beakerAt(i);
  if (!b) return g_beakerChroma;
#if PARTSIM_ENABLE_CHROMA
  const Particles& p = b->sim.particles();
  // Doubles would promote; sums stay in float, which is what the page displays anyway.
  float sr = 0.0f, sg = 0.0f;
  for (int k = 0; k < p.n; ++k) {
    sr += (float)p.cr[k];
    sg += (float)p.cg[k];
  }
  const float one = (float)kChromaOne;
  const float n = (float)(p.n > 0 ? p.n : 1);
  g_beakerChroma[0] = sr / one / n;
  g_beakerChroma[1] = sg / one / n;
  g_beakerChroma[2] = 1.0f - g_beakerChroma[0] - g_beakerChroma[1];
  // Red and green weight only: blue is implied, so it is not independent evidence.
  g_beakerChroma[3] = (sr + sg) / one;
#endif
  return g_beakerChroma;
}

PS_EXPORT void ps_beaker_render(int i) {
  Beaker* b = beakerAt(i);
  if (b) b->sim.render();
}

PS_EXPORT const uint8_t* ps_beaker_panel_ptr(int i, int face) {
  Beaker* b = beakerAt(i);
  return b ? b->sim.renderer().panelPixels(face) : nullptr;
}

// Geometry, from beaker 0. Every beaker is the same 32-unit box by construction -- that identity
// is what lets a spilled particle keep its object-space position across the wire (D58) -- so one
// answer serves the whole ring and a per-beaker accessor would imply they could differ.
PS_EXPORT int ps_beaker_panel_count() { return g_beakersReady ? g_beakers[0].sim.geometry().count() : 0; }
PS_EXPORT int ps_beaker_panel_w(int f) {
  return g_beakersReady ? (int)g_beakers[0].sim.geometry().at(f).w : 0;
}
PS_EXPORT int ps_beaker_panel_h(int f) {
  return g_beakersReady ? (int)g_beakers[0].sim.geometry().at(f).h : 0;
}
PS_EXPORT const float* ps_beaker_panel_basis(int f) {
  if (!g_beakersReady) return g_basis;
  const Panel& p = g_beakers[0].sim.geometry().at(f);
  const float v[12] = {p.origin.x, p.origin.y, p.origin.z, p.u.x, p.u.y, p.u.z,
                       p.v.x,      p.v.y,      p.v.z,      p.n.x, p.n.y, p.n.z};
  for (int k = 0; k < 12; ++k) g_basis[k] = v[k];
  return g_basis;
}
PS_EXPORT int ps_beaker_open_face() { return g_beakersReady ? g_beakers[0].sim.openFace() : -1; }
PS_EXPORT float ps_beaker_rest_spacing() { return kRestSpacing; }

}  // extern "C"
