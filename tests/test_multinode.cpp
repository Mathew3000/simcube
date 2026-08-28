// One master, three display nodes, frames carried as bytes.
//
// With a single authoritative simulation, divergence between faces is impossible by construction.
// The failure mode is STALENESS: a node that misses a frame keeps drawing its previous picture
// while the others move on. That is what these tests pin down, because it is visually obvious and
// otherwise completely silent.
#include <cstring>

#include "check.h"
#include "partsim/SimFrame.h"
#include "partsim/Simulation.h"

using namespace partsim;

namespace {

Simulation g_master;

// A display node: exactly what the firmware's display role carries, and nothing else.
struct Node {
  RenderParticles particles;
  HeatBuffer heat;
  Renderer renderer;
  uint32_t lastStep = 0;
  int faces[2] = {0, 0};
};
Node g_n[3];
uint8_t g_frame[frameMaxBytes()];
uint8_t g_held[frameMaxBytes()];

// (-Z,-X) (+Z,+Y) (+X,-Y): the all-adjacent pairing docs/CUBE-PCB.md section 2.1 specifies, so
// each board's two ribbons stay short instead of one crossing the interior.
const int kPairs[3][2] = {{0, 2}, {1, 5}, {3, 4}};

void setup() {
  CHECK(g_master.initScene(Simulation::kCube, 4, 9));  // kettle: water, sand and fire
  for (int n = 0; n < 3; ++n) {
    g_n[n].faces[0] = kPairs[n][0];
    g_n[n].faces[1] = kPairs[n][1];
    CHECK(g_n[n].renderer.init(g_master.geometry(), g_n[n].faces, 2));
    g_n[n].renderer.setExposure(kSplatExposure);
    CHECK(g_n[n].heat.init(g_master.volume()));
    g_n[n].lastStep = 0;
  }
}

int encode(uint32_t step) {
  FrameHeader h;
  h.step = step;
  h.geomHash = geometryHash(g_master.geometry());
  return encodeFrame(h, g_master.particles().view(), g_master.field().view(),
                     g_master.volume().box(), g_frame, frameMaxBytes());
}

bool deliver(int n, const uint8_t* bytes, int len) {
  FrameHeader h;
  if (!decodeFrame(bytes, len, g_master.volume().box(), geometryHash(g_master.geometry()),
                   g_n[n].particles, g_n[n].heat, h)) {
    return false;
  }
  g_n[n].lastStep = h.step;
  return true;
}

uint64_t faceHash(int n) {
  uint64_t h = 1469598103934665603ull;
  for (int k = 0; k < 2; ++k) {
    const int face = g_n[n].faces[k];
    const uint8_t* px = g_n[n].renderer.panelPixels(face);
    h = fnv1a(px, (size_t)g_n[n].renderer.panelTexels(face) * 4u, h);
  }
  return h;
}

void renderAll() {
  for (int n = 0; n < 3; ++n)
    g_n[n].renderer.render(g_n[n].particles.view(), g_n[n].heat.view(), g_master.geometry());
}

}  // namespace

TEST(multinode_three_nodes_cover_the_cube_exactly_once) {
  setup();
  int seen[6] = {0, 0, 0, 0, 0, 0};
  for (int n = 0; n < 3; ++n)
    for (int k = 0; k < 2; ++k) ++seen[g_n[n].faces[k]];
  for (int f = 0; f < 6; ++f) CHECK(seen[f] == 1);

  // And every pair is genuinely adjacent -- opposite faces have antiparallel normals, so a dot
  // product of -1 is the thing to refuse.
  for (int n = 0; n < 3; ++n) {
    const Vec3 a = g_master.geometry().at(g_n[n].faces[0]).n;
    const Vec3 b = g_master.geometry().at(g_n[n].faces[1]).n;
    CHECK(dot(a, b) > -0.5f);
    std::printf("       node %d: faces %d+%d, normals dot %.1f\n", n, g_n[n].faces[0],
                g_n[n].faces[1], dot(a, b));
  }
}

TEST(multinode_reassembles_the_same_cube_as_one_process) {
  // The property the split rests on: three nodes fed one frame produce exactly the pixels a
  // single-process render would, face for face. Quantisation is the only permitted difference, so
  // compare against a single render of the DECODED state rather than of the master's own floats.
  setup();
  for (int i = 0; i < 300; ++i) g_master.stepFixed();
  const int len = encode(1);
  CHECK(len > 0);
  for (int n = 0; n < 3; ++n) CHECK(deliver(n, g_frame, len));
  renderAll();

  // A reference renderer holding all six faces, fed the same decoded state.
  static Renderer all;
  CHECK(all.init(g_master.geometry()));
  all.setExposure(kSplatExposure);
  all.render(g_n[0].particles.view(), g_n[0].heat.view(), g_master.geometry());

  for (int n = 0; n < 3; ++n) {
    for (int k = 0; k < 2; ++k) {
      const int face = g_n[n].faces[k];
      const int bytes = all.panelTexels(face) * 4;
      CHECK(std::memcmp(g_n[n].renderer.panelPixels(face), all.panelPixels(face),
                        (size_t)bytes) == 0);
    }
  }
  std::printf("       3 nodes x 2 faces == a single 6-face render, byte for byte\n");
}

TEST(multinode_a_dropped_frame_freezes_only_that_node) {
  setup();
  for (int i = 0; i < 200; ++i) g_master.stepFixed();
  int len = encode(1);
  for (int n = 0; n < 3; ++n) CHECK(deliver(n, g_frame, len));
  renderAll();
  const uint64_t before[3] = {faceHash(0), faceHash(1), faceHash(2)};

  // Advance the master and deliver to nodes 0 and 2 only. Node 1 misses the frame.
  for (int i = 0; i < 60; ++i) g_master.stepFixed();
  len = encode(2);
  CHECK(deliver(0, g_frame, len));
  CHECK(deliver(2, g_frame, len));
  renderAll();

  CHECK(faceHash(0) != before[0]);  // moved
  CHECK(faceHash(2) != before[2]);  // moved
  CHECK(faceHash(1) == before[1]);  // frozen, holding its last good picture
  CHECK(g_n[1].lastStep == 1u);
  CHECK(g_n[0].lastStep == 2u);
  std::printf("       node 1 held at step %u while 0 and 2 reached %u\n", g_n[1].lastStep,
              g_n[0].lastStep);
}

TEST(multinode_a_corrupted_frame_is_indistinguishable_from_a_drop) {
  // A rejected frame must behave exactly like a missing one -- hold the last picture, do not draw
  // garbage. This is why decodeFrame validates everything before touching state.
  setup();
  for (int i = 0; i < 200; ++i) g_master.stepFixed();
  int len = encode(5);
  for (int n = 0; n < 3; ++n) CHECK(deliver(n, g_frame, len));
  renderAll();
  const uint64_t before = faceHash(1);

  for (int i = 0; i < 60; ++i) g_master.stepFixed();
  len = encode(6);
  std::memcpy(g_held, g_frame, (size_t)len);
  g_held[kFrameHeaderBytes + 100] ^= 0x40;  // one bit, deep in the payload
  CHECK(!deliver(1, g_held, len));
  renderAll();

  CHECK(faceHash(1) == before);
  CHECK(g_n[1].lastStep == 5u);
}

TEST(multinode_a_restarted_node_recovers_on_the_next_frame) {
  setup();
  for (int i = 0; i < 200; ++i) g_master.stepFixed();
  int len = encode(11);
  for (int n = 0; n < 3; ++n) CHECK(deliver(n, g_frame, len));
  renderAll();
  const uint64_t healthy = faceHash(2);

  // Node 2 reboots: empty pools, nothing to draw.
  g_n[2].particles.clear();
  g_n[2].heat.clear();
  g_n[2].lastStep = 0;
  renderAll();
  CHECK(faceHash(2) != healthy);

  // One frame later it is indistinguishable from never having failed -- there is no per-node
  // history to rebuild, which is the advantage of an authoritative master.
  CHECK(deliver(2, g_frame, len));
  renderAll();
  CHECK(faceHash(2) == healthy);
  std::printf("       node 2 recovered fully from one frame\n");
}
