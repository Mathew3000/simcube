// The wire format between the master and the display nodes.
//
// Three properties matter, in this order:
//   1. A rejected frame must change nothing, so a node holds its last good picture rather than
//      drawing garbage.
//   2. Encoding must be byte-deterministic, or the format cannot be shared across three targets.
//   3. Quantisation error must be small against the rest spacing, or the picture degrades.
#include <cstring>

#include "check.h"
#include "partsim/SimFrame.h"
#include "partsim/Simulation.h"

using namespace partsim;

namespace {

Simulation g_sim;
RenderParticles g_rp;
HeatBuffer g_hb;
uint8_t g_buf[frameMaxBytes()];
uint8_t g_buf2[frameMaxBytes()];

// A settled kettle: water, sand and an active heat field at once, so no path goes unexercised.
void settle(int scene = 4, int steps = 400) {
  CHECK(g_sim.initScene(Simulation::kCube, scene, 11));
  for (int i = 0; i < steps; ++i) g_sim.stepFixed();
  CHECK(g_hb.init(g_sim.volume()));
}

int encodeCurrent(uint8_t* out, uint32_t step = 7) {
  FrameHeader h;
  h.step = step;
  h.geomHash = geometryHash(g_sim.geometry());
  h.paletteA = 0;
  h.paletteB = 1;
  h.blend = 128;
  return encodeFrame(h, g_sim.particles().view(), g_sim.field().view(), g_sim.volume().box(), out,
                     frameMaxBytes());
}

}  // namespace

TEST(simframe_round_trips_within_quantisation_bounds) {
  settle();
  const int len = encodeCurrent(g_buf);
  CHECK(len > 0);

  FrameHeader got;
  CHECK(decodeFrame(g_buf, len, g_sim.volume().box(), geometryHash(g_sim.geometry()), g_rp, g_hb,
                    got));
  CHECK(got.step == 7u);
  CHECK(got.particles == (uint16_t)g_sim.particleCount());
  CHECK(got.blend == 128);
  CHECK(g_rp.count() == g_sim.particleCount());

  // Position error against the 1.5-unit rest spacing, and velocity against the 72 unit/s CFL cap.
  const Particles& p = g_sim.particles();
  const ParticleView v = g_rp.view();
  float maxPos = 0.0f, maxVel = 0.0f;
  for (int i = 0; i < p.n; ++i) {
    maxPos = pmax(maxPos, length(p.pos(i) - Vec3{v.x[i], v.y[i], v.z[i]}));
    maxVel = pmax(maxVel, length(p.vel(i) - Vec3{v.vx[i], v.vy[i], v.vz[i]}));
    CHECK(p.mat[i] == v.mat[i]);
  }
  std::printf("       %d particles in %d B (%.1f B each); max err: pos %.5f u, vel %.3f u/s\n",
              p.n, len, (float)len / (float)p.n, maxPos, maxVel);
  // Position: the quantisation step is 32/65535 = 0.00049 units, so under a thousandth.
  CHECK(maxPos < 0.002f);
  // Velocity: int8 over +-72 u/s is a 0.57 u/s step. Over one 1/30 s frame of interpolation that
  // is 0.019 units of splat displacement, against a 1.5-unit rest spacing.
  CHECK(maxVel < 0.6f);
  CHECK(maxVel * (1.0f / 30.0f) < 0.05f * kRestSpacing);
}

TEST(simframe_encoding_is_byte_deterministic) {
  // The format is shared across host, browser and device. If encoding the same state twice can
  // differ, nothing downstream can be compared.
  settle();
  const int a = encodeCurrent(g_buf);
  const int b = encodeCurrent(g_buf2);
  CHECK(a == b);
  CHECK(a > 0);
  CHECK(std::memcmp(g_buf, g_buf2, (size_t)a) == 0);
}

TEST(simframe_rejects_a_corrupted_frame_without_touching_state) {
  settle();
  const int len = encodeCurrent(g_buf);
  FrameHeader h;
  CHECK(decodeFrame(g_buf, len, g_sim.volume().box(), geometryHash(g_sim.geometry()), g_rp, g_hb, h));
  const int goodCount = g_rp.count();
  CHECK(goodCount > 0);

  // Flip one bit deep in the particle payload.
  const int victim = kFrameHeaderBytes + 40;
  g_buf[victim] ^= 0x01;
  FrameHeader bad;
  CHECK(!decodeFrame(g_buf, len, g_sim.volume().box(), geometryHash(g_sim.geometry()), g_rp, g_hb,
                     bad));
  // The previous frame must still be intact -- that is the whole point of validating first.
  CHECK(g_rp.count() == goodCount);
  g_buf[victim] ^= 0x01;
}

TEST(simframe_rejects_truncation_bad_magic_and_bad_version) {
  settle();
  const int len = encodeCurrent(g_buf);
  const Aabb box = g_sim.volume().box();
  const uint16_t gh = geometryHash(g_sim.geometry());
  FrameHeader h;

  CHECK(!decodeFrame(g_buf, len - 1, box, gh, g_rp, g_hb, h));       // truncated
  CHECK(!decodeFrame(g_buf, kFrameHeaderBytes - 1, box, gh, g_rp, g_hb, h));
  g_buf[0] ^= 0xFF;
  CHECK(!decodeFrame(g_buf, len, box, gh, g_rp, g_hb, h));           // bad magic
  g_buf[0] ^= 0xFF;
  g_buf[2] = kFrameVersion + 1;
  CHECK(!decodeFrame(g_buf, len, box, gh, g_rp, g_hb, h));           // bad version
  g_buf[2] = kFrameVersion;
  CHECK(decodeFrame(g_buf, len, box, gh, g_rp, g_hb, h));            // and still good untouched
}

TEST(simframe_rejects_a_mismatched_build) {
  // The failure this format most needs to make loud: a master built for one panel size talking to
  // a display built for another. Every particle would land in the wrong place, which looks like a
  // physics bug and is not one.
  settle();
  const int len = encodeCurrent(g_buf);
  const uint16_t mine = geometryHash(g_sim.geometry());
  const uint16_t other = geometryHash(Geometry::cube(64, 0.5f));
  std::printf("       geometry hash: 32x32@1.0 = %04x, 64x64@0.5 = %04x\n", mine, other);
  CHECK(mine != other);

  FrameHeader h;
  CHECK(!decodeFrame(g_buf, len, g_sim.volume().box(), other, g_rp, g_hb, h));
  CHECK(decodeFrame(g_buf, len, g_sim.volume().box(), mine, g_rp, g_hb, h));
}

TEST(simframe_heat_rle_handles_the_edge_cases) {
  static uint8_t src[4096], dst[4096], enc[8192];

  // All zero -- the normal case for a water-only scene.
  for (int i = 0; i < 4096; ++i) src[i] = 0;
  int n = rleEncode(src, 4096, enc, sizeof(enc));
  CHECK(n > 0);
  CHECK(n < 100);  // 4096 zeros in ~66 bytes
  CHECK(rleDecode(enc, n, dst, sizeof(dst)) == 4096);
  CHECK(std::memcmp(src, dst, 4096) == 0);
  std::printf("       4096 zeros -> %d B\n", n);

  // Saturated.
  for (int i = 0; i < 4096; ++i) src[i] = 255;
  n = rleEncode(src, 4096, enc, sizeof(enc));
  CHECK(n > 0 && n < 100);
  CHECK(rleDecode(enc, n, dst, sizeof(dst)) == 4096);
  CHECK(std::memcmp(src, dst, 4096) == 0);

  // Incompressible: alternating, the worst case. Must still round-trip and stay inside the bound
  // frameMaxBytes() reserves.
  for (int i = 0; i < 4096; ++i) src[i] = (uint8_t)(i & 1 ? 0x5A : 0xA5);
  n = rleEncode(src, 4096, enc, sizeof(enc));
  CHECK(n > 0);
  CHECK(n <= 4096 + 4096 / 127 + 2);
  CHECK(rleDecode(enc, n, dst, sizeof(dst)) == 4096);
  CHECK(std::memcmp(src, dst, 4096) == 0);
  std::printf("       4096 alternating -> %d B (bound %d)\n", n, 4096 + 4096 / 127 + 2);

  // A realistic mix: mostly cold with a hot blob.
  for (int i = 0; i < 4096; ++i) src[i] = (i > 1800 && i < 2200) ? (uint8_t)(120 + (i % 7)) : 0;
  n = rleEncode(src, 4096, enc, sizeof(enc));
  CHECK(rleDecode(enc, n, dst, sizeof(dst)) == 4096);
  CHECK(std::memcmp(src, dst, 4096) == 0);
  std::printf("       mostly-cold with a blob -> %d B\n", n);

  // Malformed streams must be refused, not trusted.
  const uint8_t zeroRun[2] = {0x00, 0x11};
  CHECK(rleDecode(zeroRun, 2, dst, sizeof(dst)) == -1);
  const uint8_t truncLit[2] = {0x85, 0x11};  // claims 5 literals, supplies 1
  CHECK(rleDecode(truncLit, 2, dst, sizeof(dst)) == -1);
  const uint8_t overflow[2] = {0x7F, 0x11};  // 127 bytes into a 4-byte buffer
  CHECK(rleDecode(overflow, 2, dst, 4) == -1);
}

TEST(simframe_water_only_scene_sends_no_heat) {
  settle(0, 300);  // water tank: field never lights
  const int len = encodeCurrent(g_buf);
  FrameHeader h;
  CHECK(decodeFrame(g_buf, len, g_sim.volume().box(), geometryHash(g_sim.geometry()), g_rp, g_hb, h));
  CHECK(h.heatBytes == 0);
  CHECK(g_hb.peak() == 0);        // so splatField skips the whole pass
  CHECK(g_hb.view().empty);
  std::printf("       water-only frame: %d B, no heat payload\n", len);
}

TEST(simframe_fits_the_spi_budget) {
  // 21 KB/frame at 30 fps was the figure the SPI transport was specified against.
  int worst = 0, worstHeat = 0, worstScene = -1;
  for (int s = 0; s < sceneCount(); ++s) {
    settle(s, 400);
    const int len = encodeCurrent(g_buf);
    CHECK(len > 0);
    // Decode to get this scene's heat payload, rather than inferring it from the total -- the
    // particle count varies per scene, so subtracting one scene's count from another's size is
    // nonsense. (It is, in fact, what a first draft of this test did, and it reported a device
    // projection LARGER than the host figure.)
    FrameHeader h;
    CHECK(decodeFrame(g_buf, len, g_sim.volume().box(), geometryHash(g_sim.geometry()), g_rp, g_hb,
                      h));
    if ((int)h.heatBytes > worstHeat) worstHeat = (int)h.heatBytes;
    if (len > worst) { worst = len; worstScene = s; }
  }
  const float kbps = (float)worst * 30.0f / 1000.0f;
  std::printf("       worst frame at this build's %d-particle capacity: %d B (scene %d), %.0f KB/s\n",
              kMaxParticles, worst, worstScene, kbps);
  std::printf("       worst heat payload across all scenes: %d B\n", worstHeat);
  // The device runs 1280 particles, which is the figure the SPI link was specified against.
  const int dev = kFrameHeaderBytes + 1280 * kFrameBytesPerParticle + worstHeat;
  std::printf("       at the device's 1280 particles: %d B/frame, %.0f KB/s at 30 fps\n", dev,
              (float)dev * 30.0f / 1000.0f);
  // 2.5 MB/s is available at 20 MHz; the 10 MHz fallback gives 1.25 MB/s.
  CHECK((float)dev * 30.0f / 1000.0f < 1250.0f);
  CHECK(worst <= frameMaxBytes());
  // Against 2.5 MB/s at 20 MHz SPI.
  CHECK(kbps < 1200.0f);
}
