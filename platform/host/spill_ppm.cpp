// M4-B as an image: liquid leaving an open face, and arriving in the next cube.
//
// The counts in tests/test_beaker_spill.cpp can tell you a ring conserves particles. They cannot
// tell you whether the liquid leaves as a STREAM or as a burst, whether it arrives as a pour or as
// a teleport, or whether a beaker that is draining looks like it is draining. Those are the
// questions this item is actually judged on, so the deliverable is a picture.
//
// Separate from beaker_ppm.cpp (M4-C, the overlay) rather than a flag on it: this drives two
// simulations wired into a chain, which is a different harness, and M4-C's tool is the reference
// image for the overlay on its own.
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "partsim/BeakerOverlay.h"
#include "partsim/Simulation.h"

using namespace partsim;

namespace {

// ~1.2MB each; static storage only.
Simulation g_a, g_b;
BeakerOverlay g_overlay;

int g_res = 32;
int g_scale = 6;

// Cube panel order from Geometry::cube: 0 = -Z, 1 = +Z, 2 = -X, 3 = +X, 4 = -Y, 5 = +Y.
//
//        [+Y]
//   [-X] [-Z] [+X] [+Z]
//        [-Y]
struct NetSlot { int panel, col, row; };
const NetSlot kNet[6] = {
    {5, 1, 0},
    {2, 0, 1}, {0, 1, 1}, {3, 2, 1}, {1, 3, 1},
    {4, 1, 2},
};

uint8_t g_face[6][kMaxPanelTexels * 4];
// Two nets side by side: the pouring cube on the left, the receiving one on the right.
uint8_t g_sheet[2 * 4 * 3 * kMaxPanelTexels * 4];

// PPM rows run top-down but panel row 0 is the BOTTOM row, so rows are emitted flipped.
void writePpm(const char* path, const uint8_t* rgba, int w, int h, int scale) {
  FILE* f = fopen(path, "wb");
  if (!f) { std::printf("  ! cannot write %s (mkdir -p out)\n", path); return; }
  fprintf(f, "P6\n%d %d\n255\n", w * scale, h * scale);
  for (int j = h - 1; j >= 0; --j)
    for (int sy = 0; sy < scale; ++sy)
      for (int i = 0; i < w; ++i)
        for (int sx = 0; sx < scale; ++sx) fwrite(rgba + ((size_t)j * w + i) * 4, 1, 3, f);
  fclose(f);
}

// Splat, composite the overlay, resolve -- the sequence beaker mode's render loop runs -- then
// blit this cube's net into one half of the sheet.
void blitNet(Simulation& sim, int half, int sheetW) {
  sim.accumulate();
  g_overlay.draw(sim.renderer(), false);
  for (int k = 0; k < 6; ++k) sim.renderer().resolve(k, g_face[k], 4);

  const int rows = 3;
  for (int s = 0; s < 6; ++s) {
    const uint8_t* src = g_face[kNet[s].panel];
    for (int j = 0; j < g_res; ++j)
      for (int i = 0; i < g_res; ++i) {
        const int nx = half * (4 * g_res + 1) + kNet[s].col * g_res + i;
        const int ny = (rows - 1 - kNet[s].row) * g_res + j;
        std::memcpy(g_sheet + ((size_t)ny * sheetW + nx) * 4, src + ((size_t)j * g_res + i) * 4, 4);
      }
  }
}

void dump(const char* tag, int step) {
  const int sheetW = 2 * 4 * g_res + 1, sheetH = 3 * g_res;
  std::memset(g_sheet, 0, (size_t)sheetW * sheetH * 4);
  // A one-texel divider so the two cubes do not read as one wide net.
  for (int j = 0; j < sheetH; ++j) {
    uint8_t* px = g_sheet + ((size_t)j * sheetW + 4 * g_res) * 4;
    px[0] = px[1] = px[2] = 40;
  }
  blitNet(g_a, 0, sheetW);
  blitNet(g_b, 1, sheetW);

  char path[256];
  std::snprintf(path, sizeof(path), "out/spill_%s.ppm", tag);
  writePpm(path, g_sheet, sheetW, sheetH, g_scale);
  std::printf("%-18s step %4d   A %4d  B %4d  total %4d   A spilled %5u (%u dropped)  -> %s\n",
              tag, step, g_a.particleCount(), g_b.particleCount(),
              g_a.particleCount() + g_b.particleCount(), g_a.spill().totalOut,
              g_a.spill().dropped, path);
}

int drain(Simulation& from, Simulation& to) {
  int rejected = 0;
  for (int i = 0; i < from.spill().count; ++i)
    if (!to.injectSpill(from.spill().items[i])) ++rejected;
  from.spill().clear();
  return rejected;
}

}  // namespace

int main(int argc, char** argv) {
  const int steps = (argc > 1) ? atoi(argv[1]) : 600;
  if (argc > 2) {
    g_res = atoi(argv[2]);
    if (g_res * g_res > kMaxPanelTexels) {
      std::printf("resolution %d needs %d texels; this build caps at %d\n", g_res, g_res * g_res,
                  kMaxPanelTexels);
      return 1;
    }
  }
  g_scale = imax(1, 192 / g_res);

  // A full, B a third full. B has to have room, or it overflows out of its own open top from the
  // first arrival and the picture is two cubes spilling rather than one filling.
  g_a.init(Simulation::kCube, (kMaxParticles * 4) / 5, 0xC0FFEEu, g_res);
  g_b.init(Simulation::kCube, particlesForFill(1000), 0xBEEFu, g_res);
  g_a.setOpenFace(kOpenPosY);
  g_b.setOpenFace(kOpenPosY);
  g_overlay.init(g_a.geometry());

#if PARTSIM_ENABLE_CHROMA
  // A red beaker pouring into a blue one, which is the whole picture M4 is after.
  Particles& pa = const_cast<Particles&>(g_a.particles());
  for (int i = 0; i < pa.n; ++i) { pa.cr[i] = (uint16_t)kChromaOne; pa.cg[i] = 0; }
  Particles& pb = const_cast<Particles&>(g_b.particles());
  for (int i = 0; i < pb.n; ++i) { pb.cr[i] = 0; pb.cg[i] = 0; }
  std::printf("A is red, B is blue; the pour should arrive red and mix toward magenta.\n");
#else
  std::printf("NOTE: built without PARTSIM_ENABLE_CHROMA -- one channel, so both cubes are the\n"
              "      same colour and only the LEVELS are readable here. Build the beaker tier\n"
              "      (-DPARTSIM_TIER_BEAKER=1) for the colour question.\n");
#endif
  std::printf("panels %dx%d, left = pouring cube A, right = receiving cube B\n", g_res, g_res);

  // Settle upright first, so the "before" frame is a full beaker rather than a jittering fill.
  for (int s = 0; s < settleSteps(200); ++s) {
    g_a.stepFixed();
    g_b.stepFixed();
    g_a.spill().clear();
    g_b.spill().clear();
  }
  dump("0_upright", 0);

  // Then tip A past horizontal and hold it there. A steady pour, not a shake: the question is
  // what a STREAM looks like, and a shake would answer a different one.
  const float a = 115.0f * kPi / 180.0f;
  g_a.setOrientation(Quat{0.0f, 0.0f, fsin(a * 0.5f), fcos(a * 0.5f)});
  int rejected = 0;
  for (int s = 1; s <= steps; ++s) {
    g_a.stepFixed();
    g_b.stepFixed();
    rejected += drain(g_a, g_b);
    g_b.spill().clear();  // B is the end of this chain, so what leaves B is lost
    if (s == 15) dump("1_first_frames", s);
    if (s == 40) dump("2_pouring", s);
    if (s == 120) dump("3_stream", s);
    if (s == steps / 2) dump("4_half", s);
  }
  dump("5_end", steps);
  std::printf("rejected on arrival: %d (pool full)   B spilled on: %u\n", rejected,
              g_b.spill().totalOut);
  return 0;
}
