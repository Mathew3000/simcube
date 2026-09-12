// Renders the beaker overlay to PPM so it can be LOOKED AT.
//
// This item is almost entirely visual and the golden hashes cannot tell you whether a glyph is
// legible at 3x5 on a 32x32 face, so the deliverable is an image, not a number. Separate from
// ppm_dump.cpp rather than a flag on it: the overlay needs the accumulate / draw / resolve
// sequence spelled out, which is exactly the sequence beaker mode's render loop will run.
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "partsim/BeakerOverlay.h"
#include "partsim/Simulation.h"

using namespace partsim;

namespace {

Simulation g_sim;  // ~1.2MB, static storage only
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

void writeNet(const char* path) {
  const int cols = 4, rows = 3;
  const int W = cols * g_res, H = rows * g_res;
  static uint8_t net[4 * 3 * kMaxPanelTexels * 4];
  std::memset(net, 0, (size_t)W * H * 4);
  for (int s = 0; s < 6; ++s) {
    const uint8_t* src = g_face[kNet[s].panel];
    for (int j = 0; j < g_res; ++j)
      for (int i = 0; i < g_res; ++i) {
        const int nx = kNet[s].col * g_res + i;
        const int ny = (rows - 1 - kNet[s].row) * g_res + j;
        std::memcpy(net + ((size_t)ny * W + nx) * 4, src + ((size_t)j * g_res + i) * 4, 4);
      }
  }
  writePpm(path, net, W, H, g_scale);
}

// The sequence beaker mode's render loop runs: splat the fluid, composite the overlay into the
// SAME accumulation buffers, then resolve. One colour path, one palette.
void dump(const char* tag, bool gateArmed) {
  g_sim.accumulate();
  g_overlay.draw(g_sim.renderer(), gateArmed);
  for (int k = 0; k < 6; ++k) g_sim.renderer().resolve(k, g_face[k], 4);

  char path[256];
  std::snprintf(path, sizeof(path), "out/beaker_%s_net.ppm", tag);
  writeNet(path);

  int lit = 0;
  double sum = 0.0;
  for (int k = 0; k < 6; ++k)
    for (int i = 0; i < g_res * g_res; ++i) {
      const uint8_t* px = g_face[k] + i * 4;
      const double lum = 0.2126 * px[0] + 0.7152 * px[1] + 0.0722 * px[2];
      sum += lum;
      if (lum > 4.0) ++lit;
    }
  const int total = 6 * g_res * g_res;
  std::printf("%-22s gate %-8s particles %4d  lit %4.1f%%  mean lum %5.1f  -> %s\n", tag,
              gateArmed ? "ARMED" : "released", g_sim.particleCount(),
              100.0 * lit / total, sum / total, path);
}

}  // namespace

int main(int argc, char** argv) {
  const int steps = (argc > 1) ? atoi(argv[1]) : 400;
  if (argc > 2) {
    g_res = atoi(argv[2]);
    if (g_res * g_res > kMaxPanelTexels) {
      std::printf("resolution %d needs %d texels; this build caps at %d\n", g_res,
                  g_res * g_res, kMaxPanelTexels);
      return 1;
    }
  }
  g_scale = imax(1, 192 / g_res);

  std::printf("panels %dx%d, glyph scale %d, overlay intensity %d (exposure %.0f)\n", g_res,
              g_res, BeakerOverlay::scaleFor(g_res), (int)kSplatExposure, kSplatExposure);
#if !PARTSIM_ENABLE_CHROMA
  std::printf("NOTE: built without PARTSIM_ENABLE_CHROMA -- one accumulation channel, so the\n"
              "      gate arrows resolve WHITE, not red. See docs/M4-C-FINDINGS.md.\n");
#endif

  // Scene 0 is the water tank; the overlay is judged against real fluid, not an empty cube,
  // because the question an image answers here is whether the lines stay readable over liquid.
  g_sim.initScene(Simulation::kCube, 0, 0xC0FFEEu, g_res);
  g_overlay.init(g_sim.geometry());

  // Armed: the cube has just been picked up on its side, so nothing has stepped yet.
  dump("armed_at_rest", true);

  for (int s = 0; s < steps; ++s) g_sim.stepFixed();
  dump("released_settled", false);

  const float a = 35.0f * kPi / 180.0f;
  g_sim.setOrientation(Quat{0.0f, 0.0f, fsin(a * 0.5f), fcos(a * 0.5f)});
  for (int s = 0; s < steps; ++s) g_sim.stepFixed();
  dump("released_tilt35", false);
  return 0;
}
