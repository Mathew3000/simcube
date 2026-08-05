// Renders each scene's six cube faces to PPM images with no browser, no WASM and no GL in the
// loop. Judging the visuals here keeps that risk off the critical path -- if "glowing liquid
// inside frosted glass" does not read on a 32x32 face, it is far cheaper to learn it now.
//
// Drives the real Simulation and the real scene table, so it doubles as an end-to-end check of
// the scene presets.
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "partsim/Simulation.h"

using namespace partsim;

namespace {

Simulation g_sim;  // ~1.2MB, static storage only

constexpr int kRes = 32;
constexpr int kScale = 6;  // upscale so the pixels are visible in an image viewer

// Cube panel order from Geometry::cube: 0 = -Z, 1 = +Z, 2 = -X, 3 = +X, 4 = -Y, 5 = +Y.
const char* kFaceNames[6] = {"negZ", "posZ", "negX", "posX", "negY_bottom", "posY_top"};

// Unfolded cross, the standard cube net:
//
//        [+Y]
//   [-X] [-Z] [+X] [+Z]
//        [-Y]
//
// The horizontal strip reads as a continuous walk around the cube's sides, so a waterline that
// jumps at a seam is immediately visible.
struct NetSlot { int panel, col, row; };
const NetSlot kNet[6] = {
    {5, 1, 0},
    {2, 0, 1}, {0, 1, 1}, {3, 2, 1}, {1, 3, 1},
    {4, 1, 2},
};

// PPM rows run top-down but panel row 0 is the BOTTOM row, so rows are emitted flipped.
void writePpm(const char* path, const uint8_t* rgba, int w, int h, int scale) {
  FILE* f = fopen(path, "wb");
  if (!f) { std::printf("  ! cannot write %s\n", path); return; }
  fprintf(f, "P6\n%d %d\n255\n", w * scale, h * scale);
  for (int j = h - 1; j >= 0; --j)
    for (int sy = 0; sy < scale; ++sy)
      for (int i = 0; i < w; ++i)
        for (int sx = 0; sx < scale; ++sx) fwrite(rgba + ((size_t)j * w + i) * 4, 1, 3, f);
  fclose(f);
}

void writeNet(const char* path, const Renderer& r, int scale) {
  const int cols = 4, rows = 3;
  const int W = cols * kRes, H = rows * kRes;
  static uint8_t net[4 * 32 * 3 * 32 * 4];
  std::memset(net, 0, (size_t)W * H * 4);

  for (int s = 0; s < 6; ++s) {
    const uint8_t* src = r.panelPixels(kNet[s].panel);
    for (int j = 0; j < kRes; ++j)
      for (int i = 0; i < kRes; ++i) {
        // Slots are placed in PANEL space (row 0 at the bottom); writePpm flips the whole image
        // once at the end, which lands the net the right way up.
        const int nx = kNet[s].col * kRes + i;
        const int ny = (rows - 1 - kNet[s].row) * kRes + j;
        std::memcpy(net + ((size_t)ny * W + nx) * 4, src + ((size_t)j * kRes + i) * 4, 4);
      }
  }
  writePpm(path, net, W, H, scale);
}

void report(const char* tag) {
  const Renderer& r = g_sim.renderer();
  int lit = 0, total = 0;
  double sum = 0.0;
  for (int k = 0; k < 6; ++k) {
    const uint8_t* px = r.panelPixels(k);
    for (int i = 0; i < kRes * kRes; ++i) {
      const double lum = 0.2126 * px[i * 4] + 0.7152 * px[i * 4 + 1] + 0.0722 * px[i * 4 + 2];
      sum += lum;
      if (lum > 4.0) ++lit;
      ++total;
    }
  }
  std::printf("%-28s particles %4d  lit %4.1f%%  mean lum %5.1f\n", tag,
              g_sim.particleCount(), 100.0 * lit / total, sum / total);
}

// Runs a scene and dumps it. tiltDeg rotates the object about Z; gravity stays world-down.
void dumpScene(int sceneId, int steps, float tiltDeg, const char* suffix) {
  g_sim.initScene(Simulation::kCube, sceneId, 0xC0FFEEu);
  if (tiltDeg != 0.0f) {
    const float a = tiltDeg * kPi / 180.0f;
    g_sim.setOrientation(Quat{0.0f, 0.0f, fsin(a * 0.5f), fcos(a * 0.5f)});
  }
  for (int s = 0; s < steps; ++s) g_sim.stepFixed();
  g_sim.render();

  char tag[64];
  std::snprintf(tag, sizeof(tag), "%d_%s%s", sceneId, sceneAt(sceneId).name, suffix);
  for (char* c = tag; *c; ++c)
    if (*c == ' ') *c = '_';

  char path[256];
  std::snprintf(path, sizeof(path), "out/net_%s.ppm", tag);
  writeNet(path, g_sim.renderer(), kScale);
  for (int k = 0; k < 6; ++k) {
    std::snprintf(path, sizeof(path), "out/%s_%d_%s.ppm", tag, k, kFaceNames[k]);
    writePpm(path, g_sim.renderer().panelPixels(k), kRes, kRes, kScale);
  }
  report(tag);
}

}  // namespace

int main(int argc, char** argv) {
  const int steps = (argc > 1) ? atoi(argv[1]) : 400;
  std::printf("splat influence %.1f, footprint %d, exposure %.0f, heat gain %.0f\n",
              kSplatInfluence, kSplatFootprint, kSplatExposure, kHeatGain);

  for (int s = 0; s < sceneCount(); ++s) dumpScene(s, steps, 0.0f, "");

  // Tilted variants: a slanted waterline continuous across the seams checks that the panel
  // mapping is right, and a leaning flame checks that heat uses the same object-space gravity
  // the solver does.
  dumpScene(0, steps, 35.0f, "_tilt35");
  dumpScene(1, steps, 35.0f, "_tilt35");
  return 0;
}
