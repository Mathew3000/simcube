// Renders the six cube faces to PPM images with no browser, no WASM and no GL in the loop.
//
// This exists to de-risk the whole visual concept off the critical path: if "glowing liquid
// inside frosted glass" does not read on a 32x32 face, it is far cheaper to find out here
// than after building a three.js frontend.
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "partsim/Renderer.h"
#include "partsim/Rng.h"
#include "partsim/Solver.h"

using namespace partsim;

namespace {

// Far too big for the stack.
Particles g_p;
SpatialHash g_h;
Solver g_solver;
Renderer g_renderer;
float g_scratch[kMaxParticles];

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
// Laid out this way the horizontal strip reads as a continuous walk around the cube's sides,
// so a discontinuity at a seam is immediately visible.
struct NetSlot { int panel, col, row; };
const NetSlot kNet[6] = {
    {5, 1, 0},  // +Y top
    {2, 0, 1}, {0, 1, 1}, {3, 2, 1}, {1, 3, 1},
    {4, 1, 2},  // -Y bottom
};

int fillBottom(Particles& p, const Aabb& box, int want, uint8_t mat, uint32_t seed) {
  Rng r(seed);
  const float d = kRestSpacing;
  const int nx = (int)(box.size().x / d);
  const int nz = (int)(box.size().z / d);
  for (int ly = 0; p.n < want && ly < 64; ++ly)
    for (int iz = 0; iz < nz && p.n < want; ++iz)
      for (int ix = 0; ix < nx && p.n < want; ++ix)
        p.add(Vec3{box.lo.x + (0.5f + (float)ix) * d + r.nextSigned() * 0.1f * d,
                   box.lo.y + (0.5f + (float)ly) * d + r.nextSigned() * 0.1f * d,
                   box.lo.z + (0.5f + (float)iz) * d + r.nextSigned() * 0.1f * d},
              Vec3{0, 0, 0}, mat);
  return p.n;
}

// PPM rows run top-down but panel +y is up, so rows are emitted flipped.
void writePpm(const char* path, const uint8_t* rgba, int w, int h, int scale) {
  FILE* f = fopen(path, "wb");
  if (!f) {
    std::printf("  ! cannot write %s\n", path);
    return;
  }
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
        const int nx = kNet[s].col * kRes + i;
        const int ny = kNet[s].row * kRes + j;
              // Slots are placed in PANEL space (row 0 at the bottom); writePpm flips the whole
        // image once at the end, which lands the net the right way up.
        const int flipped = (rows - 1 - kNet[s].row) * kRes + j;
        std::memcpy(net + ((size_t)flipped * W + nx) * 4, src + ((size_t)j * kRes + i) * 4, 4);
        (void)ny;
      }
  }
  writePpm(path, net, W, H, scale);
}

struct Stats {
  int lit, total, clipped;
  double meanLum, maxLum;
};

Stats measure(const Renderer& r, int panels) {
  Stats s{0, 0, 0, 0.0, 0.0};
  double sum = 0.0;
  for (int k = 0; k < panels; ++k) {
    const uint8_t* px = r.panelPixels(k);
    for (int i = 0; i < kRes * kRes; ++i) {
      const double lum = 0.2126 * px[i * 4] + 0.7152 * px[i * 4 + 1] + 0.0722 * px[i * 4 + 2];
      sum += lum;
      if (lum > s.maxLum) s.maxLum = lum;
      if (lum > 4.0) ++s.lit;
      if (px[i * 4] > 250 && px[i * 4 + 1] > 250 && px[i * 4 + 2] > 250) ++s.clipped;
      ++s.total;
    }
  }
  s.meanLum = sum / s.total;
  return s;
}

void scene(const char* tag, int count, int steps, Vec3 gravity, const Palette& pal,
           float exposure) {
  const Geometry g = Geometry::cube(kRes, 1.0f);
  SimVolume v;
  v.build(g, kSlabDepth, kCellSize);
  g_solver.init();
  g_p.clear();
  fillBottom(g_p, v.box(), count, kWater, 0xA11CE);
  for (int s = 0; s < steps; ++s)
    g_solver.step(g_p, v, g_h, g_scratch, defaultMaterials(), gravity, kFixedDt);

  g_renderer.setPalette(&pal);
  g_renderer.setExposure(exposure);
  g_renderer.init(g);
  g_renderer.render(g_p, g);

  char path[256];
  std::snprintf(path, sizeof(path), "out/net_%s.ppm", tag);
  writeNet(path, g_renderer, kScale);
  for (int k = 0; k < 6; ++k) {
    std::snprintf(path, sizeof(path), "out/%s_%d_%s.ppm", tag, k, kFaceNames[k]);
    writePpm(path, g_renderer.panelPixels(k), kRes, kRes, kScale);
  }

  const Stats st = measure(g_renderer, 6);
  // Peak raw accumulation tells us what exposure *should* be.
  int peakAccum = 0;
  for (int k = 0; k < 6; ++k)
    for (int j = 0; j < kRes; ++j)
      for (int i = 0; i < kRes; ++i)
        for (int c = 0; c < kChannelCount; ++c)
          peakAccum = imax(peakAccum, (int)g_renderer.accumAt(k, i, j, kRes, c));

  std::printf("%-14s exp %6.0f  lit %4.1f%%  clipped %4.1f%%  mean lum %5.1f  "
              "peak accum %5d\n",
              tag, (double)exposure, 100.0 * st.lit / st.total,
              100.0 * st.clipped / st.total, st.meanLum, peakAccum);
}

}  // namespace

int main(int argc, char** argv) {
  const int count = (argc > 1) ? atoi(argv[1]) : 3000;
  std::printf("splat influence %.1f units, footprint %d texels, exposure default\n",
              kSplatInfluence, kSplatFootprint);

  if (argc > 2 && argv[2][0] == 'e') {
    // Exposure sweep: the accumulated intensity that maps to the top of the ramp.
    const float exposures[6] = {900.0f, 2000.0f, 3500.0f, 5000.0f, 8000.0f, 14000.0f};
    for (int i = 0; i < 6; ++i) {
      char tag[32];
      std::snprintf(tag, sizeof(tag), "exp%05d", (int)exposures[i]);
      scene(tag, count, 400, Vec3{0.0f, -kGravityMag, 0.0f}, paletteNaturalistic(),
            exposures[i]);
    }
    return 0;
  }

  scene("settled", count, 400, Vec3{0.0f, -kGravityMag, 0.0f}, paletteNaturalistic(),
        kSplatExposure);
  scene("tilted", count, 400, normalize(Vec3{1.0f, -1.0f, 0.35f}) * kGravityMag,
        paletteNaturalistic(), kSplatExposure);
  scene("falling", count, 12, Vec3{0.0f, -kGravityMag, 0.0f}, paletteNaturalistic(),
        kSplatExposure);
  scene("half", count / 2, 400, Vec3{0.0f, -kGravityMag, 0.0f}, paletteNaturalistic(),
        kSplatExposure);
  scene("neon", count, 400, Vec3{0.0f, -kGravityMag, 0.0f}, paletteNeon(), kSplatExposure);
  return 0;
}
