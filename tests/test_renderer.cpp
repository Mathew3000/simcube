#include "check.h"
#include "partsim/Renderer.h"

using namespace partsim;

namespace {

// ~330KB of buffers; far too big for the stack.
Renderer g_r;
Particles g_p;

// Peak-intensity texel on a panel, for the given channel.
struct Peak { int i, j, value; };

Peak findPeak(const Renderer& r, int panel, int w, int h, int channel) {
  Peak best{-1, -1, 0};
  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; ++i) {
      const int v = (int)r.accumAt(panel, i, j, w, channel);
      if (v > best.value) best = Peak{i, j, v};
    }
  return best;
}

int totalIntensity(const Renderer& r, int panel, int w, int h, int channel) {
  int sum = 0;
  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; ++i) sum += (int)r.accumAt(panel, i, j, w, channel);
  return sum;
}

}  // namespace

TEST(renderer_single_particle_lands_on_the_predicted_texel) {
  const Geometry g = Geometry::cube(32, 1.0f);
  g_r.init(g);

  // 2 units inward from the centre of texel (10, 20) on panel 0 (the -Z face).
  const Panel& pan = g.at(0);
  const Vec3 where = texelCenter(pan, 10, 20) + pan.n * 2.0f;
  g_p.clear();
  g_p.add(where, Vec3{0, 0, 0}, kWater);

  g_r.clear();
  g_r.splat(g_p, g);

  const Peak pk = findPeak(g_r, 0, 32, 32, kChWater);
  CHECK(pk.i == 10);
  CHECK(pk.j == 20);
  CHECK(pk.value > 0);
  // It writes the water channel and nothing else.
  CHECK(totalIntensity(g_r, 0, 32, 32, kChSand) == 0);
  CHECK(totalIntensity(g_r, 0, 32, 32, kChHeat) == 0);
}

TEST(renderer_sand_uses_its_own_channel) {
  const Geometry g = Geometry::cube(32, 1.0f);
  g_r.init(g);
  const Panel& pan = g.at(0);
  g_p.clear();
  g_p.add(texelCenter(pan, 5, 5) + pan.n * 1.0f, Vec3{0, 0, 0}, kSand);
  g_r.clear();
  g_r.splat(g_p, g);
  CHECK(totalIntensity(g_r, 0, 32, 32, kChSand) > 0);
  CHECK(totalIntensity(g_r, 0, 32, 32, kChWater) == 0);
}

TEST(renderer_falloff_has_compact_support) {
  // Load-bearing: a falloff that never reaches zero would make every particle touch every
  // panel, turning the cost from O(N * footprint) into O(N * panels * texels).
  const Geometry g = Geometry::cube(32, 1.0f);
  g_r.init(g);
  const Panel& pan = g.at(0);

  // Well inside the influence radius: something. Just outside: exactly nothing. (Right at
  // the rim the contribution legitimately quantises to zero, so 0.95 would prove nothing.)
  g_p.clear();
  g_p.add(texelCenter(pan, 16, 16) + pan.n * (kSplatInfluence * 0.6f), Vec3{0, 0, 0}, kWater);
  g_r.clear();
  g_r.splat(g_p, g);
  CHECK(totalIntensity(g_r, 0, 32, 32, kChWater) > 0);

  g_p.clear();
  g_p.add(texelCenter(pan, 16, 16) + pan.n * (kSplatInfluence + 0.01f), Vec3{0, 0, 0}, kWater);
  g_r.clear();
  g_r.splat(g_p, g);
  CHECK(totalIntensity(g_r, 0, 32, 32, kChWater) == 0);

  // And exactly at the rim, also nothing -- the support is closed at the top.
  g_p.clear();
  g_p.add(texelCenter(pan, 16, 16) + pan.n * kSplatInfluence, Vec3{0, 0, 0}, kWater);
  g_r.clear();
  g_r.splat(g_p, g);
  CHECK(totalIntensity(g_r, 0, 32, 32, kChWater) == 0);
}

TEST(renderer_is_brighter_nearer_the_glass) {
  const Geometry g = Geometry::cube(32, 1.0f);
  g_r.init(g);
  const Panel& pan = g.at(0);

  int prev = 1 << 30;
  for (int d = 0; d < 8; ++d) {
    g_p.clear();
    g_p.add(texelCenter(pan, 16, 16) + pan.n * ((float)d + 0.01f), Vec3{0, 0, 0}, kWater);
    g_r.clear();
    g_r.splat(g_p, g);
    const int t = totalIntensity(g_r, 0, 32, 32, kChWater);
    CHECK(t <= prev);  // monotonically dimmer with depth
    prev = t;
  }
  CHECK(prev >= 0);
}

TEST(renderer_particle_behind_a_panel_contributes_nothing) {
  // dist < 0 means outside the volume; such a particle must not light the panel from behind.
  const Geometry g = Geometry::cube(32, 1.0f);
  g_r.init(g);
  const Panel& pan = g.at(0);
  g_p.clear();
  g_p.add(texelCenter(pan, 16, 16) - pan.n * 1.0f, Vec3{0, 0, 0}, kWater);
  g_r.clear();
  g_r.splat(g_p, g);
  CHECK(totalIntensity(g_r, 0, 32, 32, kChWater) == 0);
}

TEST(renderer_corner_particle_lights_three_panels) {
  // The reason panel selection is brute force rather than an acceleration structure.
  const Geometry g = Geometry::cube(32, 1.0f);
  g_r.init(g);
  g_p.clear();
  // 1 unit in from the (-x, -y, -z) corner.
  g_p.add(Vec3{-15.0f, -15.0f, -15.0f}, Vec3{0, 0, 0}, kWater);
  g_r.clear();
  g_r.splat(g_p, g);

  int litPanels = 0;
  for (int k = 0; k < g.count(); ++k)
    if (totalIntensity(g_r, k, 32, 32, kChWater) > 0) ++litPanels;
  CHECK(litPanels == 3);  // -Z, -X and -Y bottom

  // The three far faces see nothing at all.
  CHECK(totalIntensity(g_r, 1, 32, 32, kChWater) == 0);  // +Z
  CHECK(totalIntensity(g_r, 3, 32, 32, kChWater) == 0);  // +X
  CHECK(totalIntensity(g_r, 5, 32, 32, kChWater) == 0);  // +Y
}

TEST(renderer_centre_of_a_large_cube_lights_nothing) {
  // With influence 8 in a 32-unit cube, the middle is out of reach of every face. This is
  // expected, and worth pinning down so it is not mistaken for a bug later.
  const Geometry g = Geometry::cube(32, 1.0f);
  g_r.init(g);
  g_p.clear();
  g_p.add(Vec3{0, 0, 0}, Vec3{0, 0, 0}, kWater);
  g_r.clear();
  g_r.splat(g_p, g);
  for (int k = 0; k < g.count(); ++k) CHECK(totalIntensity(g_r, k, 32, 32, kChWater) == 0);
}

TEST(renderer_clear_and_empty_scene_are_black) {
  const Geometry g = Geometry::cube(32, 1.0f);
  g_r.init(g);
  g_p.clear();
  g_r.render(g_p, g);
  for (int k = 0; k < g.count(); ++k) {
    const uint8_t* px = g_r.panelPixels(k);
    for (int i = 0; i < 32 * 32; ++i) {
      CHECK(px[i * 4 + 0] == 0);
      CHECK(px[i * 4 + 1] == 0);
      CHECK(px[i * 4 + 2] == 0);
      CHECK(px[i * 4 + 3] == 255);  // opaque even where black
    }
  }
}

TEST(renderer_resolve_rgb_and_rgba_agree) {
  // The ESP32 wants tight RGB for its LED driver; WebGL wants RGBA for unpack alignment.
  // Both must come out of the same resolve path with identical colour.
  const Geometry g = Geometry::cube(32, 1.0f);
  g_r.init(g);
  g_p.clear();
  for (int i = 0; i < 200; ++i)
    g_p.add(Vec3{-14.0f + (float)(i % 20) * 1.2f, -14.0f, -14.0f + (float)(i / 20) * 1.2f},
            Vec3{0, 0, 0}, kWater);
  g_r.clear();
  g_r.splat(g_p, g);

  static uint8_t rgb[32 * 32 * 3];
  static uint8_t rgba[32 * 32 * 4];
  g_r.resolve(4, rgb, 3);   // the -Y bottom face
  g_r.resolve(4, rgba, 4);

  int nonBlack = 0;
  for (int i = 0; i < 32 * 32; ++i) {
    CHECK(rgb[i * 3 + 0] == rgba[i * 4 + 0]);
    CHECK(rgb[i * 3 + 1] == rgba[i * 4 + 1]);
    CHECK(rgb[i * 3 + 2] == rgba[i * 4 + 2]);
    if (rgba[i * 4] || rgba[i * 4 + 1] || rgba[i * 4 + 2]) ++nonBlack;
  }
  CHECK(nonBlack > 50);  // the scene really did light that face
}

TEST(renderer_exposure_controls_brightness_without_clipping_to_white) {
  const Geometry g = Geometry::cube(32, 1.0f);
  g_p.clear();
  for (int i = 0; i < 400; ++i)
    g_p.add(Vec3{-14.0f + (float)(i % 20) * 1.3f, -15.0f, -14.0f + (float)(i / 20) * 1.3f},
            Vec3{0, 0, 0}, kWater);

  // Lower exposure == brighter. A too-low value clips the whole ramp to near-white, which is
  // exactly the failure the first render of this project hit.
  double lumBright = 0.0, lumDim = 0.0;
  for (int pass = 0; pass < 2; ++pass) {
    g_r.setExposure(pass == 0 ? 600.0f : 6000.0f);
    g_r.init(g);
    g_r.render(g_p, g);
    const uint8_t* px = g_r.panelPixels(4);
    double sum = 0.0;
    for (int i = 0; i < 32 * 32; ++i)
      sum += 0.2126 * px[i * 4] + 0.7152 * px[i * 4 + 1] + 0.0722 * px[i * 4 + 2];
    if (pass == 0) lumBright = sum / (32 * 32); else lumDim = sum / (32 * 32);
  }
  CHECK(lumBright > lumDim * 1.5);
  g_r.setExposure(kSplatExposure);
}

// --- palettes --------------------------------------------------------------

TEST(palette_ramps_run_dark_to_bright) {
  for (int p = 0; p < paletteCount(); ++p) {
    const Palette& pal = paletteAt(p);
    const Ramp* ramps[3] = {&pal.water, &pal.sand, &pal.heat};
    for (int r = 0; r < 3; ++r) {
      // Stop 0 must be black, or an almost-empty texel glows and the fluid loses its shape.
      CHECK(ramps[r]->rgb[0][0] == 0);
      CHECK(ramps[r]->rgb[0][1] == 0);
      CHECK(ramps[r]->rgb[0][2] == 0);

      int prev = -1;
      for (int s = 0; s < kRampStops; ++s) {
        const int lum = 2 * ramps[r]->rgb[s][0] + 7 * ramps[r]->rgb[s][1] + ramps[r]->rgb[s][2];
        CHECK(lum > prev);  // strictly brightening
        prev = lum;
      }
    }
  }
}

TEST(palette_lookup_endpoints_and_interpolation) {
  const Ramp& r = paletteNaturalistic().water;
  uint8_t c[3];

  rampLookup(r, 0, c);
  CHECK(c[0] == r.rgb[0][0] && c[1] == r.rgb[0][1] && c[2] == r.rgb[0][2]);
  rampLookup(r, 255, c);
  CHECK(c[0] == r.rgb[kRampStops - 1][0] && c[2] == r.rgb[kRampStops - 1][2]);
  // Out-of-range levels clamp rather than reading past the ramp.
  rampLookup(r, -50, c);
  CHECK(c[0] == r.rgb[0][0]);
  rampLookup(r, 9999, c);
  CHECK(c[2] == r.rgb[kRampStops - 1][2]);

  // Luminance rises monotonically across the whole 0..255 sweep.
  int prev = -1;
  for (int level = 0; level <= 255; ++level) {
    rampLookup(r, level, c);
    const int lum = 2 * c[0] + 7 * c[1] + c[2];
    CHECK(lum >= prev);
    prev = lum;
  }
}

TEST(palette_naturalistic_water_is_blue_and_fire_is_warm) {
  // Cheap sanity check that the ramps were not transposed: water must be blue-dominant in
  // its mid range, fire red-dominant.
  uint8_t c[3];
  rampLookup(paletteNaturalistic().water, 140, c);
  CHECK(c[2] > c[0]);
  rampLookup(paletteNaturalistic().heat, 140, c);
  CHECK(c[0] > c[2]);
}
