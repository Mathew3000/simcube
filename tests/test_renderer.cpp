#include "check.h"
#include "partsim/Renderer.h"

using namespace partsim;

namespace {

// ~330KB of buffers; far too big for the stack.
Renderer g_r;
FieldGrid g_noFire;  // no fire in these scenes; splatField exits immediately
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
  g_r.render(g_p, g_noFire, g);
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
    g_r.render(g_p, g_noFire, g);
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

// --- pitch independence -------------------------------------------------------------------------
// A particle is a physical object: its blob must be the same size in WORLD units whatever the
// panel resolution. Before kSplatRadiusWorld existed, the footprint was a fixed 2 texels, so
// doubling the resolution silently halved the blob and the fluid read as a spray of hard dots.
// Nothing caught that, because every test hardcoded a 32-texel panel at pitch 1.0.

namespace {

// Lit texels and total intensity for one particle sitting `depth` inward from a face centre.
struct Blob { int litTexels, total, footprint; };

Blob blobFor(int res, float pitch, float depth) {
  const Geometry g = Geometry::cube(res, pitch);
  g_r.init(g);
  const Panel& pan = g.at(0);
  // Face centre, so the footprint is never clipped by the panel edge.
  const Vec3 where = texelCenter(pan, res / 2, res / 2) + pan.n * depth;
  g_p.clear();
  g_p.add(where, Vec3{0, 0, 0}, kWater);
  g_r.clear();
  g_r.splat(g_p, g);

  Blob b{0, 0, g_r.footprint()};
  for (int j = 0; j < res; ++j)
    for (int i = 0; i < res; ++i) {
      const int v = (int)g_r.accumAt(0, i, j, res, kChWater);
      if (v > 0) { ++b.litTexels; b.total += v; }
    }
  return b;
}

}  // namespace

TEST(renderer_footprint_tracks_the_panel_pitch) {
  // Both describe the SAME 32-unit world: 32 texels at pitch 1.0, or 64 at pitch 0.5.
  const Blob coarse = blobFor(32, 1.0f, 2.0f);
  const Blob fine = blobFor(64, 0.5f, 2.0f);

  // The old hardcoded value, preserved exactly at pitch 1.0 so the golden hashes did not move.
  CHECK(coarse.footprint == 2);
  // Half the pitch, so twice the texel radius.
  CHECK(fine.footprint == 5);
  std::printf("       footprint %d texels at pitch 1.0, %d at pitch 0.5\n", coarse.footprint,
              fine.footprint);
}

TEST(renderer_blob_covers_the_same_world_area_at_either_pitch) {
  const Blob coarse = blobFor(32, 1.0f, 2.0f);
  const Blob fine = blobFor(64, 0.5f, 2.0f);

  // TOTAL deposited intensity scales with the texel count, because the kernel is a function of
  // the radius NORMALISED by rMax -- so a texel at the same relative position in the blob gets
  // the same value at either pitch, and there are simply four times as many of them.
  const float totalRatio = (float)fine.total / (float)coarse.total;
  std::printf("       total intensity %d -> %d (%.2fx, want ~4)\n", coarse.total, fine.total,
              totalRatio);
  CHECK(totalRatio > 3.4f);
  CHECK(totalRatio < 4.6f);

  // Lit texel count grows by rather LESS than 4x, and the reason is worth recording: the outer
  // ring of the kernel is faint, and `contrib = (a * kernel) >> 6` rounds it to zero. At the
  // finer pitch a larger share of the footprint sits in that faint ring, so proportionally more
  // of it quantises away -- measured 3.3x rather than 4x. The visible consequence is a slightly
  // harder blob edge at 64x64, which is a thing to look at rather than a thing to assert.
  const float litRatio = (float)fine.litTexels / (float)coarse.litTexels;
  std::printf("       lit texels %d -> %d (%.2fx; the shortfall is the >>6 tail)\n",
              coarse.litTexels, fine.litTexels, litRatio);
  CHECK(litRatio > 3.0f);
  CHECK(litRatio < 4.4f);
}

// The constant this test exists to protect is kSplatExposure, and the answer is the opposite of
// what it looks like.
//
// It is tempting to argue that a texel covering a quarter of the world area collects a quarter of
// the light, so the exposure must drop 4x. That is true only if the footprint stays a fixed
// number of TEXELS. Once the blob is a fixed WORLD size, the set of particles within reach of a
// given texel is the same at either pitch, and each contributes the same kernel value -- so
// per-texel accumulation is pitch-invariant and the exposure carries over untouched.
TEST(renderer_peak_accumulation_is_pitch_invariant) {
  // The same physical body of water, described once in world coordinates and splatted at both
  // resolutions. Identical particles, so any difference is purely the renderer's.
  auto fillTank = [] {
    g_p.clear();
    for (int z = 0; z < 14; ++z)
      for (int y = 0; y < 8; ++y)
        for (int x = 0; x < 14; ++x) {
          const Vec3 at{-10.0f + (float)x * 1.5f, -15.0f + (float)y * 1.5f,
                        -10.0f + (float)z * 1.5f};
          g_p.add(at, Vec3{0, 0, 0}, kWater);
        }
  };

  int peaks[2] = {0, 0};
  const int res[2] = {32, 64};
  const float pitch[2] = {1.0f, 0.5f};
  for (int k = 0; k < 2; ++k) {
    const Geometry g = Geometry::cube(res[k], pitch[k]);
    g_r.init(g);
    fillTank();
    g_r.clear();
    g_r.splat(g_p, g);
    // Panel 4 is the bottom face, where a settled tank is densest.
    peaks[k] = findPeak(g_r, 4, res[k], res[k], kChWater).value;
  }

  const float ratio = (float)peaks[1] / (float)peaks[0];
  std::printf("       peak accumulation %d (pitch 1.0) -> %d (pitch 0.5) = %.2fx\n", peaks[0],
              peaks[1], ratio);
  std::printf("       kSplatExposure %.0f therefore carries over unchanged\n", kSplatExposure);
  CHECK(peaks[0] > 0);
  // Within 25%. If this ever drifts far from 1.0, kSplatExposure needs re-deriving and the
  // reasoning in the comment above has stopped being true.
  CHECK(ratio > 0.75f);
  CHECK(ratio < 1.25f);
}

// --- rendering a subset of the faces -------------------------------------------------------------

TEST(renderer_subset_only_touches_its_own_faces) {
  const Geometry g = Geometry::cube(32, 1.0f);
  const int mine[2] = {0, 4};  // the -Z face and the bottom
  CHECK(g_r.init(g, mine, 2));
  CHECK(g_r.panelCount() == 2);
  CHECK(g_r.rendersPanel(0));
  CHECK(g_r.rendersPanel(4));
  CHECK(!g_r.rendersPanel(1));
  CHECK(g_r.panelAtSlot(1) == 4);

  // A particle in a corner is near three faces; only the two we own may light up.
  g_p.clear();
  g_p.add(Vec3{-15.0f, -15.0f, -15.0f}, Vec3{0, 0, 0}, kWater);
  g_r.clear();
  g_r.splat(g_p, g);

  CHECK(totalIntensity(g_r, 0, 32, 32, kChWater) > 0);
  CHECK(totalIntensity(g_r, 4, 32, 32, kChWater) > 0);
  // Panel 2 (-X) is also within reach of that corner, and must stay dark here -- accumAt reports
  // zero for an undriven face rather than reading someone else's slot.
  CHECK(totalIntensity(g_r, 2, 32, 32, kChWater) == 0);
}

TEST(renderer_subset_is_identical_to_the_full_render_on_its_own_faces) {
  // The property the whole multi-node design rests on: a display node must produce exactly the
  // pixels a single-process render would have produced for those faces. Not approximately.
  const Geometry g = Geometry::cube(32, 1.0f);

  auto fill = [] {
    g_p.clear();
    for (int i = 0; i < 500; ++i) {
      const float t = (float)i;
      g_p.add(Vec3{-12.0f + fsin(t * 0.7f) * 10.0f, -14.0f + (float)(i % 17) * 1.3f,
                   fcos(t * 0.31f) * 11.0f},
              Vec3{0, 0, 0}, (i % 4 == 0) ? kSand : kWater);
    }
  };

  static uint64_t whole[6];
  g_r.init(g);
  fill();
  g_r.clear();
  g_r.splat(g_p, g);
  for (int k = 0; k < 6; ++k) {
    uint64_t h = 1469598103934665603ull;
    for (int j = 0; j < 32; ++j)
      for (int i = 0; i < 32; ++i)
        for (int c = 0; c < kChannelCount; ++c) {
          const uint16_t v = g_r.accumAt(k, i, j, 32, c);
          h = fnv1a(&v, sizeof(v), h);
        }
    whole[k] = h;
  }

  // Now the same particles through three 2-face nodes, and every face must match bit for bit.
  const int split[3][2] = {{0, 1}, {2, 3}, {4, 5}};
  for (int node = 0; node < 3; ++node) {
    CHECK(g_r.init(g, split[node], 2));
    fill();
    g_r.clear();
    g_r.splat(g_p, g);
    for (int s = 0; s < 2; ++s) {
      const int k = split[node][s];
      uint64_t h = 1469598103934665603ull;
      for (int j = 0; j < 32; ++j)
        for (int i = 0; i < 32; ++i)
          for (int c = 0; c < kChannelCount; ++c) {
            const uint16_t v = g_r.accumAt(k, i, j, 32, c);
            h = fnv1a(&v, sizeof(v), h);
          }
      CHECK(h == whole[k]);
    }
  }
  std::printf("       3 nodes x 2 faces reproduce all 6 faces bit-for-bit\n");
}

TEST(renderer_rejects_more_faces_than_it_has_slots) {
  const Geometry g = Geometry::cube(32, 1.0f);
  const int dup[2] = {1, 1};
  CHECK(!g_r.init(g, dup, 2));   // the same face twice would be blitted twice
  const int bad[1] = {99};
  CHECK(!g_r.init(g, bad, 1));
  int tooMany[kMaxRenderPanels + 1];
  for (int i = 0; i <= kMaxRenderPanels; ++i) tooMany[i] = 0;
  CHECK(!g_r.init(g, tooMany, kMaxRenderPanels + 1));
  // Leave the shared renderer in a sane state for whatever runs next.
  CHECK(g_r.init(g));
}
