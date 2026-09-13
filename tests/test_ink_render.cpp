#include "check.h"
#include "partsim/Geometry.h"
#include "partsim/InkField.h"
#include "partsim/Renderer.h"
#include "partsim/SimVolume.h"

#if PARTSIM_ENABLE_INK

using namespace partsim;

namespace {

// Own instances; both are far too big for the stack.
Renderer g_r;
InkField g_f;

// Panel order from Geometry::cube, as platform/host/bench.cpp names them.
constexpr int kMinusZ = 0, kPlusZ = 1, kMinusX = 2, kPlusX = 3;

SimVolume setup() {
  SimVolume v;
  const bool okV = v.build(Geometry::cube(32, 1.0f), 2.0f, kCellSize);
  CHECK(okV);
  CHECK(g_f.init(v));
  return v;
}

// Sum of one accumulation channel over a whole face. Summed rather than peaked: the question
// these tests ask is which face the dye is in front of, and one texel cannot answer it.
long faceTotal(int panel, int channel) {
  long t = 0;
  const int w = g_r.panelWidth(panel);
  const int h = w > 0 ? g_r.panelTexels(panel) / w : 0;
  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; ++i) t += (long)g_r.accumAt(panel, i, j, channel);
  return t;
}

int litTexels(int panel) {
  int n = 0;
  const int w = g_r.panelWidth(panel);
  const int h = w > 0 ? g_r.panelTexels(panel) / w : 0;
  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; ++i)
      if (g_r.accumAt(panel, i, j, kChWater) > 0) ++n;
  return n;
}

}  // namespace

TEST(ink_projection_of_a_clear_volume_draws_nothing) {
  const SimVolume v = setup();
  CHECK(g_r.init(Geometry::cube(32, 1.0f)));
  g_r.clear();
  g_r.splatInk(g_f, Geometry::cube(32, 1.0f));
  (void)v;
  for (int p = 0; p < 6; ++p) CHECK(faceTotal(p, kChWater) == 0);
}

TEST(ink_projection_lights_every_face_from_one_shared_volume) {
  const SimVolume v = setup();
  (void)v;
  const Geometry g = Geometry::cube(32, 1.0f);
  CHECK(g_r.init(g));
  g_f.inject(Vec3{0.0f, 0.0f, 0.0f}, 8.0f, 0, 255);

  g_r.clear();
  g_r.splatInk(g_f, g);

  // One field, six views of it. A blob in the middle must be visible from all six sides -- this
  // is the property six independent 2D simulations could not have.
  for (int p = 0; p < 6; ++p) CHECK(faceTotal(p, kChWater) > 0);
}

TEST(ink_opacity_is_the_same_from_both_sides_of_the_volume) {
  const SimVolume v = setup();
  (void)v;
  const Geometry g = Geometry::cube(32, 1.0f);
  CHECK(g_r.init(g));
  // Panel 0 is -Z, so its inward normal is +z: this dye is pressed against it, far from +Z.
  g_f.inject(Vec3{0.0f, 0.0f, -10.0f}, 5.0f, 0, 255);

  g_r.clear();
  g_r.splatInk(g_f, g);

  // Opposite faces must agree on HOW MUCH is in the way, however lopsidedly it is arranged:
  // transmittance is a product over the column, and a product does not care about order. It is
  // colour that depends on which end you look from -- see the occlusion test below.
  //
  // Worth asserting precisely because the intuition is the other way round. "The near face should
  // be brighter" is wrong, and a test written to that belief would have been made to pass by
  // breaking the compositor.
  const long near = faceTotal(kMinusZ, kChWater);
  const long far = faceTotal(kPlusZ, kChWater);
  CHECK(near > 0);
  CHECK(near == far);
}

#if PARTSIM_INK_CHANNELS > 1
TEST(ink_nearer_dye_occludes_farther_dye_per_face) {
  const SimVolume v = setup();
  (void)v;
  const Geometry g = Geometry::cube(32, 1.0f);
  CHECK(g_r.init(g));

  // Red near -Z, blue near +Z, on the same columns. Each face should read as the colour of
  // whichever blob is in FRONT of it.
  g_f.inject(Vec3{0.0f, 0.0f, -9.0f}, 5.0f, 0, 255);
  g_f.inject(Vec3{0.0f, 0.0f, 9.0f}, 5.0f, 1, 255);

  g_r.clear();
  g_r.splatInk(g_f, g);

#if PARTSIM_ENABLE_CHROMA
  const long rMinus = faceTotal(kMinusZ, kChCR), bMinus = faceTotal(kMinusZ, kChCB);
  const long rPlus = faceTotal(kPlusZ, kChCR), bPlus = faceTotal(kPlusZ, kChCB);

  // -Z looks through the red blob first, +Z through the blue one. Front-to-back compositing is
  // the whole reason these differ; a back-to-front walk, or one shared depth order for opposite
  // faces, would swap or equalise them.
  CHECK(rMinus > bMinus);
  CHECK(bPlus > rPlus);
#endif
}
#endif

TEST(ink_projection_covers_the_whole_panel_after_upscale) {
  const SimVolume v = setup();
  (void)v;
  const Geometry g = Geometry::cube(32, 1.0f);
  CHECK(g_r.init(g));
  // Fill the volume, so every column has dye and every panel texel must be lit. A bilinear
  // upscale that sampled at corners rather than centres leaves an unlit row and column at one
  // edge, which on a cube reads as a dark seam exactly where two faces meet.
  g_f.inject(Vec3{0.0f, 0.0f, 0.0f}, 40.0f, 0, 255);

  g_r.clear();
  g_r.splatInk(g_f, g);

  for (int p = 0; p < 6; ++p) {
    const int w = g_r.panelWidth(p);
    const int h = w > 0 ? g_r.panelTexels(p) / w : 0;
    CHECK(litTexels(p) == w * h);
  }
}

TEST(ink_projection_puts_an_offset_blob_on_the_same_side_of_both_neighbours) {
  const SimVolume v = setup();
  (void)v;
  const Geometry g = Geometry::cube(32, 1.0f);
  CHECK(g_r.init(g));
  // Offset along +x only. Seen from -Z and from +Z the blob must appear on OPPOSITE sides of the
  // image, because those two faces look at the volume from opposite directions. A per-face axis
  // transpose would put it at the wrong end of the wrong axis, which is hard to spot by eye on a
  // roughly symmetric cube.
  g_f.inject(Vec3{9.0f, 0.0f, 0.0f}, 5.0f, 0, 255);

  g_r.clear();
  g_r.splatInk(g_f, g);

  auto centroidU = [](int panel) {
    double s = 0.0, wsum = 0.0;
    const int w = g_r.panelWidth(panel);
    const int h = w > 0 ? g_r.panelTexels(panel) / w : 0;
    for (int j = 0; j < h; ++j)
      for (int i = 0; i < w; ++i) {
        const double a = g_r.accumAt(panel, i, j, kChWater);
        s += a * i;
        wsum += a;
      }
    return wsum > 0.0 ? s / wsum : -1.0;
  };

  const double cMinus = centroidU(kMinusZ), cPlus = centroidU(kPlusZ);
  CHECK(cMinus >= 0.0 && cPlus >= 0.0);
  const int w = g_r.panelWidth(kMinusZ);
  // Mirrored about the panel centre, to within the blob's own softness.
  CHECK_NEAR(cMinus + cPlus, (double)(w - 1), 2.0);

  // And on the two faces whose normal IS x, the blob is a depth offset, not a lateral one: both
  // should centre it.
  CHECK_NEAR(centroidU(kMinusX), (double)(w - 1) * 0.5, 2.0);
  CHECK_NEAR(centroidU(kPlusX), (double)(w - 1) * 0.5, 2.0);
}

TEST(ink_projection_is_deterministic) {
  const SimVolume v = setup();
  (void)v;
  const Geometry g = Geometry::cube(32, 1.0f);
  CHECK(g_r.init(g));
  g_f.inject(Vec3{2.0f, -3.0f, 4.0f}, 6.0f, 0, 200);

  g_r.clear();
  g_r.splatInk(g_f, g);
  const long a = faceTotal(kMinusZ, kChWater) * 31 + faceTotal(kPlusX, kChWater);

  g_r.clear();
  g_r.splatInk(g_f, g);
  const long b = faceTotal(kMinusZ, kChWater) * 31 + faceTotal(kPlusX, kChWater);
  CHECK(a == b);
}

#endif  // PARTSIM_ENABLE_INK
