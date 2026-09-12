#include "check.h"
#include "partsim/BeakerOverlay.h"
#include "partsim/Simulation.h"

using namespace partsim;

namespace {

// ~330KB of accumulation buffers; far too big for the stack.
Renderer g_r;
BeakerOverlay g_ov;
Simulation g_sim;

int panelHeight(const Renderer& r, int panel) {
  const int w = r.panelWidth(panel);
  return (w > 0) ? r.panelTexels(panel) / w : 0;
}

bool lit(const Renderer& r, int panel, int i, int j) {
  return r.accumAt(panel, i, j, kChWater) > 0;
}

int litCount(const Renderer& r, int panel) {
  const int w = r.panelWidth(panel);
  const int h = panelHeight(r, panel);
  int n = 0;
  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; ++i)
      if (lit(r, panel, i, j)) ++n;
  return n;
}

// Geometry::cube: 0 = -Z, 1 = +Z, 2 = -X, 3 = +X, 4 = -Y (bottom), 5 = +Y (top).
constexpr int kTopPanel = 5;
constexpr int kBottomPanel = 4;

void setUp(int res = 32) {
  const Geometry g = Geometry::cube(res, 1.0f);
  g_r.init(g);
  g_ov.init(g);
  g_r.clear();
}

}  // namespace

TEST(beaker_faces_are_classified_from_the_normals_not_the_indices) {
  const Geometry g = Geometry::cube(32, 1.0f);
  g_ov.init(g);
  CHECK(g_ov.faceKind(kTopPanel) == BeakerOverlay::kTop);
  CHECK(g_ov.faceKind(kBottomPanel) == BeakerOverlay::kBottom);
  for (int k = 0; k < 4; ++k) CHECK(g_ov.faceKind(k) == BeakerOverlay::kSide);
  // Every side face of Geometry::cube is built with up = +Y, so panel +v is object-up on all
  // four. If that ever stops being true the arrows are the thing that breaks.
  for (int k = 0; k < 4; ++k) {
    CHECK(g_ov.upJ(k) == 1);
    CHECK(g_ov.upI(k) == 0);
  }
}

TEST(beaker_edges_land_on_the_expected_texels) {
  setUp();
  g_ov.drawEdges(g_r);

  const int w = g_r.panelWidth(0), h = panelHeight(g_r, 0);

  // Four side faces: both vertical borders full height, the bottom row, and NOT the top row.
  for (int p = 0; p < 4; ++p) {
    for (int j = 0; j < h; ++j) {
      CHECK(lit(g_r, p, 0, j));
      CHECK(lit(g_r, p, w - 1, j));
    }
    for (int i = 0; i < w; ++i) CHECK(lit(g_r, p, i, 0));
    // The rim shared with the open top: only the two corner texels, which belong to the
    // vertical edges, are lit.
    for (int i = 1; i < w - 1; ++i) CHECK(!lit(g_r, p, i, h - 1));
    // ...and nothing in the interior.
    for (int j = 1; j < h - 1; ++j)
      for (int i = 1; i < w - 1; ++i) CHECK(!lit(g_r, p, i, j));
    CHECK(litCount(g_r, p) == 2 * h + (w - 2));
  }

  // The bottom cap is closed, so it draws all four of its borders.
  for (int i = 0; i < w; ++i) {
    CHECK(lit(g_r, kBottomPanel, i, 0));
    CHECK(lit(g_r, kBottomPanel, i, h - 1));
  }
  for (int j = 0; j < h; ++j) {
    CHECK(lit(g_r, kBottomPanel, 0, j));
    CHECK(lit(g_r, kBottomPanel, w - 1, j));
  }
  CHECK(litCount(g_r, kBottomPanel) == 2 * w + 2 * h - 4);

  // The top face is open: not one texel.
  CHECK(litCount(g_r, kTopPanel) == 0);
}

TEST(beaker_edges_are_additive_and_do_not_dim_the_fluid) {
  // An overlay that ASSIGNED would punch a hole in bright fluid wherever it wrote a dimmer
  // value, so an edge line would flicker dark exactly where the liquid touches it.
  setUp();
  g_r.addAccum(0, 0, 5, kChWater, 40000);
  const uint16_t before = g_r.accumAt(0, 0, 5, kChWater);
  g_ov.drawEdges(g_r);
  CHECK(g_r.accumAt(0, 0, 5, kChWater) == before + (uint16_t)kSplatExposure);

  // ...and it saturates rather than wrapping, which is the failure that would look like the
  // overlay punching black holes in the brightest texels.
  g_r.addAccum(0, 0, 5, kChWater, 65535);
  CHECK(g_r.accumAt(0, 0, 5, kChWater) == 65535);
  g_ov.drawEdges(g_r);
  CHECK(g_r.accumAt(0, 0, 5, kChWater) == 65535);
}

// What the overlay actually RESOLVES to. The overlay knows nothing about palettes -- it writes
// kSplatExposure into the weight channel, which is exactly the intensity resolve() maps to ramp
// level 255, and the dye channels when the build has them. These two cases are the difference
// between "red arrows" being true and being aspirational; see docs/M4-C-FINDINGS.md.
namespace {
static uint8_t g_px[kMaxPanelTexels * 4];

void resolvePanel(int panel) {
  g_r.setPalette(&paletteNaturalistic());
  g_r.setExposure(kSplatExposure);
  for (int i = 0; i < kMaxPanelTexels * 4; ++i) g_px[i] = 0;
  g_r.resolve(panel, g_px, 4);
}
const uint8_t* at(int panel, int i, int j) {
  return g_px + ((size_t)j * (size_t)g_r.panelWidth(panel) + (size_t)i) * 4;
}
}  // namespace

#if PARTSIM_ENABLE_CHROMA
namespace {
int iabsDiff(int a, int b) { return a > b ? a - b : b - a; }
}  // namespace

TEST(beaker_overlay_resolves_white_lines_and_red_arrows) {
  setUp();
  g_ov.draw(g_r, true);
  resolvePanel(0);

  // An edge-line texel: neutral and bright.
  const uint8_t* line = at(0, 0, 5);
  CHECK(line[0] > 200 && line[1] > 200 && line[2] > 200);
  CHECK(iabsDiff(line[0], line[1]) < 24 && iabsDiff(line[1], line[2]) < 24);

  // ...and the arrow's shaft, which runs up the centre column: red, not white.
  const uint8_t* arrow = at(0, 16, 16);
  CHECK(arrow[0] > 150);
  CHECK(arrow[1] < 40 && arrow[2] < 40);
}
#else
TEST(beaker_overlay_resolves_to_the_top_of_the_water_ramp) {
  setUp();
  g_ov.drawEdges(g_r);
  CHECK(g_r.accumAt(0, 0, 5, kChWater) == (uint16_t)kSplatExposure);
  resolvePanel(0);

  const uint8_t* top = paletteNaturalistic().water.rgb[kRampStops - 1];
  const uint8_t* line = at(0, 0, 5);
  CHECK(line[0] == top[0] && line[1] == top[1] && line[2] == top[2]);
  // ...which is white enough to read as an edge line: every component above 230.
  CHECK(line[0] > 230 && line[1] > 230 && line[2] > 230);

  // And the honest half of it: without chroma there is one channel and one ramp, so the arrow
  // resolves to the SAME colour as the edge lines. Red is a missing channel, not a missing
  // branch. This assertion is here so that the day chroma lands, the case above takes over and
  // this one compiles out -- rather than the limitation quietly persisting untested.
  g_r.clear();
  g_ov.drawGateGlyphs(g_r);
  resolvePanel(0);
  const uint8_t* arrow = at(0, 16, 16);
  CHECK(arrow[0] == top[0] && arrow[1] == top[1] && arrow[2] == top[2]);
}
#endif

TEST(beaker_gate_glyphs_are_inside_the_edge_lines) {
  setUp();
  g_ov.drawGateGlyphs(g_r);

  const int w = g_r.panelWidth(0), h = panelHeight(g_r, 0);
  for (int p = 0; p < 6; ++p) {
    CHECK(litCount(g_r, p) > 0);  // every face says something while the gate is armed
    for (int j = 0; j < h; ++j) {
      CHECK(!lit(g_r, p, 0, j));
      CHECK(!lit(g_r, p, w - 1, j));
    }
    for (int i = 0; i < w; ++i) {
      CHECK(!lit(g_r, p, i, 0));
      CHECK(!lit(g_r, p, i, h - 1));
    }
  }
}

TEST(beaker_bottom_reads_BOTTOM_on_one_line_at_32) {
  // The handoff expected BOTTOM to need wrapping at 32x32. It does not: six glyphs is
  // 6*3 + 5*1 = 23 texels against 28 available inside the margins. Asserted rather than
  // asserted-about, because it is the reason the wrap does not trigger here.
  CHECK(BeakerOverlay::textWidth("BOTTOM", 1) == 23);
  CHECK(BeakerOverlay::scaleFor(32) == 1);
  CHECK(BeakerOverlay::scaleFor(64) == 2);

  setUp();
  g_ov.drawText(g_r, kBottomPanel, "BOTTOM", kOverlayWhite);
  // One line means every lit texel sits within a single 5-row band.
  int minJ = 1 << 20, maxJ = -1;
  for (int j = 0; j < panelHeight(g_r, kBottomPanel); ++j)
    for (int i = 0; i < g_r.panelWidth(kBottomPanel); ++i)
      if (lit(g_r, kBottomPanel, i, j)) {
        minJ = imin(minJ, j);
        maxJ = imax(maxJ, j);
      }
  CHECK(maxJ - minJ + 1 == kGlyphH);
}

TEST(beaker_text_wraps_when_it_does_not_fit) {
  setUp();
  // Wide enough to need two lines at 32 texels: 9 glyphs is 35 texels against 28 available.
  g_ov.drawText(g_r, kBottomPanel, "ABCDEFGHI", kOverlayWhite);
  int minJ = 1 << 20, maxJ = -1, maxI = -1;
  for (int j = 0; j < panelHeight(g_r, kBottomPanel); ++j)
    for (int i = 0; i < g_r.panelWidth(kBottomPanel); ++i)
      if (lit(g_r, kBottomPanel, i, j)) {
        minJ = imin(minJ, j);
        maxJ = imax(maxJ, j);
        maxI = imax(maxI, i);
      }
  CHECK(maxJ - minJ + 1 == 2 * kGlyphH + kGlyphGap);
  CHECK(maxI < g_r.panelWidth(kBottomPanel));  // and nothing ran off the edge
}

TEST(beaker_overlay_stays_on_the_panels_this_node_drives) {
  // A display node holds the full six-panel table but allocates buffers for two faces. The
  // overlay must draw on those and silently skip the rest, not address a slot it does not own.
  const Geometry g = Geometry::cube(32, 1.0f);
  const int mine[2] = {0, kTopPanel};
  CHECK(g_r.init(g, mine, 2));
  g_ov.init(g);
  g_r.clear();
  g_ov.draw(g_r, true);
  CHECK(litCount(g_r, 0) > 0);
  CHECK(litCount(g_r, kTopPanel) > 0);  // TOP still shows; only its EDGES are suppressed
  for (int p = 1; p < 4; ++p) CHECK(litCount(g_r, p) == 0);
}

// --- the latch ---------------------------------------------------------------------------

TEST(beaker_gate_releases_only_when_upright_and_never_re_arms) {
  OrientationGate gate;
  CHECK(!gate.armed());  // not armed until beaker mode is entered
  gate.arm();
  CHECK(gate.armed());

  // Held on its side: down points along -X, 90 degrees from the cube's own down axis.
  CHECK(gate.update(Vec3{-1.0f, 0.0f, 0.0f}));
  // Tilted 45 degrees: still outside the 20-degree tolerance.
  CHECK(gate.update(normalize(Vec3{-1.0f, -1.0f, 0.0f})));
  // Just outside, at ~21 degrees.
  CHECK(gate.update(Vec3{0.36f, -0.933f, 0.0f}));
  // Just inside, at ~11 degrees.
  CHECK(!gate.update(Vec3{0.2f, -0.98f, 0.0f}));

  // ...and it stays released through a full inversion, because tilting is how you pour.
  CHECK(!gate.update(Vec3{-1.0f, 0.0f, 0.0f}));
  CHECK(!gate.update(Vec3{0.0f, 1.0f, 0.0f}));
  CHECK(!gate.update(Vec3{0.0f, -1.0f, 0.0f}));
  CHECK(!gate.armed());
}

TEST(beaker_gate_holds_the_simulation_still_while_armed) {
  // "The simulation does not step" as an assertion about state, not about a call count: a gated
  // frame must leave the particles bit-identical.
  CHECK(g_sim.init(Simulation::kCube, 128, 0xBEEFu, 32));
  g_sim.setGravityObject(Vec3{-kGravityMag, 0.0f, 0.0f});  // held on its side

  OrientationGate gate;
  gate.arm();
  const uint32_t before = g_sim.stateHash();
  const int nBefore = g_sim.particleCount();

  for (int f = 0; f < 60; ++f)
    if (!gate.update(Vec3{-1.0f, 0.0f, 0.0f})) g_sim.stepFixed();
  CHECK(gate.armed());
  CHECK(g_sim.stateHash() == before);
  CHECK(g_sim.particleCount() == nBefore);

  // Stood upright once, it releases and the fluid moves from the next frame on.
  CHECK(!gate.update(Vec3{0.0f, -1.0f, 0.0f}));
  for (int f = 0; f < 60; ++f)
    if (!gate.update(Vec3{-1.0f, 0.0f, 0.0f})) g_sim.stepFixed();
  CHECK(!gate.armed());
  CHECK(g_sim.stateHash() != before);
}
