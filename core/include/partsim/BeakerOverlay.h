#pragma once
#include "partsim/Font3x5.h"
#include "partsim/Geometry.h"
#include "partsim/Renderer.h"

namespace partsim {

// A colour the overlay asks for, 0..255 per component.
//
// Deliberately NOT an RGB write: the overlay draws into the ACCUMULATION buffers, which carry
// material intensity, and colour is decided once in Renderer::resolve() through the palette
// ramps. What this struct expresses is the colour the overlay wants; how much of it survives
// depends on which channels the build has (see BeakerOverlay.cpp, and docs/M4-C-FINDINGS.md --
// without PARTSIM_ENABLE_CHROMA there is exactly one channel and red is not representable).
struct OverlayRgb {
  uint8_t r, g, b;
};

constexpr OverlayRgb kOverlayWhite{255, 255, 255};
constexpr OverlayRgb kOverlayRed{255, 0, 0};

// The orientation latch.
//
// Armed on entering beaker mode; released the first time gravity lands within tolerance of the
// cube's own down axis; NEVER re-armed. Tilting is how you pour, so a gate that re-armed on tilt
// would make pouring impossible -- that is the single property this class exists to guarantee,
// and it is why release() has no counterpart other than an explicit arm().
class OrientationGate {
 public:
  // Cosine of the half-angle that counts as "upright" -- 20 degrees.
  //
  // A judgement, not a measurement. The accelerometer fusion has a ~0.35 s time constant
  // (MotionConfig::defaults), so a tolerance tight enough to demand a few degrees would need the
  // cube held still as well as upright, and this gate is meant to be satisfied by a person
  // standing a cube on a table. 20 degrees is roughly "visibly the right way up".
  static constexpr float kUprightCos = 0.9396926f;

  void arm() { armed_ = true; }
  bool armed() const { return armed_; }

  // `down` is unit object-space down, i.e. MotionSource::down(). `objectDown` is the cube's own
  // down axis, -Y for every geometry this project builds; a parameter rather than a constant so
  // a test can state it rather than assume it.
  //
  // Returns the armed state after the update, so a caller can gate a step in one line.
  bool update(Vec3 down, Vec3 objectDown = Vec3{0.0f, -1.0f, 0.0f}) {
    if (!armed_) return false;
    if (dot(normalize(down), normalize(objectDown)) >= kUprightCos) armed_ = false;
    return armed_;
  }

 private:
  bool armed_ = false;
};

// Draws the vessel: white edge lines on every face but the top, and, while the gate is armed,
// the red orientation glyphs.
//
// Faces are classified from the GEOMETRY, not from panel indices. Geometry::cube happens to put
// the top face at index 5, but a renderer on a display node drives an arbitrary subset and the
// panel table is built from specs, so "which one is the top" is a question about normals.
class BeakerOverlay {
 public:
  enum Face : uint8_t { kSide = 0, kTop, kBottom };

  // `objectUp` is the cube's own up axis in object space.
  void init(const Geometry& g, Vec3 objectUp = Vec3{0.0f, 1.0f, 0.0f});

  int panelCount() const { return count_; }
  Face faceKind(int panel) const {
    return (panel >= 0 && panel < count_) ? kind_[panel] : kSide;
  }
  // Panel-space direction of object-up, as a signed unit step in (i, j). Zero for the caps,
  // where up is along the normal and has no in-panel direction.
  int upI(int panel) const { return (panel >= 0 && panel < count_) ? upI_[panel] : 0; }
  int upJ(int panel) const { return (panel >= 0 && panel < count_) ? upJ_[panel] : 0; }

  // Edge lines. Every edge of the cube except the four belonging to the top face: the sides draw
  // their two vertical borders and their bottom border but NOT their top row, and the top face
  // draws nothing at all. That missing rim is what makes the cube read as an open vessel rather
  // than as a box.
  void drawEdges(Renderer& r) const;

  // Gate glyphs: an arrow pointing along object-up on each side face, TOP on the top face and
  // BOTTOM on the bottom one.
  void drawGateGlyphs(Renderer& r) const;

  // Both, with the glyphs conditional. The one call a render loop needs.
  void draw(Renderer& r, bool gateArmed) const {
    drawEdges(r);
    if (gateArmed) drawGateGlyphs(r);
  }

  // Text, centred in the panel and wrapped onto as many lines as it takes. Public because the
  // console overlay is the next thing that wants it, and because it is the part worth testing
  // directly. `scale` <= 0 picks the scale from the panel width.
  void drawText(Renderer& r, int panel, const char* text, OverlayRgb colour, int scale = 0) const;

  // Glyph scale for a panel width: 1 at 32 texels, 2 at 64. One table drawn larger, rather than
  // a second table.
  static int scaleFor(int panelWidth) { return imax(1, panelWidth / 32); }
  // Width in texels of `text` rendered at `scale`, with no wrapping.
  static int textWidth(const char* text, int scale);

 private:
  void plot(Renderer& r, int panel, int i, int j, OverlayRgb c) const;
  void drawArrow(Renderer& r, int panel, OverlayRgb c) const;

  int count_ = 0;
  Face kind_[kMaxPanels];
  int8_t upI_[kMaxPanels];
  int8_t upJ_[kMaxPanels];
};

}  // namespace partsim
