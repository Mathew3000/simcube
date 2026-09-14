#include "partsim/Calibration.h"

#include "partsim/Config.h"
#include "partsim/Math.h"

namespace partsim {

namespace {

// Indexed by (sx | sy<<1 | sz<<2), sign bits 1 for '+'. Matches MINI.md's table exactly: more red
// is further along +x, more green along +y, more blue along +z, and the all-negative corner is a
// dim grey floor rather than black -- the one corner the eye cannot afford to lose.
constexpr Rgb8 kCornerColour[8] = {
    {24, 24, 24},    // ---  dim grey (floor, not black: this corner must stay visible)
    {255, 0, 0},     // +--  red
    {0, 255, 0},     // -+-  green
    {255, 255, 0},   // ++-  yellow
    {0, 0, 255},     // --+  blue
    {255, 0, 255},   // +-+  magenta
    {0, 255, 255},   // -++  cyan
    {255, 255, 255},  // +++  white
};

constexpr Rgb8 kOff{0, 0, 0};
constexpr Rgb8 kIndexMark{255, 255, 255};

}  // namespace

Rgb8 calibrationTexel(const Geometry& g, int panel, int i, int j) {
  const Panel& p = g.at(panel);
  const Vec3 world = texelCenter(p, i, j);
  // kSlabDepth's inward push does not move a closed cube's box centre (the six quads already
  // span the box exactly -- see Geometry::bounds), so this is the true box centre for the shape
  // this pattern is built for, and the sign it produces is unaffected by pitch or resolution.
  const Vec3 center = g.bounds(kSlabDepth).center();
  const Vec3 d = world - center;
  const int idx = (d.x >= 0.0f ? 1 : 0) | (d.y >= 0.0f ? 2 : 0) | (d.z >= 0.0f ? 4 : 0);
  const Rgb8 corner = kCornerColour[idx];

  const int w = (int)p.w, h = (int)p.h;
  // 2 of 8 at the shipping 8x8 resolution, per MINI.md; scaled rather than hardcoded so the same
  // generator is correct (and host-testable) at 32 and 64 too.
  const int block = imax(1, imin(w, h) / 4);
  const bool cornerCol = (i < block) || (i >= w - block);
  const bool cornerRow = (j < block) || (j >= h - block);
  if (cornerCol && cornerRow) return corner;

  // The face-index bar: one row, free across the FULL width because the corner blocks occupy the
  // outer rows and the outer columns only where they coincide. `panel` is the geometry panel
  // index, which is also the face index -- same convention PanelDriver::testPattern uses.
  const int indexRow = h / 2;
  if (j == indexRow && i <= panel) return kIndexMark;
  return kOff;
}

}  // namespace partsim
