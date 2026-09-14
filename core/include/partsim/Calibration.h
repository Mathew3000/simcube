#pragma once
#include <cstdint>

#include "partsim/Geometry.h"

namespace partsim {

// The final colour a texel shows, not an intensity to accumulate -- the orientation pattern
// bypasses the renderer entirely, the same way PanelDriver::testPattern's hand-picked hues do.
struct Rgb8 {
  uint8_t r, g, b;
};

// The orientation calibration pattern (MINI.md M3): colours the eight corners of the box by the
// sign of each axis relative to the box centre, so the SET of four corners a face shows names the
// face, their positions name the rotation, and their cyclic order names the mirror -- and because
// three faces meet at every corner, three panels showing the same corner must show the same
// colour. See tests/test_calibration.cpp for both halves of that claim, including the one that
// proves a wrong mount is actually detectable rather than merely plausible.
//
// Derived from the panel's own basis and the box centre, not a per-face table: a table is six
// chances to make the mistake this pattern exists to catch, and it would need rewriting for every
// panel resolution this project supports. `panel` doubles as the face index for the index bar
// below, the same convention PanelDriver::testPattern uses ("hue by the GEOMETRY panel, not the
// driven index, so face colours mean the same thing on every node").
Rgb8 calibrationTexel(const Geometry& g, int panel, int i, int j);

}  // namespace partsim
