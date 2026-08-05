#pragma once
#include "partsim/Config.h"

namespace partsim {

// Colour ramps as data, one per accumulation channel. The splat loop only ever accumulates
// scalar intensities; colour is applied once, at resolve time, by looking up these ramps.
// That is what lets a naturalistic and a neon look coexist with zero change to the physics
// or the renderer's inner loop.
constexpr int kRampStops = 8;

struct Ramp {
  uint8_t rgb[kRampStops][3];
};

struct Palette {
  const char* name;
  Ramp water;
  Ramp sand;
  Ramp heat;
};

// Linear interpolation through a ramp. `level` is 0..255.
inline void rampLookup(const Ramp& r, int level, uint8_t out[3]) {
  if (level <= 0) { out[0] = r.rgb[0][0]; out[1] = r.rgb[0][1]; out[2] = r.rgb[0][2]; return; }
  if (level >= 255) {
    out[0] = r.rgb[kRampStops - 1][0];
    out[1] = r.rgb[kRampStops - 1][1];
    out[2] = r.rgb[kRampStops - 1][2];
    return;
  }
  // level in [0,255] -> stop index and 8-bit fraction between stops.
  const int scaled = level * (kRampStops - 1);  // 0 .. 255*(stops-1)
  const int idx = scaled / 255;
  const int frac = scaled - idx * 255;
  const uint8_t* a = r.rgb[idx];
  const uint8_t* b = r.rgb[idx + 1 < kRampStops ? idx + 1 : idx];
  for (int c = 0; c < 3; ++c)
    out[c] = (uint8_t)(((int)a[c] * (255 - frac) + (int)b[c] * frac) / 255);
}

int paletteCount();
const Palette& paletteAt(int i);
const Palette& paletteNaturalistic();
const Palette& paletteNeon();

}  // namespace partsim
