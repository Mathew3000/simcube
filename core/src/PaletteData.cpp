#include "partsim/Palette.h"

namespace partsim {
namespace {

// Ramps run dark -> bright. Index 0 is what an almost-empty texel shows, so it must be
// black or the panel glows everywhere and the fluid loses its shape.
//
// LED panels are punchy and have no highlight headroom above full white, so these ramps
// spend most of their range in the mid tones and only touch white at the very top.
const Palette kPalettes[] = {
    {"naturalistic",
     // Water: deep blue -> cyan -> white foam at the crest.
     {{{0, 0, 0}, {0, 6, 24}, {0, 22, 72}, {0, 54, 132}, {8, 104, 190}, {60, 168, 226},
       {150, 216, 244}, {235, 250, 255}}},
     // Sand: warm dark ochre -> pale sand. Deliberately desaturated next to the water so
     // the two read as different materials rather than two blues.
     {{{0, 0, 0}, {18, 12, 4}, {52, 36, 14}, {96, 70, 30}, {140, 108, 50},
       {188, 152, 84}, {216, 190, 138}, {242, 228, 196}}},
     // Fire on a blackbody-ish ramp: ember red -> orange -> yellow -> white hot.
     {{{0, 0, 0}, {40, 4, 0}, {104, 16, 0}, {170, 48, 0}, {216, 100, 4}, {242, 160, 20},
       {252, 216, 90}, {255, 250, 210}}}},

    {"neon",
     // Water: violet -> magenta -> electric cyan. Saturated end to end.
     {{{0, 0, 0}, {16, 0, 40}, {48, 0, 96}, {104, 0, 168}, {168, 16, 200}, {216, 60, 220},
       {120, 200, 255}, {230, 255, 255}}},
     // Sand: acid lime, nothing like the water.
     {{{0, 0, 0}, {12, 20, 0}, {36, 60, 0}, {72, 116, 0}, {124, 180, 8}, {180, 228, 40},
       {224, 250, 120}, {250, 255, 210}}},
     // Fire: hot pink -> orange -> white.
     {{{0, 0, 0}, {48, 0, 24}, {112, 0, 56}, {188, 0, 88}, {236, 40, 60}, {252, 120, 40},
       {255, 200, 100}, {255, 245, 220}}}},
};

constexpr int kCount = (int)(sizeof(kPalettes) / sizeof(kPalettes[0]));

}  // namespace

int paletteCount() { return kCount; }
const Palette& paletteAt(int i) { return kPalettes[i < 0 ? 0 : (i >= kCount ? kCount - 1 : i)]; }
const Palette& paletteNaturalistic() { return kPalettes[0]; }
const Palette& paletteNeon() { return kPalettes[1]; }

}  // namespace partsim
