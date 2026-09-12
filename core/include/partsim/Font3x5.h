#pragma once
#include <cstdint>

namespace partsim {

// A 3x5 bitmap font, as a const table.
//
// `const` rather than built at startup for the same reason ScenePresets and PaletteData are:
// it lands in flash on the device and costs no RAM, and tests/test_noalloc.cpp arms a trap
// against anything that would allocate after init.
//
// 3x5 is the smallest box in which the Latin alphabet stays distinguishable, and on a 32x32
// face it is the largest box in which a six-character word still fits on one line
// (see docs/M4-C-FINDINGS.md for the measurement). Glyphs are drawn at an integer scale, so a
// 64x64 face gets the same table at 2x rather than a second table.
constexpr int kGlyphW = 3;
constexpr int kGlyphH = 5;
// Gap between glyphs, and between text lines, in unscaled texels.
constexpr int kGlyphGap = 1;

// Row `gy` of the glyph for `c`, as 3 bits with bit 2 the LEFTMOST texel -- the same order the
// literals in the table are written in, so the table reads as a picture of the character.
// Row 0 is the TOP row of the glyph.
//
// An unknown character resolves to a filled 3x5 box: a missing glyph shows up as a solid block
// on the panel rather than as a silent hole in the middle of a word.
uint8_t glyphRow(char c, int gy);

// Whether `c` is actually in the table (as opposed to resolving to the missing-glyph box).
bool glyphKnown(char c);

// The table itself, for tests. A font is exactly the kind of data where one wrong row is
// invisible until someone reads that character on hardware, so the test walks all of it.
int glyphCount();
char glyphCharAt(int i);

}  // namespace partsim
