#include "check.h"
#include "partsim/Font3x5.h"

using namespace partsim;

namespace {

uint16_t bitmapOf(char c) {
  uint16_t b = 0;
  for (int gy = 0; gy < kGlyphH; ++gy) b = (uint16_t)(b | ((uint16_t)glyphRow(c, gy) << (gy * kGlyphW)));
  return b;
}

int rowsUsed(char c) {
  int n = 0;
  for (int gy = 0; gy < kGlyphH; ++gy)
    if (glyphRow(c, gy)) ++n;
  return n;
}

}  // namespace

// A font table is exactly the kind of data where one wrong row is invisible until someone reads
// that character on hardware, so these walk all of it rather than spot-checking.

TEST(font_every_glyph_fits_its_box) {
  for (int k = 0; k < glyphCount(); ++k) {
    const char c = glyphCharAt(k);
    // Rows are 3 bits wide by construction; assert it anyway, because the packing is the one
    // part of the table nobody can proofread.
    for (int gy = 0; gy < kGlyphH; ++gy) CHECK(glyphRow(c, gy) <= 0b111);
    // ...and nothing past row 4 exists.
    CHECK(glyphRow(c, kGlyphH) == 0);
    CHECK(glyphRow(c, -1) == 0);
  }
}

TEST(font_every_glyph_is_non_empty_except_space) {
  for (int k = 0; k < glyphCount(); ++k) {
    const char c = glyphCharAt(k);
    if (c == ' ') {
      CHECK(bitmapOf(c) == 0);
      continue;
    }
    CHECK(bitmapOf(c) != 0);
    // A one-row glyph is almost always a typo. '-' and '.' are the two that mean it.
    if (c != '-' && c != '.') CHECK(rowsUsed(c) >= 2);
  }
}

TEST(font_no_two_glyphs_share_a_bitmap) {
  // A duplicated bitmap is how a copy-paste error in a table like this survives review: the
  // build is green, the tests are green, and one letter renders as another on the panel.
  for (int a = 0; a < glyphCount(); ++a)
    for (int b = a + 1; b < glyphCount(); ++b) {
      const char ca = glyphCharAt(a), cb = glyphCharAt(b);
      if (ca == ' ' || cb == ' ') continue;
      CHECK(bitmapOf(ca) != bitmapOf(cb));
    }
}

TEST(font_covers_the_characters_beaker_mode_needs) {
  for (const char* s = "TOPBOTTOM"; *s; ++s) CHECK(glyphKnown(*s));
  for (char c = 'A'; c <= 'Z'; ++c) CHECK(glyphKnown(c));
  for (char c = '0'; c <= '9'; ++c) CHECK(glyphKnown(c));
}

TEST(font_lowercase_folds_and_unknowns_are_visible) {
  CHECK(bitmapOf('t') == bitmapOf('T'));
  // An unknown character draws a filled box rather than nothing, so a gap in the table shows up
  // on the panel instead of leaving a silent hole in the middle of a word.
  CHECK(!glyphKnown('\x01'));
  for (int gy = 0; gy < kGlyphH; ++gy) CHECK(glyphRow('\x01', gy) == 0b111);
}
