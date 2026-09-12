#include "partsim/Font3x5.h"

namespace partsim {
namespace {

// Five rows of three bits, packed low-to-high: row 0 occupies bits 0..2. Written through a macro
// so the literals in the table below stay stacked as a picture of the glyph -- the encoding is
// the thing nobody can proofread, so it appears exactly once.
#define ROWS(r0, r1, r2, r3, r4)                                                           \
  (uint16_t)((r0) | ((r1) << 3) | ((r2) << 6) | ((r3) << 9) | ((r4) << 12))

struct Glyph {
  char c;
  uint16_t rows;
};

// Uppercase, digits and the handful of symbols a status line needs. The task asks for about ten
// characters (TOP, BOTTOM); the whole alphabet is 5 bytes a letter of flash and spares the next
// person the same exercise. tests/test_font.cpp asserts every entry is non-empty, uses more than
// one row, and is distinct from every other entry -- a duplicated bitmap is the way a copy-paste
// error in a table like this survives review.
const Glyph kGlyphs[] = {
    {' ', ROWS(0b000, 0b000, 0b000, 0b000, 0b000)},
    {'A', ROWS(0b010, 0b101, 0b111, 0b101, 0b101)},
    {'B', ROWS(0b110, 0b101, 0b110, 0b101, 0b110)},
    {'C', ROWS(0b011, 0b100, 0b100, 0b100, 0b011)},
    {'D', ROWS(0b110, 0b101, 0b101, 0b101, 0b110)},
    {'E', ROWS(0b111, 0b100, 0b110, 0b100, 0b111)},
    {'F', ROWS(0b111, 0b100, 0b110, 0b100, 0b100)},
    {'G', ROWS(0b011, 0b100, 0b101, 0b101, 0b011)},
    {'H', ROWS(0b101, 0b101, 0b111, 0b101, 0b101)},
    {'I', ROWS(0b111, 0b010, 0b010, 0b010, 0b111)},
    {'J', ROWS(0b001, 0b001, 0b001, 0b101, 0b010)},
    {'K', ROWS(0b101, 0b101, 0b110, 0b101, 0b101)},
    {'L', ROWS(0b100, 0b100, 0b100, 0b100, 0b111)},
    // M and N are the pair a 3-wide box makes hardest: three columns cannot hold the middle
    // vertex that tells an M from an N, so this is the least-bad of four candidates rendered and
    // compared at panel scale (docs/M4-C-FINDINGS.md). A solid cap over two legs is distinct from
    // N (one leg, one shoulder) and from H (a bar in the middle row), which is what matters.
    // The first cut, 101/111/111/101/101, read as an N in situ.
    {'M', ROWS(0b111, 0b111, 0b101, 0b101, 0b101)},
    {'N', ROWS(0b110, 0b101, 0b101, 0b101, 0b101)},
    {'O', ROWS(0b010, 0b101, 0b101, 0b101, 0b010)},
    {'P', ROWS(0b110, 0b101, 0b110, 0b100, 0b100)},
    {'Q', ROWS(0b010, 0b101, 0b101, 0b111, 0b011)},
    {'R', ROWS(0b110, 0b101, 0b110, 0b101, 0b101)},
    {'S', ROWS(0b011, 0b100, 0b010, 0b001, 0b110)},
    {'T', ROWS(0b111, 0b010, 0b010, 0b010, 0b010)},
    {'U', ROWS(0b101, 0b101, 0b101, 0b101, 0b111)},
    {'V', ROWS(0b101, 0b101, 0b101, 0b101, 0b010)},
    {'W', ROWS(0b101, 0b101, 0b111, 0b111, 0b101)},
    {'X', ROWS(0b101, 0b101, 0b010, 0b101, 0b101)},
    {'Y', ROWS(0b101, 0b101, 0b010, 0b010, 0b010)},
    {'Z', ROWS(0b111, 0b001, 0b010, 0b100, 0b111)},
    {'0', ROWS(0b111, 0b101, 0b101, 0b101, 0b111)},
    {'1', ROWS(0b010, 0b110, 0b010, 0b010, 0b111)},
    {'2', ROWS(0b110, 0b001, 0b010, 0b100, 0b111)},
    {'3', ROWS(0b110, 0b001, 0b010, 0b001, 0b110)},
    {'4', ROWS(0b101, 0b101, 0b111, 0b001, 0b001)},
    {'5', ROWS(0b111, 0b100, 0b110, 0b001, 0b110)},
    {'6', ROWS(0b011, 0b100, 0b111, 0b101, 0b111)},
    {'7', ROWS(0b111, 0b001, 0b010, 0b010, 0b010)},
    {'8', ROWS(0b111, 0b101, 0b111, 0b101, 0b111)},
    {'9', ROWS(0b111, 0b101, 0b111, 0b001, 0b110)},
    {'-', ROWS(0b000, 0b000, 0b111, 0b000, 0b000)},
    {'.', ROWS(0b000, 0b000, 0b000, 0b000, 0b010)},
    {':', ROWS(0b000, 0b010, 0b000, 0b010, 0b000)},
    {'/', ROWS(0b001, 0b001, 0b010, 0b100, 0b100)},
    {'%', ROWS(0b101, 0b001, 0b010, 0b100, 0b101)},
    {'+', ROWS(0b000, 0b010, 0b111, 0b010, 0b000)},
    {'!', ROWS(0b010, 0b010, 0b010, 0b000, 0b010)},
    {'?', ROWS(0b110, 0b001, 0b010, 0b000, 0b010)},
};

#undef ROWS

constexpr int kCount = (int)(sizeof(kGlyphs) / sizeof(kGlyphs[0]));
// Every row lit: what an unknown character draws, so a gap in the table is loud on the panel.
constexpr uint16_t kMissing = 0x7FFF;

// Linear scan. Forty-five entries looked up a few times per frame at most -- a sorted binary
// search would be the same code plus an ordering invariant nobody can see is broken.
const Glyph* find(char c) {
  if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
  for (int i = 0; i < kCount; ++i)
    if (kGlyphs[i].c == c) return &kGlyphs[i];
  return nullptr;
}

}  // namespace

uint8_t glyphRow(char c, int gy) {
  if (gy < 0 || gy >= kGlyphH) return 0;
  const Glyph* g = find(c);
  const uint16_t rows = g ? g->rows : kMissing;
  return (uint8_t)((rows >> (gy * kGlyphW)) & 0x7u);
}

bool glyphKnown(char c) { return find(c) != nullptr; }

int glyphCount() { return kCount; }
char glyphCharAt(int i) { return (i >= 0 && i < kCount) ? kGlyphs[i].c : '\0'; }

}  // namespace partsim
