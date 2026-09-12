#include "partsim/BeakerOverlay.h"

namespace partsim {
namespace {

// What a fully-lit overlay texel adds to the weight channel.
//
// kSplatExposure is the accumulated intensity resolve() maps to the top of a ramp, and both
// shipping palettes put near-white there (235,250,255 and 230,255,255) -- so a line written at
// full scale resolves white without the overlay knowing anything about palettes. Written as the
// exposure rather than as a literal so it tracks the blob width like everything else does.
const int kOverlayFull = (int)kSplatExposure;

// A panel's normal is along the up axis to within this much before it counts as a cap rather
// than a side. Exact for a cube; the slack is for a geometry whose faces are not axis-aligned.
constexpr float kCapDot = 0.7071f;  // 45 degrees

int panelHeight(const Renderer& r, int panel) {
  const int w = r.panelWidth(panel);
  return (w > 0) ? r.panelTexels(panel) / w : 0;
}

}  // namespace

void BeakerOverlay::init(const Geometry& g, Vec3 objectUp) {
  const Vec3 up = normalize(objectUp);
  count_ = g.count();
  for (int k = 0; k < count_; ++k) {
    const Panel& p = g.at(k);
    const float dn = dot(up, p.n);
    // The INWARD normal of the top face points down, so a strongly negative dot is the top.
    kind_[k] = (dn <= -kCapDot) ? kTop : ((dn >= kCapDot) ? kBottom : kSide);

    // Up expressed in panel axes. u and v are both `pitch` long, so these are proportional to
    // the direction cosines and picking the larger is picking the dominant axis.
    const float cu = dot(up, p.u);
    const float cv = dot(up, p.v);
    int8_t ui = 0, uj = 0;
    if (kind_[k] == kSide) {
      if (pabs(cu) >= pabs(cv)) {
        ui = (int8_t)(cu >= 0.0f ? 1 : -1);
      } else {
        uj = (int8_t)(cv >= 0.0f ? 1 : -1);
      }
    }
    upI_[k] = ui;
    upJ_[k] = uj;
  }
  for (int k = count_; k < kMaxPanels; ++k) {
    kind_[k] = kSide;
    upI_[k] = 0;
    upJ_[k] = 0;
  }
}

void BeakerOverlay::plot(Renderer& r, int panel, int i, int j, OverlayRgb c) const {
  // The weight channel carries the line's brightness whatever the build, so an overlay texel is
  // always at least lit.
  r.addAccum(panel, i, j, kChWater, kOverlayFull);
#if PARTSIM_ENABLE_CHROMA
  // With chroma the three dye channels carry the hue and resolve() recovers colour as the ratio
  // between them, so a requested RGB survives exactly.
  //
  // SET, not add: the dye already in the texel is the fluid's, and adding to it made a red glyph
  // resolve magenta over a blue beaker and red over an empty one -- the glyph changing colour with
  // the fill level, which is the one thing a "hold me this way up" instruction must not do.
  // Measured on a render before it was changed; see docs/M4-C-FINDINGS.md.
  r.setAccum(panel, i, j, kChCR, kOverlayFull * (int)c.r / 255);
  r.setAccum(panel, i, j, kChCG, kOverlayFull * (int)c.g / 255);
  r.setAccum(panel, i, j, kChCB, kOverlayFull * (int)c.b / 255);
#else
  // Without it there is one channel and one ramp, so every overlay texel resolves to the top of
  // the water ramp. White lines are right; red arrows come out white. See docs/M4-C-FINDINGS.md
  // -- this is a missing channel, not a missing branch, and adding a branch here would not fix
  // it.
  (void)c;
#endif
}

void BeakerOverlay::drawEdges(Renderer& r) const {
  for (int panel = 0; panel < count_; ++panel) {
    if (!r.rendersPanel(panel)) continue;
    if (kind_[panel] == kTop) continue;  // the open face draws none of its own edges

    const int w = r.panelWidth(panel);
    const int h = panelHeight(r, panel);
    if (w <= 0 || h <= 0) continue;

    // Which border, if any, is the rim shared with the top face. -1 on the bottom cap, which
    // draws all four.
    const int skipI = (kind_[panel] == kSide && upI_[panel] > 0)   ? w - 1
                      : (kind_[panel] == kSide && upI_[panel] < 0) ? 0
                                                                   : -1;
    const int skipJ = (kind_[panel] == kSide && upJ_[panel] > 0)   ? h - 1
                      : (kind_[panel] == kSide && upJ_[panel] < 0) ? 0
                                                                   : -1;

    for (int j = 0; j < h; ++j) {
      if (skipI != 0) plot(r, panel, 0, j, kOverlayWhite);
      if (skipI != w - 1) plot(r, panel, w - 1, j, kOverlayWhite);
    }
    for (int i = 0; i < w; ++i) {
      if (skipJ != 0) plot(r, panel, i, 0, kOverlayWhite);
      if (skipJ != h - 1) plot(r, panel, i, h - 1, kOverlayWhite);
    }
  }
}

int BeakerOverlay::textWidth(const char* text, int scale) {
  int n = 0;
  for (const char* c = text; *c; ++c) ++n;
  if (n == 0) return 0;
  return n * kGlyphW * scale + (n - 1) * kGlyphGap * scale;
}

void BeakerOverlay::drawText(Renderer& r, int panel, const char* text, OverlayRgb colour,
                             int scale) const {
  if (!r.rendersPanel(panel)) return;
  const int w = r.panelWidth(panel);
  const int h = panelHeight(r, panel);
  if (w <= 0 || h <= 0) return;
  if (scale <= 0) scale = scaleFor(w);

  // Keep clear of the edge lines: one texel of border plus one of air, scaled.
  const int margin = 2 * scale;
  const int avail = w - 2 * margin;
  const int advance = (kGlyphW + kGlyphGap) * scale;   // pen step per character
  const int lineAdvance = (kGlyphH + kGlyphGap) * scale;

  int n = 0;
  for (const char* c = text; *c; ++c) ++n;
  if (n == 0) return;

  // Characters per line. Width of m glyphs is m*advance - gap*scale, so the largest m fitting
  // `avail` is (avail + gap*scale) / advance. At least one, so a panel too narrow for even one
  // glyph still draws something rather than looping forever.
  int perLine = (avail + kGlyphGap * scale) / advance;
  perLine = iclamp(perLine, 1, n);
  const int lines = (n + perLine - 1) / perLine;

  const int blockH = lines * kGlyphH * scale + (lines - 1) * kGlyphGap * scale;
  // Panel row 0 is the BOTTOM row, so the first line of text is at the HIGHEST j.
  int topJ = (h + blockH) / 2;

  for (int ln = 0; ln < lines; ++ln) {
    const int first = ln * perLine;
    const int last = imin(n, first + perLine);
    const int count = last - first;
    const int lineW = count * kGlyphW * scale + (count - 1) * kGlyphGap * scale;
    const int x0 = (w - lineW) / 2;
    const int y0 = topJ - ln * lineAdvance;  // j of the row just above the line's top row

    for (int k = 0; k < count; ++k) {
      const char ch = text[first + k];
      for (int gy = 0; gy < kGlyphH; ++gy) {
        const uint8_t row = glyphRow(ch, gy);
        for (int gx = 0; gx < kGlyphW; ++gx) {
          if ((row & (1u << (kGlyphW - 1 - gx))) == 0) continue;
          const int px = x0 + (k * (kGlyphW + kGlyphGap) + gx) * scale;
          const int py = y0 - (gy + 1) * scale;
          for (int sy = 0; sy < scale; ++sy)
            for (int sx = 0; sx < scale; ++sx) plot(r, panel, px + sx, py + sy, colour);
        }
      }
    }
  }
}

void BeakerOverlay::drawArrow(Renderer& r, int panel, OverlayRgb c) const {
  const int w = r.panelWidth(panel);
  const int h = panelHeight(r, panel);
  if (w <= 0 || h <= 0) return;

  const int ui = upI_[panel], uj = upJ_[panel];
  // Arrow space: `a` across, `b` along object-up. Mapping it here rather than assuming +v is up
  // means a face whose `up` runs along its i axis gets a correct arrow instead of a sideways one.
  const int cx = w / 2, cy = h / 2;
  auto put = [&](int a, int b) {
    int i, j;
    if (uj > 0)      { i = cx + a; j = cy + b; }
    else if (uj < 0) { i = cx - a; j = cy - b; }
    else if (ui > 0) { i = cx + b; j = cy - a; }
    else             { i = cx - b; j = cy + a; }
    plot(r, panel, i, j, c);
  };

  const int t = imax(1, w / 32);        // stroke thickness
  const int reach = imax(4, h / 4);     // half-length, so the arrow is 2*reach tall
  const int head = imax(2, reach / 2);  // barb length

  for (int b = -reach; b <= reach; ++b)
    for (int a = 0; a < t; ++a) put(a, b);
  for (int k = 0; k <= head; ++k)
    for (int a = 0; a < t; ++a) {
      put(-k, reach - k - a);
      put(k + t - 1, reach - k - a);
    }
}

void BeakerOverlay::drawGateGlyphs(Renderer& r) const {
  for (int panel = 0; panel < count_; ++panel) {
    if (!r.rendersPanel(panel)) continue;
    switch (kind_[panel]) {
      case kTop: drawText(r, panel, "TOP", kOverlayRed); break;
      case kBottom: drawText(r, panel, "BOTTOM", kOverlayRed); break;
      case kSide: drawArrow(r, panel, kOverlayRed); break;
    }
  }
}

}  // namespace partsim
