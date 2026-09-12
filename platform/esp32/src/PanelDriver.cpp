#include "PanelDriver.h"

#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

#include "PanelFramebuffer.h"
#include "Pins.h"

using namespace partsim;

bool PanelDriver::begin(const Geometry& g, uint8_t depthBits, uint8_t brightness) {
  int all[kMaxPanels];
  const int n = g.count();
  for (int k = 0; k < n && k < kMaxPanels; ++k) all[k] = k;
  return begin(g, all, n, depthBits, brightness);
}

bool PanelDriver::begin(const Geometry& g, const int* panels, int count, uint8_t depthBits,
                        uint8_t brightness) {
  faces_ = count;
  if (faces_ <= 0 || faces_ > kMaxPanels) return false;
  const int panelW = (int)g.at(panels[0]).w;
  const int panelH = (int)g.at(panels[0]).h;
  if (panelW * panelH * 3 > (int)sizeof(staging_)) return false;

  // Straight-through mounts to start with. Nothing else is knowable before the object exists;
  // the `mount` console command is how the real arrangement gets recorded.
  ChainMap::defaultMounts(faces_, mounts_);
  if (!chain_.init(g, mounts_, panels, faces_)) return false;

  HUB75_I2S_CFG::i2s_pins gpio = {pins::kR1, pins::kG1, pins::kB1, pins::kR2, pins::kG2,
                                  pins::kB2, pins::kA,  pins::kB,  pins::kC,  pins::kD,
                                  pins::kE,  pins::kLat, pins::kOe, pins::kClk};

  HUB75_I2S_CFG cfg((uint16_t)panelW, (uint16_t)panelH, (uint16_t)faces_, gpio);
  // double_buff is not optional here: without it the blit writes into the buffer being scanned
  // out, and a 3ms blit against a 141Hz refresh means roughly half of every frame is torn.
  cfg.double_buff = true;
  // The refresh field in the driver is uint8_t-backed in places; asking for more than 255 wraps
  // to something very slow. 120 is a request, and the achieved rate at 6 bits works out ~141Hz.
  cfg.min_refresh_rate = 120;
  cfg.i2sspeed = HUB75_I2S_CFG::HZ_16M;
  // Blank a couple of clocks around the latch. Below 2 the panels ghost: the previous row is
  // still lit while the shift register is being loaded for the next one.
  cfg.latch_blanking = 2;
  cfg.clkphase = false;
  cfg.setPixelColorDepthBits(depthBits);

  dma_ = new MatrixPanel_I2S_DMA(cfg);
  if (!dma_ || !dma_->begin()) {
    dma_ = nullptr;
    return false;
  }
  dma_->setBrightness8(brightness);

  depth_ = cfg.getPixelColorDepthBits();
  rowsPerFrame_ = panelH / MATRIX_ROWS_IN_PARALLEL;
  fastBlit_ = verifyFastBlit();

  // After verifyFastBlit(), which deliberately dirties a few texels of the back buffer.
  dma_->clearScreen();
  return true;
}

#if PARTSIM_FAST_BLIT
namespace {

// The three RGB bits texel `packed` contributes to bitplane `plane`.
//
// `packed` interleaves the compensated channels three bits apart and is ALREADY shifted into the
// half of the DMA word this row owns (see PanelDriver::spread_ and blitRowFast), so `mask` is
// 0b111 in that same position and this is one shift and one mask -- not the six mask-and-test
// rounds the library does per texel.
inline uint16_t planeBits(uint32_t packed, int plane, uint32_t mask) {
  return (uint16_t)((packed >> (3 * plane)) & mask);
}

}  // namespace
#endif

#if PARTSIM_FAST_BLIT
// The raw DMA words a run occupies, all bitplanes, for verifyFastBlit to compare. Whole words,
// including the half of each that belongs to the other scan row -- that half must survive a blit.
void PanelDriver::snapRow(void* fbv, const ChainRun& run, int w, uint16_t* out) const {
  frameStruct* fb = (frameStruct*)fbv;
  const int cy = run.cy < rowsPerFrame_ ? run.cy : run.cy - rowsPerFrame_;
  const rowBitStruct* rb = fb->rowBits[cy].get();
  for (int p = 0; p < depth_; ++p)
    for (int i = 0; i < w; ++i)
      out[p * w + i] = rb->data[(size_t)p * rb->width + (size_t)(run.cx + i * run.dx)];
}
#endif

// spread_[v]: the compensated value for input byte v, with bit p moved to bit 3p, so three table
// loads and two shifts give a texel's whole six-plane contribution.
//
// lumConvTab is the HUB75 library's own CIE table, selected by PIXEL_COLOR_DEPTH_BITS at COMPILE
// time -- which is not the runtime depth passed to setPixelColorDepthBits(). The library takes the
// low `depth_` bits of whatever that table holds, so this does too. Reproducing the library
// exactly is the requirement here, including where the library is wrong: see docs/W5-FINDINGS.md
// for what the mismatch costs and why fixing it is a separate change.
void PanelDriver::buildSpread() {
#if PARTSIM_FAST_BLIT
  const uint32_t mask = (1u << depth_) - 1u;
  for (int v = 0; v < 256; ++v) {
    const uint32_t c = (uint32_t)lumConvTab[v] & mask;
    uint32_t s = 0;
    for (int p = 0; p < depth_; ++p)
      if (c & (1u << p)) s |= 1u << (3 * p);
    spread_[v] = s;
  }
#endif
}

// Proves the row-walking path writes exactly the DMA words drawPixelRGB888 would have.
//
// The house rule applies to correctness as much as to timing: PanelFramebuffer.h reaches a private
// member, and "the layout is what I think it is" is an assertion. So this blits a real row through
// the real shipping code both ways and compares the raw words -- which covers the brightness
// table, the bitplane packing, the two-halves bit offset, the chain addressing and the run
// direction in one test, on the actual buffer the library allocated.
//
// Runs once, in begin(), into the back buffer that nothing is scanning out yet.
bool PanelDriver::verifyFastBlit() {
#if !PARTSIM_FAST_BLIT
  return false;
#else
  if (!dma_ || depth_ < 1 || depth_ > 8 || rowsPerFrame_ < 1) return false;

  buildSpread();

  frameStruct* fb = panelfb::backBuffer(dma_);
  if (fb == nullptr) return false;
  if ((int)fb->rowBits.size() != rowsPerFrame_) return false;

  const int chainW = chain_.chainWidth();
  for (int y = 0; y < rowsPerFrame_; ++y) {
    const rowBitStruct* rb = fb->rowBits[y].get();
    if (rb == nullptr || rb->data == nullptr) return false;
    if ((int)rb->width < chainW) return false;
    if (rb->colour_depth < depth_) return false;
  }

  // Face 0 at two rows: one in each half of the scan, so both the RGB1 and the RGB2 bit offset
  // are exercised, and with them the requirement that a write to one half leaves the other alone.
  const int w = chain_.chainWidth() / chain_.count();
  if (w < 2 || w > kPanelRes) return false;
  const int h = chain_.chainHeight();

  uint16_t ref[8 * kPanelRes];
  for (int pass = 0; pass < 2; ++pass) {
    const int j = pass == 0 ? 0 : h - 1;
    const ChainRun run = chain_.row(0, j);
    if (run.dy != 0) continue;  // this mount has no span to walk; nothing to verify

    // A pattern that hits both ends of the ramp and every low bit in between, so a dropped
    // bitplane or a swapped channel cannot pass by coincidence.
    for (int i = 0; i < w; ++i) {
      staging_[i * 3 + 0] = (uint8_t)(i * 7 + 1);
      staging_[i * 3 + 1] = (uint8_t)(255 - i * 5);
      staging_[i * 3 + 2] = (uint8_t)(i * 3 + 128);
    }

    // The other half of each shared word, set to something non-zero first: if the fast path used
    // the wrong bit offset or the wrong clear mask, it would wipe this and the comparison after
    // the second blit would still pass. Writing it through the library makes it a real reference.
    const int other = run.cy < rowsPerFrame_ ? run.cy + rowsPerFrame_ : run.cy - rowsPerFrame_;
    for (int i = 0; i < w; ++i)
      dma_->drawPixelRGB888((int16_t)(run.cx + i * run.dx), (int16_t)other, 200, 120, 60);

    blitRowSlow(run, staging_, w);
    snapRow(fb, run, w, ref);

    // Dirty the row so an unwritten word cannot be mistaken for a matching one.
    for (int i = 0; i < w * 3; ++i) staging_[i] = (uint8_t)(255 - staging_[i]);
    blitRowSlow(run, staging_, w);
    for (int i = 0; i < w * 3; ++i) staging_[i] = (uint8_t)(255 - staging_[i]);

    blitRowFast(fb, run, staging_, w);

    uint16_t mine[8 * kPanelRes];
    snapRow(fb, run, w, mine);
    for (int i = 0; i < depth_ * w; ++i)
      if (mine[i] != ref[i]) return false;
  }
  return true;
#endif
}

void PanelDriver::setBrightness(uint8_t b) {
  if (dma_) dma_->setBrightness8(b);
}

void PanelDriver::clear() {
  if (dma_) dma_->clearScreen();
}

bool PanelDriver::allRunsHorizontal(const Geometry&) const {
  for (int f = 0; f < chain_.count(); ++f) {
    if (chain_.row(f, 0).dy != 0) return false;
  }
  return true;
}

void PanelDriver::blitRowSlow(const ChainRun& run, const uint8_t* src, int w) {
  // One drawPixelRGB888 per texel. Correct for any mount and the only path when a quarter-turn
  // maps the renderer row onto a chain COLUMN, where the six bitplane words are `width` apart
  // instead of adjacent and there is no span to walk.
  int cx = run.cx, cy = run.cy;
  for (int i = 0; i < w; ++i) {
    dma_->drawPixelRGB888((int16_t)cx, (int16_t)cy, src[0], src[1], src[2]);
    src += 3;
    cx += run.dx;
    cy += run.dy;
  }
}

void PanelDriver::blitRowFast(void* fbv, const ChainRun& run, const uint8_t* src, int w) {
#if PARTSIM_FAST_BLIT
  frameStruct* fb = (frameStruct*)fbv;

  // Which half of the scan this row belongs to, and therefore which three bits of the shared DMA
  // word it owns. The other half's bits must survive: one word drives two rows at once.
  int cy = run.cy;
  uint16_t clear = BITMASK_RGB1_CLEAR;
  int coff = 0;
  if (cy >= rowsPerFrame_) {
    cy -= rowsPerFrame_;
    clear = BITMASK_RGB2_CLEAR;
    coff = BITS_RGB2_OFFSET;
  }
  rowBitStruct* rb = fb->rowBits[cy].get();
  uint16_t* const base = rb->data;
  const size_t stride = rb->width;

  // Compensate and interleave the row once, then read it back once per bitplane. Doing it the
  // other way round -- texel outer, plane inner -- is what forces the DMA row pointer to be
  // recomputed per texel, and that recomputation is the cost being removed here.
  //
  // The half-of-panel shift rides along in this pass rather than happening once per texel per
  // PLANE below: six times fewer shifts, for free, because this pass touches every texel anyway.
  // Measured 6.75 -> 6.49 ms.
  const uint32_t mask = 7u << coff;
  for (int i = 0; i < w; ++i)
    packed_[i] = (spread_[src[i * 3 + 0]] | (spread_[src[i * 3 + 1]] << 1) |
                  (spread_[src[i * 3 + 2]] << 2))
                 << coff;

  for (int p = 0; p < depth_; ++p) {
    uint16_t* q = base + (size_t)p * stride + (size_t)run.cx;
    // dx is +1 or -1: a mirrored mount runs the renderer row backwards along the chain. Two loops
    // rather than a variable step so the common direction indexes forward.
    if (run.dx > 0) {
      for (int i = 0; i < w; ++i)
        q[i] = (uint16_t)((q[i] & clear) | planeBits(packed_[i], p, mask));
    } else {
      for (int i = 0; i < w; ++i)
        q[-i] = (uint16_t)((q[-i] & clear) | planeBits(packed_[i], p, mask));
    }
  }
#else
  (void)fbv;
  (void)run;
  (void)src;
  (void)w;
#endif
}

void PanelDriver::blitFace(int face, int w, int h) {
  // Row by row rather than texel by texel in map(): row() hoists the mount arithmetic out of the
  // inner loop. What ChainRun was actually built for is blitRowFast above -- a contiguous span of
  // renderer texels landing on a contiguous span of chain pixels, so the DMA row pointer is
  // fetched once per bitplane per ROW rather than once per bitplane per texel, which is what
  // drawPixelRGB888 does and what made the blit 7.50 ms of an 11.10 ms frame.
#if PARTSIM_FAST_BLIT
  void* fb = (fastBlit_ && w <= kPanelRes) ? (void*)panelfb::backBuffer(dma_) : nullptr;
#else
  void* fb = nullptr;
#endif
  for (int j = 0; j < h; ++j) {
    const ChainRun run = chain_.row(face, j);
    const uint8_t* src = staging_ + (size_t)j * (size_t)w * 3u;
    // A quarter-turn maps this renderer row onto a chain COLUMN, where the run's texels are a
    // whole row apart in the DMA buffer and there is no span to walk. Per-texel is what that is.
    if (fb != nullptr && run.dy == 0)
      blitRowFast(fb, run, src, w);
    else
      blitRowSlow(run, src, w);
  }
}

void PanelDriver::present(const Renderer& r, const Geometry& g) {
  if (!dma_) return;
  for (int f = 0; f < chain_.count(); ++f) {
    const int panel = chain_.panelAt(f);
    const Panel& pan = g.at(panel);
    // Tight RGB, not RGBA: the alpha byte would be a third of the staging buffer and the panel
    // has nothing to do with it. Renderer::resolve serves both widths for exactly this reason.
    //
    // resolve() takes a PANEL index while blitFace takes a DRIVEN-FACE index. They coincide on a
    // single-node cube and do not on a display node, which is why panelAt() exists.
    r.resolve(panel, staging_, 3);
    blitFace(f, (int)pan.w, (int)pan.h);
  }
  dma_->flipDMABuffer();
}

void PanelDriver::testPattern(const Geometry& g) {
  if (!dma_) return;

  // One hue per face, dim enough that the markers stand out against it.
  static const uint8_t kFaceRgb[6][3] = {
      {40, 0, 0}, {0, 40, 0}, {0, 0, 40}, {40, 40, 0}, {40, 0, 40}, {0, 40, 40},
  };

  for (int f = 0; f < chain_.count(); ++f) {
    const Panel& pan = g.at(chain_.panelAt(f));
    const int w = (int)pan.w, h = (int)pan.h;
    // Hue by the GEOMETRY panel, not the driven index, so face colours mean the same thing on
    // every node -- otherwise each display node would start its palette again from red.
    const uint8_t* base = kFaceRgb[chain_.panelAt(f) % 6];

    for (int j = 0; j < h; ++j) {
      uint8_t* dst = staging_ + (size_t)j * (size_t)w * 3u;
      for (int i = 0; i < w; ++i) {
        uint8_t rr = base[0], gg = base[1], bb = base[2];
        // The origin marker sits at texel (1,1), one in from the corner so it is unambiguous
        // which corner it is even if the outermost row is hidden by a bezel.
        if (i == 1 && j == 1) { rr = 255; gg = 255; bb = 255; }
        // A short arm along +x and a longer one along +y: two different lengths, so a 90-degree
        // rotation is distinguishable from a mirror at a glance.
        else if (j == 1 && i >= 2 && i <= 4) { rr = 255; gg = 0; bb = 0; }
        else if (i == 1 && j >= 2 && j <= 6) { rr = 0; gg = 255; bb = 0; }
        dst[0] = rr; dst[1] = gg; dst[2] = bb;
      }
    }
    blitFace(f, w, h);
  }
  dma_->flipDMABuffer();
}
