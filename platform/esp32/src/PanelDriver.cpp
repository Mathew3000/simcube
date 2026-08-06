#include "PanelDriver.h"

#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

#include "Pins.h"

using namespace partsim;

bool PanelDriver::begin(const Geometry& g, uint8_t depthBits, uint8_t brightness) {
  faces_ = g.count();
  if (faces_ <= 0) return false;
  const int panelW = (int)g.at(0).w;
  const int panelH = (int)g.at(0).h;
  if (panelW * panelH * 3 > (int)sizeof(staging_)) return false;

  // Straight-through mounts to start with. Nothing else is knowable before the object exists;
  // the `mount` console command is how the real arrangement gets recorded.
  ChainMap::defaultMounts(faces_, mounts_);
  if (!chain_.init(g, mounts_, faces_)) return false;

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
  dma_->clearScreen();
  return true;
}

void PanelDriver::setBrightness(uint8_t b) {
  if (dma_) dma_->setBrightness8(b);
}

void PanelDriver::clear() {
  if (dma_) dma_->clearScreen();
}

bool PanelDriver::allRunsHorizontal(const Geometry& g) const {
  for (int p = 0; p < faces_ && p < g.count(); ++p) {
    if (chain_.row(p, 0).dy != 0) return false;
  }
  return true;
}

void PanelDriver::blitFace(int panel, int w, int h) {
  // Row by row rather than texel by texel in map(): row() hoists the mount arithmetic out of
  // the inner loop, so the per-pixel cost is one add and the driver call.
  //
  // That driver call is the expensive part -- drawPixelRGB888 is a read-modify-write per
  // bitplane, on the order of 130 cycles. Six 32x32 faces is 6144 of them, about 3ms at 240MHz,
  // or roughly a tenth of a frame at 30fps. Measurable but affordable, and the ChainRun
  // structure is here so a direct row write into the DMA buffer can replace it without
  // disturbing the mapping if that tenth is ever needed.
  for (int j = 0; j < h; ++j) {
    const ChainRun run = chain_.row(panel, j);
    const uint8_t* src = staging_ + (size_t)j * (size_t)w * 3u;
    int cx = run.cx, cy = run.cy;
    for (int i = 0; i < w; ++i) {
      dma_->drawPixelRGB888((int16_t)cx, (int16_t)cy, src[0], src[1], src[2]);
      src += 3;
      cx += run.dx;
      cy += run.dy;
    }
  }
}

void PanelDriver::present(const Renderer& r, const Geometry& g) {
  if (!dma_) return;
  const int n = (faces_ < g.count()) ? faces_ : g.count();
  for (int p = 0; p < n; ++p) {
    const Panel& pan = g.at(p);
    // Tight RGB, not RGBA: the alpha byte would be a third of the staging buffer and the panel
    // has nothing to do with it. Renderer::resolve serves both widths for exactly this reason.
    r.resolve(p, staging_, 3);
    blitFace(p, (int)pan.w, (int)pan.h);
  }
  dma_->flipDMABuffer();
}

void PanelDriver::testPattern(const Geometry& g) {
  if (!dma_) return;

  // One hue per face, dim enough that the markers stand out against it.
  static const uint8_t kFaceRgb[6][3] = {
      {40, 0, 0}, {0, 40, 0}, {0, 0, 40}, {40, 40, 0}, {40, 0, 40}, {0, 40, 40},
  };

  const int n = (faces_ < g.count()) ? faces_ : g.count();
  for (int p = 0; p < n; ++p) {
    const Panel& pan = g.at(p);
    const int w = (int)pan.w, h = (int)pan.h;
    const uint8_t* base = kFaceRgb[p % 6];

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
    blitFace(p, w, h);
  }
  dma_->flipDMABuffer();
}
