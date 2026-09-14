#pragma once
#include <FastLED.h>

#include "partsim/app/Display.h"

// Drives six 8x8 WS2812B matrices, one data chain, from the renderer's per-panel output.
//
// The model is platform/esp32/src/PanelDriver.cpp's present() (11 lines): resolve one face at a
// time into a staging buffer, blit it, one show() at the end. There is no fast-blit path here --
// at 384 LEDs total the per-texel cost that HUB75's six-bitplane read-modify-write made worth
// optimising around does not exist.
class Ws2812Display final : public partsim::app::Display {
 public:
  // Panel size and face count come from the geometry, the same rule PanelDriver follows and for
  // the same reason: the physics and the display must not be able to disagree about how big a
  // face is.
  bool begin(const partsim::Geometry& g);

  void present(const partsim::Renderer& r, const partsim::Geometry& g) override;
  // The orientation calibration pattern (MINI.md M3): calibrationTexel() per texel, through the
  // same ChainMap every other frame goes through -- this is what M3 needs to actually LIVE, since
  // simStep keeps calling this every frame while orientation mode is on.
  void testPattern(const partsim::Geometry& g) override;

  void setBrightness(uint8_t b) override;
  bool ready() const override { return begun_; }
  partsim::ChainMap& chain() override { return chain_; }
  bool allRunsHorizontal(const partsim::Geometry& g) const override;

  // The walk (MINI.md 6.3). Bypasses chain_ entirely on purpose -- see Display.h.
  bool lightOne(int index) override;
  int lightCount() const override { return kNumLeds; }

 private:
  static constexpr int kMatrixSize = 8;  // one face
  static constexpr int kNumFaces = 6;
  static constexpr int kNumLeds = kMatrixSize * kMatrixSize * kNumFaces;  // 384

  // Chain pixel (cx, cy) -> raw strip index. One place, with the reasoning above it, per
  // MINI.md M2's third bullet.
  //
  // `slot = cx / kMatrixSize` is which physical matrix this chain column belongs to, in WIRING
  // order -- ChainMap::FaceMount.slot's own definition ("position in the daisy chain"), so face
  // order along the chain is already handled by the mount table / `m` console command and needs
  // no code here. What is new is that each matrix is fully traversed before the next begins
  // (`slot * 64 + local`), rather than the chain being one long 48x8 raster -- almost certainly
  // the real wiring for six separate matrices, per MINI.md M2.
  int chainPixelToStripIndex(int cx, int cy) const;
  void blitFace(int face, int w, int h, const uint8_t* rgb);

  partsim::ChainMap chain_;
  partsim::FaceMount mounts_[partsim::kMaxPanels];
  CRGB leds_[kNumLeds];
  // One face of RGB, like PanelDriver's staging_ -- 192B rather than 1.15KB for all six.
  uint8_t staging_[partsim::kMaxPanelTexels * 3];
  bool begun_ = false;
};
