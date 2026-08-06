#pragma once
#include "partsim/ChainMap.h"
#include "partsim/Renderer.h"

class MatrixPanel_I2S_DMA;

// Drives the HUB75 chain from the renderer's per-panel output.
//
// The interesting part is what is NOT here: no framebuffer of its own beyond one face's worth of
// staging. Six panels of RGBA is 24KB, which at this budget is the difference between fitting
// internal SRAM and not, so faces are resolved and pushed one at a time.
class PanelDriver {
 public:
  // Panel count and size come from the geometry, never from constants here -- the same rule the
  // browser frontend follows, and for the same reason: the physics and the display must not be
  // able to disagree about how many faces there are or how big they are.
  bool begin(const partsim::Geometry& g, uint8_t depthBits, uint8_t brightness);

  // Resolves each face out of the renderer's accumulation buffers and pushes it to the chain,
  // then flips the DMA back buffer so a whole frame appears at once. Partial frames on an LED
  // panel read as tearing, which on a cube looks like the fluid breaking apart.
  void present(const partsim::Renderer& r, const partsim::Geometry& g);

  // A calibration pattern: each face a distinct hue, with a marker at renderer texel (1,1) and
  // arms of 3 along +x and 5 along +y. That is enough to read off each panel's rotation and
  // mirror by eye, type the correction into the `mount` console command and see it applied --
  // rather than reflashing once per guess.
  void testPattern(const partsim::Geometry& g);

  void setBrightness(uint8_t b);
  void clear();

  partsim::ChainMap& chain() { return chain_; }
  const partsim::ChainMap& chain() const { return chain_; }
  bool ready() const { return dma_ != nullptr; }
  // Whether every face maps to horizontal runs, i.e. whether the fast blit path is available
  // for all of them. Reported at boot because it is a property of the mount table, not the code.
  bool allRunsHorizontal(const partsim::Geometry& g) const;

 private:
  void blitFace(int panel, int w, int h);

  MatrixPanel_I2S_DMA* dma_ = nullptr;
  partsim::ChainMap chain_;
  partsim::FaceMount mounts_[partsim::kMaxPanels];
  int faces_ = 0;
  // One face of RGB. Static, like everything else -- 3KB rather than 24KB for all six.
  uint8_t staging_[partsim::kMaxPanelTexels * 3];
};
