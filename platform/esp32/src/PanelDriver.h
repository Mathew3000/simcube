#pragma once
#include "partsim/app/Display.h"

class MatrixPanel_I2S_DMA;

// Drives the HUB75 chain from the renderer's per-panel output.
//
// The interesting part is what is NOT here: no framebuffer of its own beyond one face's worth of
// staging. Six panels of RGBA is 24KB, which at this budget is the difference between fitting
// internal SRAM and not, so faces are resolved and pushed one at a time.
// Implements partsim::app::Display -- the interface was lifted from this class rather than
// designed beside it, so a board that drives no panels can satisfy it with NullDisplay and a
// non-ESP solver board needs no HUB75 code at all.
class PanelDriver final : public partsim::app::Display {
 public:
  // Panel size comes from the geometry, never from constants here -- the same rule the browser
  // frontend follows, and for the same reason: the physics and the display must not be able to
  // disagree about how big a face is.
  //
  // Drives every panel in the geometry.
  bool begin(const partsim::Geometry& g, uint8_t depthBits, uint8_t brightness);

  // Drives only the listed panels. A display node in a multi-node cube holds the full panel
  // table for the physics but has DMA buffers for just its own faces.
  bool begin(const partsim::Geometry& g, const int* panels, int count, uint8_t depthBits,
             uint8_t brightness);

  // Resolves each face out of the renderer's accumulation buffers and pushes it to the chain,
  // then flips the DMA back buffer so a whole frame appears at once. Partial frames on an LED
  // panel read as tearing, which on a cube looks like the fluid breaking apart.
  void present(const partsim::Renderer& r, const partsim::Geometry& g) override;

  // A calibration pattern: each face a distinct hue, with a marker at renderer texel (1,1) and
  // arms of 3 along +x and 5 along +y. That is enough to read off each panel's rotation and
  // mirror by eye, type the correction into the `mount` console command and see it applied --
  // rather than reflashing once per guess.
  void testPattern(const partsim::Geometry& g) override;

  void setBrightness(uint8_t b) override;
  void clear();

  partsim::ChainMap& chain() override { return chain_; }
  const partsim::ChainMap& chain() const { return chain_; }
  bool ready() const override { return dma_ != nullptr; }
  // Whether every face maps to horizontal runs, i.e. whether the fast blit path is available
  // for all of them. Reported at boot because it is a property of the mount table, not the code.
  bool allRunsHorizontal(const partsim::Geometry& g) const override;

  // Whether the row-walking blit verified at boot and is in use. False either because the HUB75
  // library's buffer layout did not match what PanelFramebuffer.h expects, or because it was
  // compiled out. The picture is the same either way; the blit is just ~4x slower. Not on the
  // Display interface -- it is a fact about this driver, and the boot report holds the concrete
  // type.
  bool fastBlit() const { return fastBlit_; }

 private:
  void blitFace(int face, int w, int h);
  // The per-texel path. Correct for any mount, including the quarter-turns that map a renderer row
  // onto a chain COLUMN, where there is no contiguous run to walk.
  void blitRowSlow(const partsim::ChainRun& run, const uint8_t* src, int w);
  // The row-walking path, for a run that is horizontal in the chain. `fb` is the library's back
  // buffer; see PanelFramebuffer.h for how it is reached and why.
  void blitRowFast(void* fb, const partsim::ChainRun& run, const uint8_t* src, int w);
  // Builds spread_ from the HUB75 library's own brightness table.
  void buildSpread();
  // Reads back the raw DMA words a run occupies, all bitplanes, for verifyFastBlit to compare.
  void snapRow(void* fb, const partsim::ChainRun& run, int w, uint16_t* out) const;
  // Blits a test row both ways and compares the raw DMA words. See PanelFramebuffer.h.
  bool verifyFastBlit();

  MatrixPanel_I2S_DMA* dma_ = nullptr;
  partsim::ChainMap chain_;
  partsim::FaceMount mounts_[partsim::kMaxPanels];
  int faces_ = 0;
  bool fastBlit_ = false;
  uint8_t depth_ = 0;
  // Rows the panel scans in parallel halves: chain y below this drives R1G1B1, at or above it
  // drives R2G2B2 of the same DMA word.
  int rowsPerFrame_ = 0;
  // One face of RGB. Static, like everything else -- 3KB rather than 24KB for all six.
  uint8_t staging_[partsim::kMaxPanelTexels * 3];
  // Brightness compensation and bitplane interleave folded into one table: entry v holds the
  // library's compensated value for input v with bit p moved to bit 3p, so a texel's whole
  // six-plane contribution is three table loads and two shifts instead of six mask-and-test
  // rounds. 1KB, built once in begin().
  uint32_t spread_[256];
  // One row's texels, pre-spread, so the six bitplane passes read it instead of recomputing it.
  uint32_t packed_[partsim::kPanelRes];
};
