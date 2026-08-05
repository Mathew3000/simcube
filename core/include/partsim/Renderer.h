#pragma once
#include "partsim/FieldGrid.h"
#include "partsim/Palette.h"
#include "partsim/Particles.h"
#include "partsim/Geometry.h"

namespace partsim {

// Depth-weighted particle splatting.
//
// Each particle adds glow to every panel it is near, attenuated by its distance from that
// panel: particles pressed against the glass are bright, deep ones are dim. Cost scales with
// particle count, not pixel count, which is the opposite of ray marching and the reason this
// fits an ESP32 budget.
//
// Two properties are load-bearing:
//
//  * The falloff has COMPACT SUPPORT -- exactly zero at kSplatInfluence. A 1/(1+d^2) falloff
//    never reaches zero, so every particle would touch every panel and the cost would become
//    O(N * panels * texels) instead of O(N * footprint).
//
//  * Accumulation is per-MATERIAL intensity, not RGB. Colour is applied once in resolve(),
//    through the palette ramps. That is what makes palettes pure data.
class Renderer {
 public:
  void init(const Geometry& g);
  void setPalette(const Palette* p) { palette_ = p; paletteB_ = nullptr; blend_ = 0.0f; }
  const Palette& palette() const { return *palette_; }

  // Crossfade between two palettes. t = 0 is all `a`, t = 1 all `b`. Costs a second ramp lookup
  // per lit texel, and only while a fade is actually running.
  void setPaletteBlend(const Palette* a, const Palette* b, float t) {
    palette_ = a;
    paletteB_ = (b == a) ? nullptr : b;
    blend_ = pclamp(t, 0.0f, 1.0f);
  }

  // Brightness scale: accumulated intensity that maps to the top of the ramp. Lower makes
  // the fluid glow harder.
  void setExposure(float fullScale) { fullScale_ = pmax(1.0f, fullScale); }

  // Advance each particle along its own velocity by this many seconds when splatting, without
  // touching the simulation. Lets the display run faster than the physics: whatever time the
  // fixed-step accumulator has not consumed yet is covered visually instead of being shown as a
  // stutter. Zero means splat exactly where the particles are.
  void setTimeOffset(float seconds) { timeOffset_ = seconds; }
  float timeOffset() const { return timeOffset_; }

  // clear -> splat particles -> splat heat -> resolve every panel into the RGBA buffers.
  void render(const Particles& p, const FieldGrid& f, const Geometry& g);

  void clear();
  void splat(const Particles& p, const Geometry& g);
  // Heat cells splat through the SAME path as particles, into the heat channel, so flame and
  // fluid composite consistently and the palette stays the only place colour is decided.
  void splatField(const FieldGrid& f, const Geometry& g);
  // bytesPerTexel: 3 for tight RGB (LED panels), 4 for RGBA (WebGL wants alignment 4).
  void resolve(int panel, uint8_t* out, int bytesPerTexel) const;

  // Stable-address RGBA8 buffer for panel i, filled by render(). The WASM layer hands these
  // addresses to JS once and never again, so the browser uploads straight out of the heap.
  const uint8_t* panelPixels(int i) const { return pixels_[i]; }
  int panelCount() const { return panels_; }

  // Raw accumulated intensity, for tests.
  uint16_t accumAt(int panel, int i, int j, int w, int channel) const {
    return accum_[panel][(j * w + i) * kChannelCount + channel];
  }

 private:
  static constexpr int kAttenSize = 64;
  static constexpr int kKernelSize = 64;

  const Palette* palette_ = nullptr;
  const Palette* paletteB_ = nullptr;  // crossfade target, null when not fading
  float blend_ = 0.0f;
  int panels_ = 0;
  int texels_[kMaxPanels] = {0};
  float fullScale_ = 900.0f;
  float timeOffset_ = 0.0f;

  // Radial splat kernel indexed by squared texel distance, and depth attenuation indexed by
  // distance from the panel. Both are LUTs so the inner loop has no divides, no sqrt and no
  // transcendentals -- which is what keeps this affordable at 240MHz.
  uint8_t atten_[kAttenSize + 1];
  uint8_t heatAtten_[kAttenSize + 1];  // separate, far longer reach; see kHeatInfluence
  uint8_t kernel_[kKernelSize + 1];
  float attenScale_ = 1.0f;
  float heatAttenScale_ = 1.0f;
  float kernelScale_ = 1.0f;

  uint16_t accum_[kMaxPanels][kMaxPanelTexels * kChannelCount];
  uint8_t pixels_[kMaxPanels][kMaxPanelTexels * 4];
};

}  // namespace partsim
