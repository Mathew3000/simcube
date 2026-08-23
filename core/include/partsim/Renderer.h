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
  // Render every panel in the geometry. Fails if there are more panels than render slots.
  bool init(const Geometry& g);

  // Render only the listed panels, by panel index.
  //
  // This is the multi-node case: every node holds the FULL panel table, because
  // Geometry::bounds() derives the container from it and the container must be identical
  // everywhere -- but a node allocates accumulation buffers only for the faces it physically
  // drives. Accumulators are indexed by slot internally; every public method still speaks in
  // panel indices, so nothing above this line has to know.
  bool init(const Geometry& g, const int* panels, int count);

  // Whether this renderer produces pixels for that panel at all.
  bool rendersPanel(int panel) const {
    return panel >= 0 && panel < kMaxPanels && slotOf_[panel] >= 0;
  }
  // Panel index for render slot s, or -1.
  int panelAtSlot(int s) const { return (s >= 0 && s < renderCount_) ? panelOf_[s] : -1; }
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

  // clear -> splat particles -> splat heat. Stops short of resolving, so a caller with no room
  // for a second full copy of the panels can resolve one face at a time into its own buffer.
  // This is the ESP32 path; render() below is this plus the resolve loop.
  void accumulate(const Particles& p, const FieldGrid& f, const Geometry& g);

#if PARTSIM_INTERNAL_PIXELS
  // clear -> splat particles -> splat heat -> resolve every panel into the RGBA buffers.
  void render(const Particles& p, const FieldGrid& f, const Geometry& g);
#endif

  void clear();
  void splat(const Particles& p, const Geometry& g);
  // Heat cells splat through the SAME path as particles, into the heat channel, so flame and
  // fluid composite consistently and the palette stays the only place colour is decided.
  void splatField(const FieldGrid& f, const Geometry& g);
  // bytesPerTexel: 3 for tight RGB (LED panels), 4 for RGBA (WebGL wants alignment 4).
  void resolve(int panel, uint8_t* out, int bytesPerTexel) const;

#if PARTSIM_INTERNAL_PIXELS
  // Stable-address RGBA8 buffer for panel i, filled by render(). The WASM layer hands these
  // addresses to JS once and never again, so the browser uploads straight out of the heap.
  // Null for a panel this renderer does not drive.
  const uint8_t* panelPixels(int i) const {
    return rendersPanel(i) ? pixels_[slotOf_[i]] : nullptr;
  }
#endif
  // Panels this renderer produces pixels for -- not how many exist in the geometry.
  int panelCount() const { return renderCount_; }
  // Kernel half-width in texels, derived from kSplatRadiusWorld and the pitch. Exposed so a test
  // can assert it actually tracks the pitch rather than silently staying at its old value.
  int footprint() const { return footprint_; }

  // Raw accumulated intensity, for tests. Zero for a panel that is not driven here.
  uint16_t accumAt(int panel, int i, int j, int w, int channel) const {
    if (!rendersPanel(panel)) return 0;
    return accum_[slotOf_[panel]][(j * w + i) * kChannelCount + channel];
  }

 private:
  static constexpr int kAttenSize = 64;
  static constexpr int kKernelSize = 64;

  const Palette* palette_ = nullptr;
  const Palette* paletteB_ = nullptr;  // crossfade target, null when not fading
  float blend_ = 0.0f;
  // Render slots, not panels. slotOf_ maps panel -> slot (-1 for undriven), panelOf_ back again.
  int renderCount_ = 0;
  int slotOf_[kMaxPanels];
  int panelOf_[kMaxRenderPanels];
  int texels_[kMaxRenderPanels] = {0};
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
  // Kernel half-width in texels, derived from kSplatRadiusWorld and the panel pitch in init().
  // At pitch 1.0 this is 2, the value it used to be hardcoded to.
  int footprint_ = 2;

  uint16_t accum_[kMaxRenderPanels][kMaxPanelTexels * kChannelCount];
#if PARTSIM_INTERNAL_PIXELS
  uint8_t pixels_[kMaxRenderPanels][kMaxPanelTexels * 4];
#endif
};

}  // namespace partsim
