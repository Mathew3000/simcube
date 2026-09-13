#pragma once
#include "partsim/FieldGrid.h"
#include "partsim/InkField.h"
#include "partsim/Palette.h"
#include "partsim/Particles.h"
#include "partsim/Geometry.h"
#include "partsim/RenderState.h"

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
  void accumulate(ParticleView p, HeatView f, const Geometry& g);
  // Templated on the field so it accepts a FieldGrid or the no-heat stand-in Simulation
  // substitutes when PARTSIM_ENABLE_HEAT is 0. Anything exposing view() works, which is the
  // point -- the alternative was every call site naming the view explicitly.
  template <class F>
  void accumulate(const Particles& p, const F& f, const Geometry& g) {
    accumulate(p.view(), f.view(), g);
  }

#if PARTSIM_INTERNAL_PIXELS
  // clear -> splat particles -> splat heat -> resolve every driven panel into the RGBA buffers.
  void render(ParticleView p, HeatView f, const Geometry& g);
  template <class F>
  void render(const Particles& p, const F& f, const Geometry& g) {
    render(p.view(), f.view(), g);
  }
#endif

  void clear();

  // Both splat entry points take a VIEW rather than the owning type, so one implementation serves
  // a full simulation and a draw-only display node alike. See RenderState.h.
  void splat(ParticleView p, const Geometry& g);
  // Heat cells splat through the SAME path as particles, into the heat channel, so flame and
  // fluid composite consistently and the palette stays the only place colour is decided.
  void splatField(HeatView f, const Geometry& g);

  // Six low-resolution volume projections of the dye field, upscaled into the accumulators.
  //
  // Not a splat. The particle path costs O(particles x footprint) and is the right shape for a
  // sparse pool; a dense field splatted the same way would touch every panel texel for every
  // occupied cell. This walks one column of the field per INTERMEDIATE texel instead, composites
  // front to back, and bilinearly upscales kInkDim x kInkDim to the panel. Cost is therefore
  // independent of panel resolution until the upscale: six faces visit 6 * kInkDim^3 samples, and
  // a display node driving two visits a third of that.
  //
  // Writes the same channels the particle path does -- weight for brightness, chroma for colour --
  // so resolve(), the palette, the quantisation and the blit are all unchanged.
#if PARTSIM_ENABLE_INK
  void splatInk(InkView f, const Geometry& g);
  void splatInk(const InkField& f, const Geometry& g);
#endif

  // Convenience overloads for callers holding the full types. Thin adapters, not a second path.
  void splat(const Particles& p, const Geometry& g) { splat(p.view(), g); }
  void splatField(const FieldGrid& f, const Geometry& g) { splatField(f.view(), g); }
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
  //
  // The panel width used to be a caller-supplied argument, which was a quiet trap: a stale literal
  // meant the read silently landed on the wrong texel and the test carried on passing while
  // checking a fraction of the face. The renderer knows the width, so it uses its own.
  uint16_t accumAt(int panel, int i, int j, int channel) const {
    if (!rendersPanel(panel)) return 0;
    const int slot = slotOf_[panel];
    return accum_[slot][(j * width_[slot] + i) * kChannelCount + channel];
  }

  // Additive write into the accumulation buffer -- the hook BeakerOverlay composites through.
  //
  // The overlay runs AFTER the fluid has splatted and BEFORE resolve(), so it goes through the
  // same palette path the fluid does and there is no second colour pipeline to keep in step.
  // That is the whole reason it writes here rather than into pixels.
  //
  // ADDITIVE, not assignment: an overlay that assigned would punch a hole in bright fluid
  // wherever it wrote a value dimmer than what was already there, so an edge line would flicker
  // dark exactly where the liquid touches it. Saturates at the accumulator's 16 bits.
  //
  // Bounds-checked on i, j and channel, unlike the splat loop: the splat's footprint is clamped
  // by construction, whereas overlay coordinates are computed from panel dimensions, and an
  // off-by-one there would silently scribble into the next panel's buffer.
  void addAccum(int panel, int i, int j, int channel, int value) {
    if (!rendersPanel(panel) || value <= 0) return;
    const int slot = slotOf_[panel];
    const int w = width_[slot];
    const int h = (w > 0) ? texels_[slot] / w : 0;
    if (i < 0 || i >= w || j < 0 || j >= h) return;
    if (channel < 0 || channel >= kChannelCount) return;
    uint16_t& cell = accum_[slot][(j * w + i) * kChannelCount + channel];
    cell = (uint16_t)imin(65535, (int)cell + value);
  }

  // Overwriting write, for the overlay's CHROMA channels only.
  //
  // Weight is additive because brightness must never go down where the overlay draws; dye is not,
  // because resolve() takes the RATIO of the chroma channels, so adding red to the blue dye
  // already in a texel gives a magenta glyph over a full beaker and a red one over an empty
  // one. A glyph's colour is its own statement, not a mixture with whatever it covers.
  void setAccum(int panel, int i, int j, int channel, int value) {
    if (!rendersPanel(panel)) return;
    const int slot = slotOf_[panel];
    const int w = width_[slot];
    const int h = (w > 0) ? texels_[slot] / w : 0;
    if (i < 0 || i >= w || j < 0 || j >= h) return;
    if (channel < 0 || channel >= kChannelCount) return;
    accum_[slot][(j * w + i) * kChannelCount + channel] = (uint16_t)iclamp(value, 0, 65535);
  }

  // Texels and width of a driven panel, so a caller iterating pixels never has to assume either.
  int panelTexels(int panel) const { return rendersPanel(panel) ? texels_[slotOf_[panel]] : 0; }
  int panelWidth(int panel) const { return rendersPanel(panel) ? width_[slotOf_[panel]] : 0; }

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
  int width_[kMaxRenderPanels] = {0};
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

  // Which field axis each panel axis runs along, and in which direction, for the six axis-aligned
  // faces. Precomputed at init from the panel basis: deriving it per texel would be both slower
  // and a place for a per-face transpose to hide, and a transposed face is hard to see on a cube
  // whose contents are roughly symmetric.
#if PARTSIM_ENABLE_INK
  struct InkColumns {
    int8_t uAxis, vAxis, dAxis;
    int8_t uSign, vSign, dSign;
    bool valid;  // false when the panel is not axis-aligned; that face is then skipped
  };
  InkColumns inkCols_[kMaxRenderPanels];
  void buildInkTables(const Geometry& g);
  // Concentration sum -> opacity, and its reciprocal, so the colour mix needs no divide. The
  // reciprocal table is the one docs/DESIGN-SUGGESTIONS.md section 3.1 originally hid a divide
  // behind: on a host it is worth 6%, and the target has no divider at all.
  uint8_t inkOpacity_[kInkSumMax + 1];
  uint16_t inkRecip_[kInkSumMax + 1];
  // One face's intermediate image: opacity, r, g, b. Reused across faces, so this is 1 KB total
  // rather than one per driven panel.
  uint8_t inkFace_[kInkDim * kInkDim * 4];
#endif
#if PARTSIM_ENABLE_CHROMA
  // Kernel-LUT index at the chroma radius: a texel is inside the narrow disc when its kq is below
  // this, so the test is one compare on a value the weight splat has already computed.
  int chromaKq_ = 0;
  // Dye for one texel, inside the narrow disc only. Defined next to the splat loop it belongs to.
  void splatChroma(uint16_t* row, int ii, int kq, int contrib, int cr, int cg);
#endif

  uint16_t accum_[kMaxRenderPanels][kMaxPanelTexels * kChannelCount];
#if PARTSIM_INTERNAL_PIXELS
  uint8_t pixels_[kMaxRenderPanels][kMaxPanelTexels * 4];
#endif
};

}  // namespace partsim
