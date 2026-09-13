#include "partsim/Renderer.h"

#include <cstddef>

namespace partsim {
namespace {

// Which accumulation channel a material writes into.
inline int channelOf(uint8_t material) {
#if PARTSIM_ENABLE_SAND
  return material == kSand ? (int)kChSand : (int)kChWater;
#else
  (void)material;
  return (int)kChWater;
#endif
}

inline uint16_t satAdd(uint16_t a, int b) {
  const int s = (int)a + b;
  return (uint16_t)(s > 65535 ? 65535 : s);
}

}  // namespace

bool Renderer::init(const Geometry& g) {
  int all[kMaxPanels];
  const int n = imin(g.count(), kMaxPanels);
  for (int i = 0; i < n; ++i) all[i] = i;
  return init(g, all, n);
}

bool Renderer::init(const Geometry& g, const int* panels, int count) {
  for (int i = 0; i < kMaxPanels; ++i) slotOf_[i] = -1;
  renderCount_ = 0;
  if (count < 0 || count > kMaxRenderPanels) return false;

  for (int s = 0; s < count; ++s) {
    const int p = panels[s];
    if (p < 0 || p >= g.count()) return false;
    if (slotOf_[p] >= 0) return false;  // the same face listed twice would be blitted twice
    slotOf_[p] = s;
    panelOf_[s] = p;
    texels_[s] = (int)g.at(p).w * (int)g.at(p).h;
    width_[s] = (int)g.at(p).w;
  }
  renderCount_ = count;
  if (!palette_) palette_ = &paletteNaturalistic();

  // Depth attenuation: (1 - d/D)^2, so full brightness against the glass falling smoothly to
  // exactly zero at D. Squared rather than linear because it reads more like light scattering
  // through a translucent medium.
  attenScale_ = (float)kAttenSize / kSplatInfluence;
  for (int q = 0; q <= kAttenSize; ++q) {
    const float t = 1.0f - (float)q / (float)kAttenSize;
    atten_[q] = (uint8_t)(255.0f * t * t + 0.5f);
  }
  atten_[kAttenSize] = 0;

  // Heat's falloff is gentler as well as longer-ranged: squared would leave a plume crossing
  // the middle of the box nearly invisible on the side faces. This is between linear and
  // squared, and still reaches exactly zero at kHeatInfluence so the support stays compact.
  heatAttenScale_ = (float)kAttenSize / kHeatInfluence;
  for (int q = 0; q <= kAttenSize; ++q) {
    const float t = 1.0f - (float)q / (float)kAttenSize;
    heatAtten_[q] = (uint8_t)(255.0f * t * (0.35f + 0.65f * t) + 0.5f);
  }
  heatAtten_[kAttenSize] = 0;

  // Radial kernel over squared distance in texels, out to the blob edge. Gaussian-ish but forced
  // to zero at the rim so the footprint really is bounded.
  //
  // The radius arrives in WORLD units and is converted here, which is the only place that
  // conversion happens. Panels are required to share a pitch (a daisy chain of identical tiles
  // always does), so one LUT and one footprint serve all of them.
  const float pitch = (g.count() > 0) ? length(g.at(0).u) : 1.0f;
  const float rMax = kSplatRadiusWorld / pitch;  // texels
  footprint_ = imax(1, (int)rMax);
  kernelScale_ = (float)kKernelSize / (rMax * rMax);
#if PARTSIM_ENABLE_CHROMA
  // The chroma disc, expressed in the SAME LUT index the weight splat already computes, so the
  // narrow test is one compare rather than a second distance.
  const float rc = kChromaRadiusWorld / pitch;
  chromaKq_ = (int)(rc * rc * kernelScale_);
  if (chromaKq_ > kKernelSize) chromaKq_ = kKernelSize;
#endif
  for (int q = 0; q <= kKernelSize; ++q) {
    const float r2 = (float)q / kernelScale_;
    const float t = 1.0f - r2 / (rMax * rMax);
    kernel_[q] = (uint8_t)(255.0f * pmax(0.0f, t * t) + 0.5f);
  }
  kernel_[kKernelSize] = 0;

#if PARTSIM_ENABLE_INK
  buildInkTables(g);
#else
  (void)g;
#endif

  clear();
  return true;
}

#if PARTSIM_ENABLE_INK
// Concentration response and the reciprocal that keeps the colour mix divide-free, plus the
// per-face mapping from panel axes to field axes.
void Renderer::buildInkTables(const Geometry& g) {
  inkOpacity_[0] = 0;
  inkRecip_[0] = 0;
  for (int i = 1; i <= kInkSumMax; ++i) {
    // Linear to saturation for now. Section 3.2 of the design document wants a curve with bright
    // cores and still-visible faint wisps; that is a tuning decision and it needs the visual
    // fixtures to tune against, so it is deliberately not guessed at here.
    inkOpacity_[i] = (uint8_t)imin(255, (i * kInkOpacityGain) >> 8);
    inkRecip_[i] = (uint16_t)imin(65535, 65536 / i);
  }

  for (int sIdx = 0; sIdx < renderCount_; ++sIdx) {
    InkColumns& c = inkCols_[sIdx];
    c.valid = false;
    const Panel& pan = g.at(panelOf_[sIdx]);
    // Dominant axis of each basis vector. On an axis-aligned cube these are exact unit vectors,
    // and anything else is rejected rather than approximated -- a projection through a tilted
    // panel would silently shear the field.
    const Vec3 basis[3] = {pan.u, pan.v, pan.n};
    int8_t axis[3], sign[3];
    bool ok = true;
    int seen = 0;
    for (int b = 0; b < 3 && ok; ++b) {
      const float a[3] = {basis[b].x, basis[b].y, basis[b].z};
      int best = 0;
      for (int k = 1; k < 3; ++k)
        if (pabs(a[k]) > pabs(a[best])) best = k;
      const float mag = length(basis[b]);
      // Every off-axis component must be negligible against the dominant one.
      for (int k = 0; k < 3; ++k)
        if (k != best && pabs(a[k]) > 0.001f * mag) ok = false;
      axis[b] = (int8_t)best;
      sign[b] = (int8_t)(a[best] >= 0.0f ? 1 : -1);
      seen |= 1 << best;
    }
    if (!ok || seen != 0x7) continue;  // not axis-aligned, or two basis vectors on one axis

    c.uAxis = axis[0]; c.uSign = sign[0];
    c.vAxis = axis[1]; c.vSign = sign[1];
    c.dAxis = axis[2]; c.dSign = sign[2];
    c.valid = true;
  }
}
#endif  // PARTSIM_ENABLE_INK

void Renderer::clear() {
  for (int k = 0; k < renderCount_; ++k) {
    const int n = texels_[k] * kChannelCount;
    for (int i = 0; i < n; ++i) accum_[k][i] = 0;
  }
}

#if PARTSIM_ENABLE_CHROMA
// Dye for one texel, premultiplied by the kernel weight, inside the narrow disc only.
//
// `row` is already offset to this particle's WEIGHT channel, so the chroma channels are reached
// relative to it -- which keeps the hot loop free of a second base pointer.
//
// Scaled by >>8 rather than /255. The S3's FPU has no divide and the integer one is not free
// either, and it does not matter: resolve() takes the RATIO of these channels, so a common factor
// of 255/256 cancels exactly. Blue is implied, so all three sum to the narrow weight and that sum
// is the denominator resolve needs.
void Renderer::splatChroma(uint16_t* row, int ii, int kq, int contrib, int cr, int cg) {
  if (kq >= chromaKq_) return;
  uint16_t* c = row + (size_t)ii * kChannelCount - (size_t)kChWater;
  c[kChCR] = satAdd(c[kChCR], (contrib * cr) >> 8);
  c[kChCG] = satAdd(c[kChCG], (contrib * cg) >> 8);
  c[kChCB] = satAdd(c[kChCB], (contrib * (255 - cr - cg)) >> 8);
}
#endif

void Renderer::splat(ParticleView p, const Geometry& g) {
  const int n = p.n;
  const float dtOff = timeOffset_;
  for (int i = 0; i < n; ++i) {
    // Extrapolate along the particle's own velocity. Only the splat moves; the simulation state
    // is untouched, so this can never feed back into the physics.
    const Vec3 pos{p.x[i], p.y[i], p.z[i]};
    const Vec3 q = (dtOff == 0.0f) ? pos : pos + Vec3{p.vx[i], p.vy[i], p.vz[i]} * dtOff;
    const int ch = channelOf(p.mat[i]);
#if PARTSIM_ENABLE_CHROMA
    // 8.8 down to 0..255 for the premultiply below; the low bits matter for MIXING, not
    // for one texel of splat.
    const int cr = p.cr[i] >> 8, cg = p.cg[i] >> 8;
#endif

    // Brute force over the panels this node drives. With at most 8 of them the rejection test is
    // one dot product each, and any acceleration structure would cost more than it saves --
    // while also breaking the property that a particle in a corner correctly lights three faces.
    //
    // Iterating the render set rather than the whole panel table is the entire saving on a
    // display node: it tests two faces instead of six, per particle.
    for (int k = 0; k < renderCount_; ++k) {
      const Panel& pan = g.at(panelOf_[k]);
      const Vec3 d = q - pan.origin;
      const float dist = dot(d, pan.n);
      if (dist < 0.0f || dist >= kSplatInfluence) continue;

      const float s = dot(d, pan.u) * pan.invU2;
      const float t = dot(d, pan.v) * pan.invV2;

      const int i0 = imax(0, (int)s - footprint_);
      const int i1 = imin((int)pan.w - 1, (int)s + footprint_);
      const int j0 = imax(0, (int)t - footprint_);
      const int j1 = imin((int)pan.h - 1, (int)t + footprint_);
      if (i0 > i1 || j0 > j1) continue;

      const int a = atten_[(int)(dist * attenScale_)];
      if (a == 0) continue;

      uint16_t* dst = accum_[k];
      const int w = (int)pan.w;
      // The kernel is circular and the box is square, so about a fifth of the texels in [i0,i1]
      // can never contribute. Rather than compute the chord width per row -- which wants a square
      // root, and the S3's FPU has none (DECISIONS.md F1) -- walk outward from the texel nearest
      // the particle and stop at the first miss.
      //
      // That is exact, not a heuristic, and it is why the pixel hash does not move: |dx| grows
      // monotonically away from `ic`, so does dx*dx + dy2, so does the product with kernelScale_,
      // and so does its truncation to int. A texel past the first failure cannot pass the test
      // the old loop applied to it. The same argument in dy gives the whole-row rejection.
      //
      // Sweeping right-then-left visits a row in a different ORDER than before. Every texel in a
      // row is a different cell, so the saturating add sees the same operands either way.
      for (int j = j0; j <= j1; ++j) {
        const float dy = ((float)j + 0.5f) - t;
        const float dy2 = dy * dy;
        if ((int)(dy2 * kernelScale_) >= kKernelSize) continue;  // row entirely outside the disc
        uint16_t* row = dst + ((size_t)j * (size_t)w) * kChannelCount + ch;

        // (int)s truncates rather than floors, which differ only when s < 0 -- and there both
        // land below i0, so the clamp gives the same texel either way.
        const int ic = iclamp((int)s, i0, i1);
        for (int ii = ic; ii <= i1; ++ii) {
          const float dx = ((float)ii + 0.5f) - s;
          const int kq = (int)((dx * dx + dy2) * kernelScale_);
          if (kq >= kKernelSize) break;
          // >> 6 rather than >> 8: at >> 8 a single particle's contribution maxes out at 255
          // and the dim tail of the falloff rounds to zero, truncating the outer glow. Two
          // extra bits keep that tail, and a dense texel still only reaches ~6500 of the
          // 65535 a uint16 holds.
          const int contrib = (a * kernel_[kq]) >> 6;
          if (contrib == 0) continue;
          uint16_t& cell = row[(size_t)ii * kChannelCount];
          cell = satAdd(cell, contrib);
#if PARTSIM_ENABLE_CHROMA
          splatChroma(row, ii, kq, contrib, cr, cg);
#endif
        }
        for (int ii = ic - 1; ii >= i0; --ii) {
          const float dx = ((float)ii + 0.5f) - s;
          const int kq = (int)((dx * dx + dy2) * kernelScale_);
          if (kq >= kKernelSize) break;
          const int contrib = (a * kernel_[kq]) >> 6;
          if (contrib == 0) continue;
          uint16_t& cell = row[(size_t)ii * kChannelCount];
          cell = satAdd(cell, contrib);
#if PARTSIM_ENABLE_CHROMA
          splatChroma(row, ii, kq, contrib, cr, cg);
#endif
        }
      }
    }
  }
}

void Renderer::resolve(int panel, uint8_t* out, int bytesPerTexel) const {
  if (!rendersPanel(panel)) return;  // not ours to draw; leave the caller's buffer alone
  const int slot = slotOf_[panel];
  const uint16_t* src = accum_[slot];
  const int n = texels_[slot];
  const Palette& pal = *palette_;
  const float toLevel = 255.0f / fullScale_;
  const bool fading = paletteB_ != nullptr && blend_ > 0.0f;
  const int wB = (int)(blend_ * 256.0f), wA = 256 - wB;

  // One lookup when not fading, two and a lerp while a scene transition is in flight.
  auto ramp = [&](const Ramp& a, const Ramp& b, int level, uint8_t out[3]) {
    rampLookup(a, level, out);
    if (!fading) return;
    uint8_t o2[3];
    rampLookup(b, level, o2);
    for (int c = 0; c < 3; ++c)
      out[c] = (uint8_t)(((int)out[c] * wA + (int)o2[c] * wB) >> 8);
  };

  for (int i = 0; i < n; ++i) {
    const uint16_t aw = src[i * kChannelCount + kChWater];
#if PARTSIM_ENABLE_SAND
    const uint16_t as = src[i * kChannelCount + kChSand];
#endif
#if PARTSIM_ENABLE_HEAT
    const uint16_t ah = src[i * kChannelCount + kChHeat];
#endif

    int r = 0, gg = 0, b = 0;
    uint8_t c[3];
    const Palette& palB = fading ? *paletteB_ : pal;
#if PARTSIM_ENABLE_CHROMA
    // Colour is the RATIO of the chroma channels; brightness comes from the weight channel, which
    // was splatted through the wider kernel. Keeping them separate is the whole point: the surface
    // stays continuous at the weight radius while the dye stays sharp at half of it.
    //
    // The three chroma channels sum to the narrow disc's own accumulated weight, which is exactly
    // the denominator this needs -- and it is why blue is stored rather than implied.
    const uint16_t cR = src[i * kChannelCount + kChCR];
    const uint16_t cG = src[i * kChannelCount + kChCG];
    const uint16_t cB = src[i * kChannelCount + kChCB];
    const int cSum = (int)cR + (int)cG + (int)cB;
    if (aw && cSum > 0) {
      // Brightness through the water ramp as usual, then tinted. A texel lit by the wide kernel but
      // outside every particle's narrow disc has cSum == 0 and keeps the untinted ramp colour,
      // which is the right answer for the faint outer glow: no particle is close enough to say
      // what colour it is.
      ramp(pal.water, palB.water, iclamp((int)((float)aw * toLevel), 0, 255), c);
      const int lum = (c[0] * 77 + c[1] * 151 + c[2] * 28) >> 8;
      // One divide per LIT texel, not per channel. The shift is 8, not 16: the ratio wanted here
      // is cX/cSum scaled to 0..256, and >>16 collapses it to the integer 0 or 1 -- which renders
      // the fluid almost black, because a pure dye then contributes lum*3/256 instead of lum*3.
      const int inv = 65536 / cSum;
      r += (lum * 3 * (((int)cR * inv) >> 8)) >> 8;
      gg += (lum * 3 * (((int)cG * inv) >> 8)) >> 8;
      b += (lum * 3 * (((int)cB * inv) >> 8)) >> 8;
    } else if (aw) {
      // Lit by the weight kernel but outside every particle's chroma disc. Rendering it through
      // the palette would assert a colour no particle claimed -- and at a narrow chroma radius that
      // paints a fringe of undyed palette blue along the surface of a magenta pool, which reads as
      // the colour floating on the fluid rather than being in it.
      //
      // Neutral at the same luminance instead: no particle is close enough to say what colour this
      // texel is, and a white glow is what the outer tail of a splat actually looks like.
      ramp(pal.water, palB.water, iclamp((int)((float)aw * toLevel), 0, 255), c);
      const int lum = (c[0] * 77 + c[1] * 151 + c[2] * 28) >> 8;
      r += lum; gg += lum; b += lum;
    }
#else
    if (aw) {
      ramp(pal.water, palB.water, iclamp((int)((float)aw * toLevel), 0, 255), c);
      r += c[0]; gg += c[1]; b += c[2];
    }
#endif
#if PARTSIM_ENABLE_SAND
    if (as) {
      ramp(pal.sand, palB.sand, iclamp((int)((float)as * toLevel), 0, 255), c);
      r += c[0]; gg += c[1]; b += c[2];
    }
#endif
#if PARTSIM_ENABLE_HEAT
    if (ah) {
      ramp(pal.heat, palB.heat, iclamp((int)((float)ah * (255.0f / kHeatGain)), 0, 255), c);
      r += c[0]; gg += c[1]; b += c[2];
    }
#endif

    uint8_t* o = out + (std::size_t)i * (std::size_t)bytesPerTexel;
#if PARTSIM_QUANTISE_OUTPUT
    // Round to what the panel can actually display. The hardware already does this by taking the
    // top kColourBits of each channel, so on the device this changes nothing -- it makes the
    // BROWSER show the same banding, which is the whole point of the browser.
    //
    // Shift down and replicate the high bits back up rather than shifting down and up, so full
    // scale stays 255 instead of collapsing to 252 at 6 bits.
    constexpr int kDrop = 8 - kColourBits;
    auto q = [](int v) {
      const int t = imin(255, v) >> kDrop;
      return (uint8_t)((t << kDrop) | (t >> imax(0, kColourBits - kDrop)));
    };
    o[0] = q(r);
    o[1] = q(gg);
    o[2] = q(b);
#else
    o[0] = (uint8_t)imin(255, r);
    o[1] = (uint8_t)imin(255, gg);
    o[2] = (uint8_t)imin(255, b);
#endif
    if (bytesPerTexel == 4) o[3] = 255;
  }
}

void Renderer::splatField(HeatView f, const Geometry& g) {
#if !PARTSIM_ENABLE_HEAT
  // No heat channel to accumulate into. The signature stays so callers need no guard.
  (void)f;
  (void)g;
  return;
#else
  if (f.empty) return;  // nothing burning: whole pass skipped

  const IVec3 d = f.dim;
  for (int z = 0; z < d.z; ++z) {
    for (int y = 0; y < d.y; ++y) {
      for (int x = 0; x < d.x; ++x) {
        const uint8_t heat = f.atCoord(x, y, z);
        if (heat < kHeatFloor) continue;  // most of a cold volume exits here

        const Vec3 q = f.cellCentre(x, y, z);
        const float gain = (float)heat * (1.0f / 255.0f);

        for (int k = 0; k < renderCount_; ++k) {
          const Panel& pan = g.at(panelOf_[k]);
          const Vec3 dd = q - pan.origin;
          const float dist = dot(dd, pan.n);
          if (dist < 0.0f || dist >= kHeatInfluence) continue;

          const float s = dot(dd, pan.u) * pan.invU2;
          const float t = dot(dd, pan.v) * pan.invV2;
          const int i0 = imax(0, (int)s - footprint_);
          const int i1 = imin((int)pan.w - 1, (int)s + footprint_);
          const int j0 = imax(0, (int)t - footprint_);
          const int j1 = imin((int)pan.h - 1, (int)t + footprint_);
          if (i0 > i1 || j0 > j1) continue;

          const int a = heatAtten_[(int)(dist * heatAttenScale_)];
          if (a == 0) continue;

          uint16_t* dst = accum_[k];
          const int w = (int)pan.w;
          for (int j = j0; j <= j1; ++j) {
            const float dy = ((float)j + 0.5f) - t;
            const float dy2 = dy * dy;
            for (int ii = i0; ii <= i1; ++ii) {
              const float dx = ((float)ii + 0.5f) - s;
              const int kq = (int)((dx * dx + dy2) * kernelScale_);
              if (kq >= kKernelSize) continue;
              const int contrib = (int)((float)((a * kernel_[kq]) >> 6) * gain);
              if (contrib == 0) continue;
              uint16_t& cell = dst[((j * w) + ii) * kChannelCount + kChHeat];
              cell = satAdd(cell, contrib);
            }
          }
        }
      }
    }
  }
#endif
}

#if PARTSIM_ENABLE_INK
void Renderer::splatInk(const InkField& f, const Geometry& g) { splatInk(f.view(), g); }
void Renderer::splatInk(const InkField& f, const Geometry& g, uint32_t serial, int phaseQ8) {
  splatInk(f.view(), g, serial, phaseQ8);
}

// One face's 16x16 projection. Split out because it now runs at the FIELD rate rather than the
// display rate -- see splatInk below.
void Renderer::projectInkFace(InkView f, int s, uint8_t* out) {
  {
    const InkColumns& c = inkCols_[s];

    // --- one column per intermediate texel, composited front to back -------------------------
    for (int iv = 0; iv < kInkDim; ++iv) {
      for (int iu = 0; iu < kInkDim; ++iu) {
        int coord[3];
        coord[c.uAxis] = (c.uSign > 0) ? iu : (kInkDim - 1 - iu);
        coord[c.vAxis] = (c.vSign > 0) ? iv : (kInkDim - 1 - iv);

        int trans = 255, cr = 0, cg = 0, cb = 0;
        for (int d = 0; d < kInkDim; ++d) {
          // Depth runs inward from this face, so opposite faces walk the same field in opposite
          // order -- which is the whole reason one shared volume reads correctly from six sides.
          coord[c.dAxis] = (c.dSign > 0) ? d : (kInkDim - 1 - d);
          const int idx = f.index(coord[0], coord[1], coord[2]);

          int sum = 0, mix[3] = {0, 0, 0};
          for (int ch = 0; ch < kInkChannels; ++ch) {
            const int m = f.dye[ch][idx];
            if (!m) continue;
            sum += m;
            mix[0] += m * kInkDyeColour[ch][0];
            mix[1] += m * kInkDyeColour[ch][1];
            mix[2] += m * kInkDyeColour[ch][2];
          }
          if (sum == 0) continue;  // clear carrier liquid: most of a young plume exits here

          const int alpha = inkOpacity_[sum];
          const int contrib = (trans * alpha) >> 8;
          if (contrib > 0) {
            // Colour is the concentration-weighted mix of the dye colours. The division by `sum`
            // that normalises it comes from a LUT: section 2.3's no-divide rule applies here too.
            const int rcp = inkRecip_[sum];
            cr += (((mix[0] >> 8) * rcp) >> 8) * contrib >> 8;
            cg += (((mix[1] >> 8) * rcp) >> 8) * contrib >> 8;
            cb += (((mix[2] >> 8) * rcp) >> 8) * contrib >> 8;
          }
          trans -= (trans * alpha) >> 8;
          if (trans <= 3) break;  // effectively opaque; nothing behind it can show through
        }

        // Normalise the colour to full scale, keeping only its HUE.
        //
        // resolve() uses the chroma channels purely as a ratio and takes brightness from the
        // weight channel, so carrying brightness here as well is not merely redundant -- it is
        // harmful. Premultiplied by `contrib`, a faint column's colour is small, and three >>8
        // shifts plus the bilinear upscale round it to zero while the opacity (scaled by ~900)
        // survives. A texel with weight and no chroma takes resolve()'s untinted branch and comes
        // out NEUTRAL WHITE, which is right for a particle outside its narrow chroma disc and
        // meaningless for ink, where both numbers describe the same dye.
        //
        // Measured before this: 7% of lit texels, none of them while the dye was fresh and all of
        // them once it began to disperse -- so the plume grew a shimmering white fringe that
        // flickered as texels crossed the truncation threshold in and out.
        int cmax = cr > cg ? cr : cg;
        if (cb > cmax) cmax = cb;
        if (cmax > 0) {
          const int rcp = inkRecip_[imin(cmax, kInkSumMax)];  // 65536/cmax
          cr = imin(255, (cr * rcp) >> 8);
          cg = imin(255, (cg * rcp) >> 8);
          cb = imin(255, (cb * rcp) >> 8);
        }

        uint8_t* o = &out[(iv * kInkDim + iu) * 4];
        o[0] = (uint8_t)iclamp(255 - trans, 0, 255);
        o[1] = (uint8_t)cr;
        o[2] = (uint8_t)cg;
        o[3] = (uint8_t)cb;
      }
    }

  }
}

// Non-interpolating form: every call is its own field state, shown in full.
//
// It must NOT increment inkSerial_, which is the "last field I projected" marker the interpolating
// path compares against -- incrementing it here made the argument equal to the member, so the
// comparison never fired and the projection was never computed at all.
void Renderer::splatInk(InkView f, const Geometry& g) { splatInk(f, g, ++inkAutoSerial_, 256); }

// Project at the field rate, upscale at the display rate, and cross-fade between the two most
// recent projections.
//
// The field runs at 20 Hz and the browser draws at 60, so without this the image changes on
// exactly every third frame and holds still for the other two: measured at ~880 texels jumping at
// once, which is a 20 Hz strobe across the whole plume rather than motion. Section 2.3 of the
// design document calls for interpolating the PROJECTIONS rather than the 3D state, and that is
// what this does -- two 16x16 images per face, not two copies of the volume.
//
// It is also cheaper than what it replaces. The column walk used to run every frame against a
// field that had not changed; now it runs only when the field actually steps.
//
// `phaseQ8` is 0..256 across the interval between field steps. The picture therefore lags the
// simulation by one field step, which at 20 Hz is 50 ms and is the standard cost of interpolating
// rather than extrapolating.
void Renderer::splatInk(InkView f, const Geometry& g, uint32_t serial, int phaseQ8) {
  const int n = f.dim.x;
  // The intermediate is one texel per field cell, which is what makes the column walk a direct
  // index rather than a trilinear sample. A field that is not cubic would need the general path.
  if (n != kInkDim || f.dim.y != kInkDim || f.dim.z != kInkDim) return;

  if (f.empty) {
    // Snap rather than fade: a cleared volume should go dark at once, and holding a ghost of it
    // for 50 ms reads as the clear having failed.
    if (serial != inkSerial_) {
      for (int s = 0; s < renderCount_; ++s) {
        for (int i = 0; i < kInkDim * kInkDim * 4; ++i) { inkProj_[s][i] = 0; inkPrev_[s][i] = 0; }
      }
      inkSerial_ = serial;
    }
    return;
  }

  if (serial != inkSerial_) {
    for (int s = 0; s < renderCount_; ++s) {
      if (!inkCols_[s].valid) continue;
      for (int i = 0; i < kInkDim * kInkDim * 4; ++i) inkPrev_[s][i] = inkProj_[s][i];
      projectInkFace(f, s, inkProj_[s]);
      // First field the renderer has ever seen: there is no earlier projection to come from, and
      // fading up out of the previous buffer's stale contents would be worse than not fading.
      if (!inkPrimed_)
        for (int i = 0; i < kInkDim * kInkDim * 4; ++i) inkPrev_[s][i] = inkProj_[s][i];
    }
    inkPrimed_ = true;
    inkSerial_ = serial;
  }

  const int t = iclamp(phaseQ8, 0, 256);
  for (int s = 0; s < renderCount_; ++s) {
    if (!inkCols_[s].valid) continue;
    const Panel& pan = g.at(panelOf_[s]);

    // Blend the two projections once, over 256 intermediate texels, rather than during the
    // upscale where it would cost four more reads per PANEL texel.
    for (int i = 0; i < kInkDim * kInkDim * 4; ++i) {
      const int a = inkPrev_[s][i], b = inkProj_[s][i];
      inkFace_[i] = (uint8_t)(a + (((b - a) * t) >> 8));
    }

    // --- bilinear upscale into the accumulators ----------------------------------------------
    // kInkDim is below the panel resolution on purpose; this is what hides the grid. Sampling at
    // texel centres in both spaces, so the image is not shifted half an intermediate texel.
    const int w = (int)pan.w, h = (int)pan.h;
    uint16_t* dst = accum_[s];
    // Weight such that resolve()'s level is the opacity itself: the field already states how
    // opaque a column is, so exposure is not the knob for it.
    const int wScale = (int)(fullScale_ * (256.0f / 255.0f));
    for (int j = 0; j < h; ++j) {
      const int fy = ((j * 2 + 1) * kInkDim * 128) / h - 128;  // Q8, centre-to-centre
      const int y0 = iclamp(fy >> 8, 0, kInkDim - 1);
      const int y1 = imin(y0 + 1, kInkDim - 1);
      const int ty = iclamp(fy - (y0 << 8), 0, 255);
      for (int i = 0; i < w; ++i) {
        const int fx = ((i * 2 + 1) * kInkDim * 128) / w - 128;
        const int x0 = iclamp(fx >> 8, 0, kInkDim - 1);
        const int x1 = imin(x0 + 1, kInkDim - 1);
        const int tx = iclamp(fx - (x0 << 8), 0, 255);

        const uint8_t* p00 = &inkFace_[(y0 * kInkDim + x0) * 4];
        const uint8_t* p10 = &inkFace_[(y0 * kInkDim + x1) * 4];
        const uint8_t* p01 = &inkFace_[(y1 * kInkDim + x0) * 4];
        const uint8_t* p11 = &inkFace_[(y1 * kInkDim + x1) * 4];

        int v[4];
        for (int k = 0; k < 4; ++k) {
          const int a = p00[k] + (((p10[k] - p00[k]) * tx) >> 8);
          const int b = p01[k] + (((p11[k] - p01[k]) * tx) >> 8);
          v[k] = a + (((b - a) * ty) >> 8);
        }
        if (v[0] == 0) continue;
#if PARTSIM_ENABLE_CHROMA
        // A texel whose hue interpolated away entirely holds no dye worth drawing, and writing
        // weight without chroma is precisely what makes resolve() paint it neutral. This is the
        // last 0.2% the hue normalisation above cannot reach: opacity rounds to 1 while all three
        // channels round to 0, which needs an interpolation weight below 1/255. Dropping it costs
        // a texel at level 1 -- indistinguishable from black -- and removes the white speckle.
        if (v[1] + v[2] + v[3] == 0) continue;
#endif

        uint16_t* row = dst + (std::size_t)(j * w + i) * kChannelCount;
        row[kChWater] = satAdd(row[kChWater], (v[0] * wScale) >> 8);
#if PARTSIM_ENABLE_CHROMA
        row[kChCR] = satAdd(row[kChCR], v[1]);
        row[kChCG] = satAdd(row[kChCG], v[2]);
        row[kChCB] = satAdd(row[kChCB], v[3]);
#endif
      }
    }
  }
}
#endif  // PARTSIM_ENABLE_INK

void Renderer::accumulate(ParticleView p, HeatView f, const Geometry& g) {
  clear();
  splat(p, g);
  splatField(f, g);
}

#if PARTSIM_INTERNAL_PIXELS
void Renderer::render(ParticleView p, HeatView f, const Geometry& g) {
  accumulate(p, f, g);
  resolveAll();
}

void Renderer::resolveAll() {
  for (int k = 0; k < renderCount_; ++k) resolve(panelOf_[k], pixels_[k], 4);
}
#endif

}  // namespace partsim
