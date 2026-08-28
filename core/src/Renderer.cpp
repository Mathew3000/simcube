#include "partsim/Renderer.h"

#include <cstddef>

namespace partsim {
namespace {

// Which accumulation channel a material writes into.
inline int channelOf(uint8_t material) {
  return material == kSand ? (int)kChSand : (int)kChWater;
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
  for (int q = 0; q <= kKernelSize; ++q) {
    const float r2 = (float)q / kernelScale_;
    const float t = 1.0f - r2 / (rMax * rMax);
    kernel_[q] = (uint8_t)(255.0f * pmax(0.0f, t * t) + 0.5f);
  }
  kernel_[kKernelSize] = 0;

  clear();
  return true;
}

void Renderer::clear() {
  for (int k = 0; k < renderCount_; ++k) {
    const int n = texels_[k] * kChannelCount;
    for (int i = 0; i < n; ++i) accum_[k][i] = 0;
  }
}

void Renderer::splat(ParticleView p, const Geometry& g) {
  const int n = p.n;
  const float dtOff = timeOffset_;
  for (int i = 0; i < n; ++i) {
    // Extrapolate along the particle's own velocity. Only the splat moves; the simulation state
    // is untouched, so this can never feed back into the physics.
    const Vec3 pos{p.x[i], p.y[i], p.z[i]};
    const Vec3 q = (dtOff == 0.0f) ? pos : pos + Vec3{p.vx[i], p.vy[i], p.vz[i]} * dtOff;
    const int ch = channelOf(p.mat[i]);

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
      for (int j = j0; j <= j1; ++j) {
        const float dy = ((float)j + 0.5f) - t;
        const float dy2 = dy * dy;
        for (int ii = i0; ii <= i1; ++ii) {
          const float dx = ((float)ii + 0.5f) - s;
          const int kq = (int)((dx * dx + dy2) * kernelScale_);
          if (kq >= kKernelSize) continue;
          // >> 6 rather than >> 8: at >> 8 a single particle's contribution maxes out at 255
          // and the dim tail of the falloff rounds to zero, truncating the outer glow. Two
          // extra bits keep that tail, and a dense texel still only reaches ~6500 of the
          // 65535 a uint16 holds.
          const int contrib = (a * kernel_[kq]) >> 6;
          if (contrib == 0) continue;
          uint16_t& cell = dst[((j * w) + ii) * kChannelCount + ch];
          cell = satAdd(cell, contrib);
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
    const uint16_t as = src[i * kChannelCount + kChSand];
    const uint16_t ah = src[i * kChannelCount + kChHeat];

    int r = 0, gg = 0, b = 0;
    uint8_t c[3];
    const Palette& palB = fading ? *paletteB_ : pal;
    if (aw) {
      ramp(pal.water, palB.water, iclamp((int)((float)aw * toLevel), 0, 255), c);
      r += c[0]; gg += c[1]; b += c[2];
    }
    if (as) {
      ramp(pal.sand, palB.sand, iclamp((int)((float)as * toLevel), 0, 255), c);
      r += c[0]; gg += c[1]; b += c[2];
    }
    if (ah) {
      ramp(pal.heat, palB.heat, iclamp((int)((float)ah * (255.0f / kHeatGain)), 0, 255), c);
      r += c[0]; gg += c[1]; b += c[2];
    }

    uint8_t* o = out + (std::size_t)i * (std::size_t)bytesPerTexel;
    o[0] = (uint8_t)imin(255, r);
    o[1] = (uint8_t)imin(255, gg);
    o[2] = (uint8_t)imin(255, b);
    if (bytesPerTexel == 4) o[3] = 255;
  }
}

void Renderer::splatField(HeatView f, const Geometry& g) {
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
}

void Renderer::accumulate(ParticleView p, HeatView f, const Geometry& g) {
  clear();
  splat(p, g);
  splatField(f, g);
}

#if PARTSIM_INTERNAL_PIXELS
void Renderer::render(const Particles& p, const FieldGrid& f, const Geometry& g) {
  accumulate(p, f, g);
  for (int k = 0; k < renderCount_; ++k) resolve(panelOf_[k], pixels_[k], 4);
}
#endif

}  // namespace partsim
