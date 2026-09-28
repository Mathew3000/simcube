#pragma once
#include "partsim/SurfaceWater.h"

namespace partsim {

struct DyeSample { float red, blue, green = 0; };

// Three colour channels in one object-space 16^3 grid. Q0.16 concentrations
// retain weak wisps during slow advection. No pressure solve or heap storage.
class DyeField {
 public:
  static constexpr int kSize = 16;
  static constexpr int kCells = kSize*kSize*kSize;
  void reset(DyeSample colour);
  DyeSample sample(Vec3 p) const;
  DyeSample cell(int x, int y, int z) const;
  // Blend local ink, or replace a resolved liquid parcel with a pure core.
  // liquidParcel is used only after the caller accumulates a sub-grid budget.
  void inject(Vec3 position, DyeSample colour, float amount, float radius = 0.12f, bool liquidParcel = false);
  void stir(Vec3 position, Vec3 down, float strength);
  void step(float dt, const SurfaceWater& water, Vec3 bulkVelocity, float agitation = 0.0f);

 private:
  struct Vortex { Vec3 position, axis; float strength; };
  uint16_t grid_[3][kCells][3] = {};
  // Fractional diffusion increments survive Q0.16 rounding (12 KiB).
  int8_t diffusionRemainder_[kCells][3] = {};
  void diffuse(float dt, const SurfaceWater& water, float agitation);
  DyeSample sampleBuffer(int buffer, Vec3 p) const;
  Vec3 displacement(Vec3 p, float dt, Vec3 down, Vec3 bulkVelocity) const;
  Vortex vortices_[6] = {};
  int front_ = 0, nextVortex_ = 0;
  Vec3 jetPosition_{0,0,0}, jetDown_{0,-1,0};
  float jet_ = 0.0f;
};

}  // namespace partsim
