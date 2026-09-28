#pragma once
#include "partsim/SurfaceWater.h"

namespace partsim {

struct DyeSample { float red, blue; };

// Two concentrations in one object-space 16^3 grid. Q0.16 concentrations
// retain weak wisps during slow advection. No pressure solve or heap storage.
class DyeField {
 public:
  static constexpr int kSize = 16;
  static constexpr int kCells = kSize*kSize*kSize;
  void reset(DyeSample colour);
  DyeSample sample(Vec3 p) const;
  DyeSample cell(int x, int y, int z) const;
  // Replace locally by incoming coloured liquid, not a cube-wide average.
  void inject(Vec3 position, DyeSample colour, float amount, float radius = 0.12f);
  void stir(Vec3 position, Vec3 down, float strength);
  void step(float dt, const SurfaceWater& water, Vec3 bulkVelocity);

 private:
  struct Vortex { Vec3 position, axis; float strength; };
  uint16_t grid_[3][kCells][2] = {};
  DyeSample sampleBuffer(int buffer, Vec3 p) const;
  Vec3 displacement(Vec3 p, float dt, Vec3 down, Vec3 bulkVelocity) const;
  Vortex vortices_[6] = {};
  int front_ = 0, nextVortex_ = 0;
  Vec3 jetPosition_{0,0,0}, jetDown_{0,-1,0};
  float jet_ = 0.0f;
};

}  // namespace partsim
