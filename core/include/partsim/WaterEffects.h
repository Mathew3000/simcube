#pragma once
#include "partsim/DyeField.h"

namespace partsim {

// Optional visual layer around the small, volume-correct surface model.
// Fixed-capacity droplets carry real volume; dye is an approximate visual field.
class WaterEffects {
 public:
  static constexpr int kDrops = 32;
  void reset(float volume, DyeSample colour);
  void step(float dt, Vec3 down);
  void impulse(Vec3 accelerationG);
  float pourTo(WaterEffects& receiver, float dt);
  void addInk(DyeSample colour);
  void recolour(DyeSample colour) { dye_.reset(colour); }
  void renderPanel(const Panel& panel, float cubeSide, uint8_t* rgba) const;
  float volume() const;
  float airborneVolume() const;
  int dropletCount() const;
  const SurfaceWater& surface() const { return surface_; }
  const DyeField& dye() const { return dye_; }

 private:
  struct Droplet { Vec3 p, v; DyeSample dye; float volume, age; };
  SurfaceWater surface_;
  DyeField dye_;
  Droplet drops_[kDrops] = {};
  Vec3 lastDown_{0,-1,0}, flow_{0,0,0};
  float dyeTime_ = 0, splashCooldown_ = 0, stirCooldown_ = 0;
  unsigned spawnSerial_ = 0;
  bool seeded_ = false;
  Vec3 inlet() const;
  void splash(Vec3 direction, float strength);
};
}  // namespace partsim
