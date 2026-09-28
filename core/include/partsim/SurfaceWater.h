#pragma once
#include "partsim/Geometry.h"

namespace partsim {

// Lightweight, allocation-free water in a unit cube [-0.5, 0.5]^3.
// Volume is a fraction of capacity, independent of LED resolution. The +Y face
// is the virtual opening; all six physical LED faces can still be rendered.
// This is a visual slosh model, not a pressure/particle fluid solver.
class SurfaceWater {
 public:
  void reset(float volume = 0.5f);
  void setVolume(float volume);
  float volume() const { return volume_; }
  Vec3 up() const { return up_; }
  float level() const { return level_; }

  // Object-space unit gravity direction and container acceleration in g.
  // A zero gravity vector keeps the last target. dt is bounded to 100 ms;
  // callers should use a fixed step and drop excess time after a long pause.
  void step(float dt, Vec3 down, Vec3 accelerationG = {0, 0, 0});
  void impulse(Vec3 accelerationG);

  // Desired outgoing capacity-fraction / second at the +Y opening.
  float outflowRate() const;
  // In-process reference transfer. Receiver-full policy is backpressure:
  // only accepted volume leaves the source. Network transport is separate.
  float pourTo(SurfaceWater& receiver, float dt);

  // Fraction below dot(unitUp, position) == level, evaluated analytically.
  // Exposed for volume/orientation verification and future snapshot consumers.
  static float volumeBelow(Vec3 unitUp, float level);
  float depth(Vec3 unitPosition) const { return level_ - dot(up_, unitPosition); }

  // CPU face shader. Pixel coordinates use the project's existing Panel basis.
  // cubeSide converts that geometry to the unit cube (normally 32 world units).
  // Caller owns w*h*4 bytes; no particle pools or framebuffer are owned here.
  void renderPanel(const Panel& panel, float cubeSide, uint8_t* rgba) const;

 private:
  void solveLevel();
  float volume_ = 0.5f;
  float level_ = 0.0f;
  Vec3 up_{0, 1, 0};
  Vec3 targetUp_{0, 1, 0};
  Vec3 velocity_{0, 0, 0};
  float ripple_ = 0.0f;
  float rippleVelocity_ = 0.0f;
  float phase_ = 0.0f;
};

}  // namespace partsim
