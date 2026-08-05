#pragma once
#include "partsim/Rng.h"
#include "partsim/SimVolume.h"

namespace partsim {

// A heat emitter, expressed in normalised box coordinates (0..1 on each axis) so a scene
// preset does not have to know the volume's size.
struct Emitter {
  float nx, ny, nz;  // position, 0..1 within the box
  float radius;      // world units
  float rate;        // heat added per second, 0..1 scale
};

// Heat and smoke as a coarse uint8 grid rather than particles.
//
// Fire has no surface and no incompressibility, so the whole PBF machinery buys nothing for it:
// what reads as flame is a buoyant scalar field advecting upward and cooling. One byte per cell
// at half the sim resolution costs a few KB and a single semi-Lagrangian pass, against ~5500
// cycles per particle. Water gets particles because its surface is the whole point; fire does
// not because it has none.
class FieldGrid {
 public:
  bool init(const SimVolume& v);

  void clear();

  // Semi-Lagrangian advection along an analytic velocity: buoyancy opposite gravity, plus the
  // container's acceleration, plus a little curl-ish swirl so a plume is not a straight column.
  void step(const SimVolume& v, Vec3 gravity, Vec3 containerAccel, const Emitter* emitters,
            int emitterCount, float dt, Rng& rng);

  IVec3 dim() const { return dim_; }
  int cellCount() const { return cellCount_; }
  float cellSize() const { return cell_; }
  bool empty() const { return peak_ == 0; }

  uint8_t at(int i) const { return cur_[i]; }
  uint8_t atCoord(int x, int y, int z) const { return cur_[(z * dim_.y + y) * dim_.x + x]; }
  Vec3 cellCentre(int x, int y, int z) const {
    return Vec3{lo_.x + ((float)x + 0.5f) * cell_, lo_.y + ((float)y + 0.5f) * cell_,
                lo_.z + ((float)z + 0.5f) * cell_};
  }

  // Trilinear sample in world space, 0..1. Used by the renderer.
  float sample(Vec3 world) const;

 private:
  float sampleCur(float fx, float fy, float fz) const;

  uint8_t a_[kMaxFieldCells];
  uint8_t b_[kMaxFieldCells];
  uint8_t* cur_ = a_;
  uint8_t* nxt_ = b_;

  Vec3 lo_{0.0f, 0.0f, 0.0f};
  float cell_ = 1.0f;
  float invCell_ = 1.0f;
  IVec3 dim_{1, 1, 1};
  int cellCount_ = 1;
  int peak_ = 0;    // cheap "is there anything burning" flag, so idle scenes skip the pass
  float time_ = 0.0f;  // drives the swirl phase; pure function of step count, so deterministic
};

}  // namespace partsim
