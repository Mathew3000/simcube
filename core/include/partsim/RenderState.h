#pragma once
#include "partsim/Particles.h"
#include "partsim/SimVolume.h"

namespace partsim {

// What it takes to DRAW the simulation, separated from what it takes to RUN it.
//
// A display node in a multi-node cube receives state over the wire and splats it. It never
// integrates anything, so it has no business carrying the solver's working set -- and carrying it
// anyway is why a display node measured 244.6KB against a 230KB ceiling, with 55.7KB of the
// overshoot being pools it never touches:
//
//   Particles sx/sy/sz + lam   31.3KB   predicted positions and Lagrange multipliers
//   FieldGrid second buffer    10.3KB   ping-pong for advection; only cur_ is ever splatted
//   SpatialHash                10.5KB   a neighbour grid it never builds
//   Simulation::scratch_        3.6KB   the counting sort's scratch space
//
// The two views below are the seam. Renderer::splat and splatField take a view rather than a
// Particles or a FieldGrid, so ONE implementation serves both the full simulation and a
// receive-only node -- no templates, no virtual dispatch, no second copy of the splat loop to
// drift out of sync.

// Everything the particle splat reads, as plain contiguous arrays.
//
// Deliberately float, not the int16/int8 the wire carries. Dequantising once at decode costs
// 1280 conversions per frame; dequantising inside splat would repeat them per panel, and would
// put a branch in the hot loop. Storing floats is both simpler and faster, for 12KB.
struct ParticleView {
  const float* x;
  const float* y;
  const float* z;
  const float* vx;
  const float* vy;
  const float* vz;
  const uint8_t* mat;
  int n;
};

// Everything the heat splat reads. One buffer, because advection needs two and drawing needs one.
struct HeatView {
  const uint8_t* cells;  // dim.x * dim.y * dim.z, row-major with x fastest
  IVec3 dim;
  Vec3 lo;      // world position of the low corner of cell (0,0,0)
  float cell;   // world units per cell
  bool empty;   // nothing burning: the whole pass is skipped

  uint8_t atCoord(int x, int y, int z) const {
    return cells[(z * dim.y + y) * dim.x + x];
  }
  Vec3 cellCentre(int x, int y, int z) const {
    return Vec3{lo.x + ((float)x + 0.5f) * cell, lo.y + ((float)y + 0.5f) * cell,
                lo.z + ((float)z + 0.5f) * cell};
  }
};

// The heat grid's dimensions, derived in ONE place.
//
// FieldGrid and HeatBuffer must agree on this exactly or the master and a display node disagree
// about what a byte in the payload means -- and the symptom would be a plume that renders in the
// wrong place rather than an error. Sharing the derivation is what makes that impossible.
IVec3 heatGridDim(const SimVolume& v, float cellSize);
// The cell size the field actually uses. Half the smoothing radius; see FieldGrid.cpp.
float heatCellSize();

// Fixed-capacity particle state for a node that only draws.
//
// Positions, velocities and material: 25 bytes per particle against Particles' 41, because the
// predicted position and the Lagrange multiplier exist only for the solver's benefit.
class RenderParticles {
 public:
  void clear() { n_ = 0; }
  int count() const { return n_; }
  int capacity() const { return kMaxParticles; }

  bool add(Vec3 pos, Vec3 vel, uint8_t material) {
    if (n_ >= kMaxParticles) return false;
    const int i = n_++;
    x_[i] = pos.x; y_[i] = pos.y; z_[i] = pos.z;
    vx_[i] = vel.x; vy_[i] = vel.y; vz_[i] = vel.z;
    mat_[i] = material;
    return true;
  }

  ParticleView view() const {
    return ParticleView{x_, y_, z_, vx_, vy_, vz_, mat_, n_};
  }

 private:
  alignas(16) float x_[kMaxParticles];
  alignas(16) float y_[kMaxParticles];
  alignas(16) float z_[kMaxParticles];
  alignas(16) float vx_[kMaxParticles];
  alignas(16) float vy_[kMaxParticles];
  alignas(16) float vz_[kMaxParticles];
  uint8_t mat_[kMaxParticles];
  int n_ = 0;
};

// Single-buffer heat state for a node that only draws.
class HeatBuffer {
 public:
  // Dimensions come from the shared derivation, so they match whatever the master computed.
  bool init(const SimVolume& v);
  void clear();

  int cellCount() const { return cellCount_; }
  IVec3 dim() const { return dim_; }
  // Where the decoder writes. Exactly cellCount() bytes.
  uint8_t* cells() { return cells_; }
  const uint8_t* cells() const { return cells_; }
  // The decoder sets this so an all-cold frame skips the splat pass entirely, exactly as
  // FieldGrid's own peak tracking does.
  void setPeak(uint8_t peak) { peak_ = peak; }
  uint8_t peak() const { return peak_; }

  HeatView view() const {
    return HeatView{cells_, dim_, lo_, cell_, peak_ == 0};
  }

 private:
  uint8_t cells_[kMaxFieldCells];
  Vec3 lo_{0.0f, 0.0f, 0.0f};
  float cell_ = 1.0f;
  IVec3 dim_{1, 1, 1};
  int cellCount_ = 1;
  uint8_t peak_ = 0;
};

}  // namespace partsim
