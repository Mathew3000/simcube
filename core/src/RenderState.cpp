#include "partsim/RenderState.h"

#include "partsim/FieldGrid.h"

namespace partsim {
namespace {

// Ceil without <cmath>, matching FieldGrid.cpp's local helper. The two must round identically or
// the grids differ by a cell at some box sizes.
inline int ceilDiv(float a, float b) {
  const float q = a / b;
  const int i = (int)q;
  return (q > (float)i) ? i + 1 : i;
}

}  // namespace

float heatCellSize() { return kSmoothRadius * 0.5f; }

IVec3 heatGridDim(const SimVolume& v, float cellSize) {
  const Vec3 s = v.box().size();
  return IVec3{imax(1, ceilDiv(s.x, cellSize)), imax(1, ceilDiv(s.y, cellSize)),
               imax(1, ceilDiv(s.z, cellSize))};
}

bool HeatBuffer::init(const SimVolume& v) {
  lo_ = v.box().lo;
  cell_ = heatCellSize();
  dim_ = heatGridDim(v, cell_);
  cellCount_ = dim_.x * dim_.y * dim_.z;
  if (cellCount_ > kMaxFieldCells) return false;
  clear();
  return true;
}

void HeatBuffer::clear() {
  for (int i = 0; i < cellCount_; ++i) cells_[i] = 0;
  peak_ = 0;
}

// --- adapters, so the full simulation feeds the same splat path -----------------------------

ParticleView Particles::view() const {
  return ParticleView{x, y, z, vx, vy, vz, mat, n};
}

HeatView FieldGrid::view() const {
  return HeatView{cur_, dim_, lo_, cell_, peak_ == 0};
}

}  // namespace partsim
