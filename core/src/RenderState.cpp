#include "partsim/RenderState.h"

#include "partsim/InkField.h"

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
#if !PARTSIM_ENABLE_HEAT
  // No storage and no grid. Matching the disabled field on the master matters: a node that
  // reported a grid size here would disagree with the frames it receives.
  (void)v;
  lo_ = Vec3{0.0f, 0.0f, 0.0f};
  cell_ = 0.0f;
  dim_ = IVec3{0, 0, 0};
  cellCount_ = 0;
  peak_ = 0;
  return true;
#else
  lo_ = v.box().lo;
  cell_ = heatCellSize();
  dim_ = heatGridDim(v, cell_);
  cellCount_ = dim_.x * dim_.y * dim_.z;
  if (cellCount_ > kMaxFieldCells) return false;
  clear();
  return true;
#endif
}

void HeatBuffer::clear() {
  for (int i = 0; i < cellCount_; ++i) cells_[i] = 0;
  peak_ = 0;
}

// --- adapters, so the full simulation feeds the same splat path -----------------------------

ParticleView Particles::view() const {
#if PARTSIM_ENABLE_CHROMA
  return ParticleView{x, y, z, vx, vy, vz, mat, cr, cg, n};
#else
  return ParticleView{x, y, z, vx, vy, vz, mat, n};
#endif
}

HeatView FieldGrid::view() const {
  return HeatView{cur_, dim_, lo_, cell_, peak_ == 0};
}

InkView InkField::view() const {
  InkView v;
  for (int c = 0; c < kInkChannels; ++c) v.dye[c] = dye_[cur_][c];
  v.dim = IVec3{kInkDim, kInkDim, kInkDim};
  v.lo = lo_;
  v.cell = cell_;
  v.empty = (peak_ == 0);
  return v;
}

}  // namespace partsim
