#include "partsim/SpatialHash.h"

namespace partsim {

bool SpatialHash::build(const SimVolume& v, Particles& p, void* scratch) {
  cellCount_ = v.cellCount();
  if (cellCount_ > kMaxGridCells) return false;

  const int n = p.n;
  for (int c = 0; c < cellCount_; ++c) start_[c] = 0;
  start_[cellCount_] = (uint16_t)n;
  if (n == 0) return true;

  // Pass 1: per-cell counts.
  for (int i = 0; i < n; ++i) ++start_[v.cellIndexOf(p.pred(i))];

  // Pass 2: inclusive prefix sum, so start_[c] is now the END offset of cell c.
  uint16_t sum = 0;
  for (int c = 0; c < cellCount_; ++c) {
    sum = (uint16_t)(sum + start_[c]);
    start_[c] = sum;
  }
  start_[cellCount_] = (uint16_t)n;

  // Pass 3: place by pre-decrementing, which walks each cell's slots backwards and leaves
  // start_[c] holding the cell's BEGIN offset -- so no separate cursor array is needed and
  // start_[c+1] is exactly cell c's end. Iterating i downwards makes the indices within a
  // cell come out ascending, which the deterministic gather order depends on.
  for (int i = n - 1; i >= 0; --i) {
    const int c = v.cellIndexOf(p.pred(i));
    idx_[--start_[c]] = (uint16_t)i;
  }

  // Pass 4: permute every array into cell order so the neighbour gather streams.
  float* fs = (float*)scratch;
  float* const arrays[10] = {p.x, p.y, p.z, p.vx, p.vy, p.vz, p.sx, p.sy, p.sz, p.lam};
  for (int a = 0; a < 10; ++a) {
    float* arr = arrays[a];
    for (int k = 0; k < n; ++k) fs[k] = arr[idx_[k]];
    for (int k = 0; k < n; ++k) arr[k] = fs[k];
  }
  uint8_t* bs = (uint8_t*)scratch;
  for (int k = 0; k < n; ++k) bs[k] = p.mat[idx_[k]];
  for (int k = 0; k < n; ++k) p.mat[k] = bs[k];

  return true;
}

}  // namespace partsim
