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

#if PARTSIM_NEIGHBOUR_CACHE
  buildNeighbours(v, p);
#endif
  return true;
}

#if PARTSIM_NEIGHBOUR_CACHE
// One 27-cell scan per particle, keeping only what is actually within the smoothing radius, in
// gather order. The solver then walks these lists instead of rescanning, which is four of the five
// scans a step used to cost.
//
// What this changes about the answer, stated plainly: the correction pass is Gauss-Seidel, so it
// moves predicted positions as it goes, and a live re-gather on the SECOND iteration would see a
// slightly different candidate set for any particle that had crossed a cell boundary. A cached
// list freezes the neighbourhood at the start of the step. That is the same class of approximation
// as the grid itself already being stale for the whole step -- and with one solver iteration the
// two are bit-identical, because nothing has moved yet.
//
// Order is preserved exactly: cells ascending in flatten() order, particles ascending within a
// cell, which is what the cross-target determinism rests on.
void SpatialHash::buildNeighbours(const SimVolume& v, const Particles& p) {
  const float h2 = kNeighbourRadius * kNeighbourRadius;
  const int n = p.n;
  truncated_ = 0;
  for (int i = 0; i < n; ++i) {
    uint16_t* out = list_ + (unsigned)i * (unsigned)kMaxNeighbours;
    const Vec3 pi = p.pred(i);
    int c = 0;
    bool full = false;
    forEachNeighbour(v, *this, pi, [&](int j) {
      if (j == i) return;
      if (length2(p.pred(j) - pi) >= h2) return;
      if (c >= kMaxNeighbours) { full = true; return; }
      out[c++] = (uint16_t)j;
    });
    count_[i] = (uint8_t)c;
    if (full) ++truncated_;
  }
}
#endif

}  // namespace partsim
