#include "partsim/SpatialHash.h"

namespace partsim {

bool SpatialHash::build(const SimVolume& v, Particles& p, void* scratch, Parallel* par) {
  cellCount_ = v.cellCount();
  if (cellCount_ > kMaxGridCells) return false;

  const int n = p.n;
  for (int c = 0; c < cellCount_; ++c) start_[c] = 0;
  start_[cellCount_] = (ParticleIndex)n;
  if (n == 0) return true;

  // Pass 1: per-cell counts.
  for (int i = 0; i < n; ++i) ++start_[v.cellIndexOf(p.pred(i))];

  // Pass 2: inclusive prefix sum, so start_[c] is now the END offset of cell c.
  ParticleIndex sum = 0;
  for (int c = 0; c < cellCount_; ++c) {
    sum = (ParticleIndex)(sum + start_[c]);
    start_[c] = sum;
  }
  start_[cellCount_] = (ParticleIndex)n;

  // Pass 3: place by pre-decrementing, which walks each cell's slots backwards and leaves
  // start_[c] holding the cell's BEGIN offset -- so no separate cursor array is needed and
  // start_[c+1] is exactly cell c's end. Iterating i downwards makes the indices within a
  // cell come out ascending, which the deterministic gather order depends on.
  for (int i = n - 1; i >= 0; --i) {
    const int c = v.cellIndexOf(p.pred(i));
    idx_[--start_[c]] = (ParticleIndex)i;
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
#if PARTSIM_ENABLE_CHROMA
  // Chroma permutes with everything else. Miss it and the dye detaches from the particle carrying
  // it -- which does not crash, does not move the state hash, and shows up only as colour that
  // smears across the fluid a little more each step.
  uint16_t* ws = (uint16_t*)scratch;  // chroma is 16-bit; the byte scratch above will not hold it
  for (int k = 0; k < n; ++k) ws[k] = p.cr[idx_[k]];
  for (int k = 0; k < n; ++k) p.cr[k] = ws[k];
  for (int k = 0; k < n; ++k) ws[k] = p.cg[idx_[k]];
  for (int k = 0; k < n; ++k) p.cg[k] = ws[k];
#endif

#if PARTSIM_NEIGHBOUR_CACHE
  buildNeighbours(v, p, par ? *par : serialParallel());
#else
  (void)par;
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
void SpatialHash::buildNeighbours(const SimVolume& v, const Particles& p, Parallel& par) {
  truncated_ = 0;
  struct Ctx {
    SpatialHash* self;
    const SimVolume* v;
    const Particles* p;
  } ctx{this, &v, &p};
  par.forRange(p.n, &ctx, [](void* vp, int begin, int end) {
    Ctx& c = *(Ctx*)vp;
    c.self->buildNeighbourRange(*c.v, *c.p, begin, end);
  });
}

void SpatialHash::buildNeighbourRange(const SimVolume& v, const Particles& p, int begin, int end) {
  const float h2 = kNeighbourRadius * kNeighbourRadius;
  for (int i = begin; i < end; ++i) {
    ParticleIndex* out = list_ + (size_t)i * (size_t)kMaxNeighbours;
    const Vec3 pi = p.pred(i);
    int c = 0;
    bool full = false;
    forEachNeighbour(v, *this, pi, [&](int j) {
      if (j == i) return;
      if (length2(p.pred(j) - pi) >= h2) return;
      if (c >= kMaxNeighbours) { full = true; return; }
      out[c++] = (ParticleIndex)j;
    });
    count_[i] = (uint8_t)c;
    // Diagnostic only, and deliberately not synchronised: a lost increment under concurrency
    // understates a count that is zero in every shipping configuration, and the alternative is an
    // atomic in the hottest loop in the project to protect a number nobody reads at speed.
    if (full) ++truncated_;
  }
}
#endif

}  // namespace partsim
