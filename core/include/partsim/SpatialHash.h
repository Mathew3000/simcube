#pragma once
#include <cstddef>

#include "partsim/Parallel.h"

#include "partsim/Particles.h"
#include "partsim/SimVolume.h"

namespace partsim {

// Uniform-grid neighbour search by counting sort.
//
// Not a hash: the container is a known AABB, so the bucket index is a direct
// flatten(coordOf(p)) -- no modulo, no collisions, no probing.
//
// Not linked-list buckets either. head[cell] + next[particle] is two arrays and no sort,
// but the gather then chases pointers to random addresses, costing roughly a cache miss
// per neighbour; with ~33 neighbours x 3 iterations that dominates the frame. Counting
// sort plus a full SoA permutation makes the gather stream almost linearly instead.
//
// Neighbour lists ARE cached, since PARTSIM_NEIGHBOUR_CACHE. The M3 plan rejected them at
// 384KB, but that was 4096 particles; hardware measurement put the real budget at a few hundred
// (docs/RESOURCES.md section 5.1) and the same structure is a tenth of the size. See
// buildNeighbours() below for what the cache changes about the answer.
class SpatialHash {
 public:
  // Buckets by PREDICTED position and permutes every particle array into cell order.
  // scratch must be at least kMaxParticles * 4 bytes.
  // Returns false if the volume has more cells than the compiled capacity.
  // `par` splits the neighbour-list build only. The counting sort and the permutation stay serial:
  // the sort is a prefix sum over cells, which is inherently sequential, and the permutation writes
  // through an index vector where two workers could collide. The build is the expensive part -- it
  // is a full 27-cell scan per particle, the single largest gather in a step -- and it writes only
  // list_[i] and count_[i], so it satisfies the Parallel.h contract.
  bool build(const SimVolume& v, Particles& p, void* scratch, Parallel* par = nullptr);

  int cellCount() const { return cellCount_; }

#if PARTSIM_NEIGHBOUR_CACHE
  // Neighbours of particle i within kSmoothRadius, in gather order, self excluded. Valid until
  // the next build(); indices are into the PERMUTED arrays, which build() has already produced.
  const ParticleIndex* neighbours(int i) const {
    return list_ + (std::size_t)i * (std::size_t)kMaxNeighbours;
  }
  int neighbourCount(int i) const { return (int)count_[i]; }
  // Particles whose list hit kMaxNeighbours and lost their furthest neighbours. Zero across every
  // scene as configured; a non-zero value here is the signal to raise the cap, not a fault.
  int truncated() const { return truncated_; }
#endif
  // After build(), particles of cell `flat` occupy [begin, end) in the arrays directly.
  int cellBegin(int flat) const { return (int)start_[flat]; }
  int cellEnd(int flat) const { return (int)start_[flat + 1]; }

 private:
#if PARTSIM_NEIGHBOUR_CACHE
  void buildNeighbours(const SimVolume& v, const Particles& p, Parallel& par);
  void buildNeighbourRange(const SimVolume& v, const Particles& p, int begin, int end);
#endif

  int cellCount_ = 0;
  // start_[c] is the first slot of cell c; start_[cellCount_] == n. cellCount_+1 entries.
  ParticleIndex start_[kMaxGridCells + 1];
  ParticleIndex idx_[kMaxParticles];
#if PARTSIM_NEIGHBOUR_CACHE
  ParticleIndex list_[(std::size_t)kMaxParticles * (std::size_t)kMaxNeighbours];
  uint8_t count_[kMaxParticles];
  int truncated_ = 0;
#endif
};

// Visit every particle index in the 27 cells touching `p`. Iteration order is a pure
// function of positions -- cells in ascending flatten() order, particles ascending within
// a cell -- which is what keeps the solver bit-deterministic across targets.
template <class F>
inline void forEachNeighbour(const SimVolume& v, const SpatialHash& h, Vec3 p, F&& fn) {
  const IVec3 c = v.coordOf(p);
  const IVec3 d = v.dim();
  const int z0 = imax(0, c.z - 1), z1 = imin(d.z - 1, c.z + 1);
  const int y0 = imax(0, c.y - 1), y1 = imin(d.y - 1, c.y + 1);
  const int x0 = imax(0, c.x - 1), x1 = imin(d.x - 1, c.x + 1);

  for (int z = z0; z <= z1; ++z) {
    for (int y = y0; y <= y1; ++y) {
      // Cells along x are contiguous in flatten() order, so one index and a walk.
      const int base = (z * d.y + y) * d.x;
      const int begin = h.cellBegin(base + x0);
      const int end = h.cellEnd(base + x1);
      for (int j = begin; j < end; ++j) fn(j);
    }
  }
}

// Visit the neighbours of particle i that are within the smoothing radius -- from the cache when
// there is one, and otherwise by scanning, which is what makes PARTSIM_NEIGHBOUR_CACHE=0 a
// behaviour-preserving fallback rather than a second code path. Self is already excluded either
// way. Callers still test r2 against h2: a cached neighbour can drift outside the radius during
// the correction pass, and the gradient kernel is not defined out there.
template <class F>
inline void forEachNear(const SimVolume& v, const SpatialHash& h, const Particles& p, int i,
                        F&& fn) {
#if PARTSIM_NEIGHBOUR_CACHE
  (void)v;
  (void)p;
  const ParticleIndex* nb = h.neighbours(i);
  const int c = h.neighbourCount(i);
  for (int k = 0; k < c; ++k) fn((int)nb[k]);
#else
  forEachNeighbour(v, h, p.pred(i), [&](int j) {
    if (j != i) fn(j);
  });
#endif
}

}  // namespace partsim
