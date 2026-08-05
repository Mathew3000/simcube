#pragma once
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
// Neighbour lists are deliberately NOT cached: 48 neighbours x 2B x 4096 particles is
// 384KB, more than the entire ESP32 budget. The grid is built once per step and re-gathered
// each solver iteration.
class SpatialHash {
 public:
  // Buckets by PREDICTED position and permutes every particle array into cell order.
  // scratch must be at least kMaxParticles * 4 bytes.
  // Returns false if the volume has more cells than the compiled capacity.
  bool build(const SimVolume& v, Particles& p, void* scratch);

  int cellCount() const { return cellCount_; }
  // After build(), particles of cell `flat` occupy [begin, end) in the arrays directly.
  int cellBegin(int flat) const { return (int)start_[flat]; }
  int cellEnd(int flat) const { return (int)start_[flat + 1]; }

 private:
  int cellCount_ = 0;
  // start_[c] is the first slot of cell c; start_[cellCount_] == n. cellCount_+1 entries.
  uint16_t start_[kMaxGridCells + 1];
  uint16_t idx_[kMaxParticles];
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

}  // namespace partsim
