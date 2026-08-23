#include "partsim/SimVolume.h"

#include "partsim/Config.h"

namespace partsim {

bool SimVolume::build(const Geometry& g, float slabDepthWorld, float cellSize) {
  if (g.count() == 0) return false;
  if (cellSize < kSmoothRadius) return false;  // see the header comment; not negotiable

  box_ = g.bounds(slabDepthWorld);
  cell_ = cellSize;
  invCell_ = 1.0f / cellSize;

  const Vec3 s = box_.size();
  // Ceil, so the last (partial) cell still exists; coordOf clamps into it.
  dim_.x = imax(1, (int)(s.x * invCell_) + ((s.x * invCell_ > (float)(int)(s.x * invCell_)) ? 1 : 0));
  dim_.y = imax(1, (int)(s.y * invCell_) + ((s.y * invCell_ > (float)(int)(s.y * invCell_)) ? 1 : 0));
  dim_.z = imax(1, (int)(s.z * invCell_) + ((s.z * invCell_ > (float)(int)(s.z * invCell_)) ? 1 : 0));

  cellCount_ = dim_.x * dim_.y * dim_.z;
  return cellCount_ <= kMaxGridCells;
}

}  // namespace partsim
