#pragma once
#include "partsim/Geometry.h"

namespace partsim {

// The container AABB plus the uniform grid used for neighbour search.
//
// Because the container is a known box, the "spatial hash" is not a hash at all: the cell
// index is a direct flatten(coordOf(p)). No modulo, no collisions, no probing.
class SimVolume {
 public:
  // cellSize MUST be >= kSmoothRadius. If it is smaller, the 27-cell gather silently
  // misses neighbours, which shows up much later as inexplicable solver explosions rather
  // than as a search bug -- hence the hard check.
  bool build(const Geometry& g, float slabDepthTexels, float cellSize);

  const Aabb& box() const { return box_; }
  float cellSize() const { return cell_; }
  IVec3 dim() const { return dim_; }
  int cellCount() const { return cellCount_; }

  IVec3 coordOf(Vec3 p) const {
    return IVec3{iclamp((int)((p.x - box_.lo.x) * invCell_), 0, dim_.x - 1),
                 iclamp((int)((p.y - box_.lo.y) * invCell_), 0, dim_.y - 1),
                 iclamp((int)((p.z - box_.lo.z) * invCell_), 0, dim_.z - 1)};
  }

  int flatten(IVec3 c) const { return (c.z * dim_.y + c.y) * dim_.x + c.x; }
  int cellIndexOf(Vec3 p) const { return flatten(coordOf(p)); }

  bool inRange(IVec3 c) const {
    return c.x >= 0 && c.y >= 0 && c.z >= 0 && c.x < dim_.x && c.y < dim_.y && c.z < dim_.z;
  }

 private:
  Aabb box_ = Aabb::empty();
  float cell_ = 1.0f;
  float invCell_ = 1.0f;
  IVec3 dim_{1, 1, 1};
  int cellCount_ = 1;
};

}  // namespace partsim
