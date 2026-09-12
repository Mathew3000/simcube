#pragma once
#include "partsim/Geometry.h"

namespace partsim {

// Which of the container's six faces, if any, is absent.
//
// Beaker mode opens exactly one. An axis-and-side pair rather than a bitmask: a vessel with two
// open faces is not a vessel, and a mask would make "which way is up" a search rather than a
// lookup. Named by the outward direction, so kOpenPosY is the open TOP of an upright cube --
// which is the face BeakerOverlay classifies as kTop from the same normal.
enum OpenFace : int {
  kOpenNone = -1,
  kOpenNegX = 0,
  kOpenPosX = 1,
  kOpenNegY = 2,
  kOpenPosY = 3,
  kOpenNegZ = 4,
  kOpenPosZ = 5,
};

// The container AABB plus the uniform grid used for neighbour search.
//
// Because the container is a known box, the "spatial hash" is not a hash at all: the cell
// index is a direct flatten(coordOf(p)). No modulo, no collisions, no probing.
class SimVolume {
 public:
  // cellSize MUST be >= kSmoothRadius. If it is smaller, the 27-cell gather silently
  // misses neighbours, which shows up much later as inexplicable solver explosions rather
  // than as a search bug -- hence the hard check.
  bool build(const Geometry& g, float slabDepthWorld, float cellSize);

  const Aabb& box() const { return box_; }

  // --- the open face ---------------------------------------------------------------------
  //
  // kOpenNone by default, which is every build but beaker mode: with no open face every method
  // below reduces to exactly what it did when the box was unconditionally closed, so the golden
  // hashes cannot move. Pass one of the OpenFace values to cut a hole.
  void setOpenFace(int f) { open_ = (f >= kOpenNegX && f <= kOpenPosZ) ? f : (int)kOpenNone; }
  int openFace() const { return open_; }
  bool isOpen() const { return open_ != kOpenNone; }

  // Clamp a predicted position back into the container -- the ONE place that says "no particle
  // is ever outside this". Every face is a wall except the open one, which a particle passes
  // straight through.
  //
  // Lives here rather than in Solver.cpp because the box is what asserts the constraint, and the
  // solver calls it from three separate passes; a fourth caller that reached for the raw Aabb
  // would silently re-close the face.
  Vec3 clampInto(Vec3 p) const {
    return Vec3{clampAxis(p.x, box_.lo.x, box_.hi.x - kBoxSkin, 0),
                clampAxis(p.y, box_.lo.y, box_.hi.y - kBoxSkin, 1),
                clampAxis(p.z, box_.lo.z, box_.hi.z - kBoxSkin, 2)};
  }

  // Has this position left through the open face? False for a closed box, and false for a
  // particle that is merely outside on some other axis -- clampInto guarantees there are none,
  // and answering "yes" for one would turn a solver bug into a silent particle leak.
  bool pastOpenFace(Vec3 p) const {
    switch (open_) {
      case kOpenNegX: return p.x < box_.lo.x;
      case kOpenPosX: return p.x > box_.hi.x - kBoxSkin;
      case kOpenNegY: return p.y < box_.lo.y;
      case kOpenPosY: return p.y > box_.hi.y - kBoxSkin;
      case kOpenNegZ: return p.z < box_.lo.z;
      case kOpenPosZ: return p.z > box_.hi.z - kBoxSkin;
      default: return false;
    }
  }

  // Is there a wall on this side of this axis? The solver's wall-density compensation asks,
  // because an absent face hides no neighbours: see DECISIONS.md D57.
  //   axis 0/1/2 = x/y/z, high = the +face.
  bool hasWall(int axis, bool high) const { return open_ != (2 * axis + (high ? 1 : 0)); }
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

  // The open axis and the inward direction along it, for placing an arrival. Returns -1 when the
  // box is closed.
  int openAxis() const { return open_ == kOpenNone ? -1 : open_ / 2; }
  bool openIsHigh() const { return (open_ & 1) != 0; }

  bool inRange(IVec3 c) const {
    return c.x >= 0 && c.y >= 0 && c.z >= 0 && c.x < dim_.x && c.y < dim_.y && c.z < dim_.z;
  }

 private:
  // A hair inside the far edge so cell indexing never lands on the exclusive upper bound.
  static constexpr float kBoxSkin = 1e-3f;

  float clampAxis(float x, float lo, float hi, int axis) const {
    // The closed path must be pclamp and nothing else: it is what every non-beaker build runs,
    // and a differently-associated min/max here would move both golden hashes.
    if (axis != open_ / 2 || open_ == kOpenNone) return pclamp(x, lo, hi);
    return (open_ & 1) ? pmax(x, lo) : pmin(x, hi);
  }

  int open_ = kOpenNone;
  Aabb box_ = Aabb::empty();
  float cell_ = 1.0f;
  float invCell_ = 1.0f;
  IVec3 dim_{1, 1, 1};
  int cellCount_ = 1;
};

}  // namespace partsim
