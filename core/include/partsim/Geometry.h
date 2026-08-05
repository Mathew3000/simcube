#pragma once
#include "partsim/Config.h"
#include "partsim/Types.h"

namespace partsim {

// What a scene author writes.
struct PanelSpec {
  Vec3 center;   // world centre of the panel face
  Vec3 normal;   // INWARD, pointing into the sim volume
  Vec3 up;       // world direction the panel's +y should point
  uint16_t w, h; // texels
  float pitch;   // world units per texel
};

// Baked form, what the runtime uses.
//
// Orientation convention: panel +x is RIGHT and +y is UP as seen by an observer standing
// OUTSIDE the object looking at that panel. Picking "outside" rather than "inside" means
// the byte buffer the renderer emits is simultaneously correct for LED scan order and for
// a three.js texture -- no UV flip anywhere, and the same bytes drive both.
struct Panel {
  Vec3 origin;      // world position of the (0,0) texel *corner*
  Vec3 u, v;        // world step per +1 texel in x / y; |u| == |v| == pitch, u perp v
  Vec3 n;           // unit inward normal, == normalize(cross(u, v))
  float invU2;      // 1/|u|^2, precomputed so world->panel needs no division
  float invV2;
  uint16_t w, h;
};

// Result of projecting a world point onto a panel.
struct Proj {
  float s;     // texel x, fractional, may fall outside [0, w)
  float t;     // texel y
  float dist;  // signed distance along the inward normal; > 0 means inside the volume
};

// World -> panel. Exact (not a least-squares fit) because u and v are orthogonal, and
// division-free because the inverse squared lengths are baked. This is the inner loop of
// splatting, so it stays in the header.
inline Proj project(const Panel& p, Vec3 world) {
  const Vec3 d = world - p.origin;
  return Proj{dot(d, p.u) * p.invU2, dot(d, p.v) * p.invV2, dot(d, p.n)};
}

inline Vec3 texelCenter(const Panel& p, int i, int j) {
  return p.origin + p.u * ((float)i + 0.5f) + p.v * ((float)j + 0.5f);
}

class Geometry {
 public:
  void clear() { count_ = 0; }
  int count() const { return count_; }
  const Panel& at(int i) const { return panels_[i]; }

  // Bakes spec -> Panel. Returns false if the table is full or up is parallel to normal.
  bool addPanel(const PanelSpec& spec);

  // Total texels across all panels, i.e. the size of the concatenated output canvas.
  int totalTexels() const;

  // The physical container. Union of every panel quad, each also pushed inward by
  // slabDepthTexels so that a single flat panel gets a non-degenerate volume.
  //
  // Closed cube: the six quads already span the full box and the inward pushes land
  // strictly inside, so the result is exactly res^3.
  // Single panel: the quad alone is zero-thickness; the push gives it depth.
  // Deliberately NOT inflated -- padding belongs to the grid, not the container.
  Aabb bounds(float slabDepthTexels) const;

  // res^3 volume centred on the origin, six inward-facing panels.
  static Geometry cube(int res, float pitch);
  // One panel in the z = 0 plane facing +z, centred on the origin in x/y.
  static Geometry slab(int w, int h, float pitch);

 private:
  Panel panels_[kMaxPanels];
  int count_ = 0;
};

}  // namespace partsim
