#include "partsim/Geometry.h"

namespace partsim {

bool Geometry::addPanel(const PanelSpec& spec) {
  if (count_ >= kMaxPanels) return false;
  if ((int)spec.w * (int)spec.h > kMaxPanelTexels) return false;

  const Vec3 n = normalize(spec.normal);

  // u = up x n, then re-derive v from n and u. Re-deriving v rather than using `up`
  // directly means a slightly non-perpendicular `up` is silently orthogonalised instead
  // of producing a skewed basis.
  Vec3 uHat = cross(normalize(spec.up), n);
  if (length2(uHat) < 1e-8f) return false;  // up parallel to normal: no valid basis
  uHat = normalize(uHat);
  const Vec3 vHat = cross(n, uHat);

  Panel& p = panels_[count_];
  p.u = uHat * spec.pitch;
  p.v = vHat * spec.pitch;
  p.n = n;
  p.invU2 = 1.0f / length2(p.u);
  p.invV2 = 1.0f / length2(p.v);
  p.w = spec.w;
  p.h = spec.h;
  p.origin = spec.center - p.u * (0.5f * (float)spec.w) - p.v * (0.5f * (float)spec.h);
  ++count_;
  return true;
}

int Geometry::totalTexels() const {
  int n = 0;
  for (int i = 0; i < count_; ++i) n += (int)panels_[i].w * (int)panels_[i].h;
  return n;
}

Aabb Geometry::bounds(float slabDepthTexels) const {
  Aabb box = Aabb::empty();
  for (int i = 0; i < count_; ++i) {
    const Panel& p = panels_[i];
    const Vec3 du = p.u * (float)p.w;
    const Vec3 dv = p.v * (float)p.h;
    const Vec3 corners[4] = {p.origin, p.origin + du, p.origin + dv, p.origin + du + dv};
    // Depth is in texels, and |u| == pitch, so scale by the pitch to reach world units.
    const Vec3 push = p.n * (slabDepthTexels * length(p.u));
    for (int c = 0; c < 4; ++c) {
      box.expand(corners[c]);
      box.expand(corners[c] + push);
    }
  }
  return box;
}

Geometry Geometry::cube(int res, float pitch) {
  const float half = 0.5f * (float)res * pitch;
  const uint16_t r = (uint16_t)res;

  // Inward normal, then an `up` choice per face. For the top and bottom faces `up` cannot
  // be +Y, so they use +Z / -Z respectively (the usual cube-map convention).
  const PanelSpec specs[6] = {
      {Vec3{0, 0, -half}, Vec3{0, 0, 1},  Vec3{0, 1, 0},  r, r, pitch},  // -Z
      {Vec3{0, 0, half},  Vec3{0, 0, -1}, Vec3{0, 1, 0},  r, r, pitch},  // +Z
      {Vec3{-half, 0, 0}, Vec3{1, 0, 0},  Vec3{0, 1, 0},  r, r, pitch},  // -X
      {Vec3{half, 0, 0},  Vec3{-1, 0, 0}, Vec3{0, 1, 0},  r, r, pitch},  // +X
      {Vec3{0, -half, 0}, Vec3{0, 1, 0},  Vec3{0, 0, -1}, r, r, pitch},  // -Y (bottom)
      {Vec3{0, half, 0},  Vec3{0, -1, 0}, Vec3{0, 0, 1},  r, r, pitch},  // +Y (top)
  };

  Geometry g;
  for (int i = 0; i < 6; ++i) g.addPanel(specs[i]);
  return g;
}

Geometry Geometry::slab(int w, int h, float pitch) {
  Geometry g;
  g.addPanel(PanelSpec{Vec3{0, 0, 0}, Vec3{0, 0, 1}, Vec3{0, 1, 0}, (uint16_t)w,
                       (uint16_t)h, pitch});
  return g;
}

}  // namespace partsim
