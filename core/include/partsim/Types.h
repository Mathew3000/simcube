#pragma once
#include <cstdint>

#include "partsim/Math.h"

namespace partsim {

struct Vec3 {
  float x, y, z;
};

inline Vec3 vec3(float x, float y, float z) { return Vec3{x, y, z}; }

inline Vec3 operator+(Vec3 a, Vec3 b) { return Vec3{a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return Vec3{a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator-(Vec3 a) { return Vec3{-a.x, -a.y, -a.z}; }
inline Vec3 operator*(Vec3 a, float s) { return Vec3{a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator*(float s, Vec3 a) { return a * s; }
inline Vec3& operator+=(Vec3& a, Vec3 b) { a = a + b; return a; }
inline Vec3& operator-=(Vec3& a, Vec3 b) { a = a - b; return a; }
inline Vec3& operator*=(Vec3& a, float s) { a = a * s; return a; }

inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
  return Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float length2(Vec3 a) { return dot(a, a); }
inline float length(Vec3 a) { return psqrt(dot(a, a)); }

inline Vec3 normalize(Vec3 a) {
  const float l2 = dot(a, a);
  if (l2 <= 1e-20f) return Vec3{0.0f, 0.0f, 0.0f};
  return a * prsqrt(l2);
}

struct IVec3 {
  int x, y, z;
};

struct Aabb {
  Vec3 lo, hi;

  static Aabb empty() {
    return Aabb{Vec3{1e30f, 1e30f, 1e30f}, Vec3{-1e30f, -1e30f, -1e30f}};
  }
  void expand(Vec3 p) {
    lo.x = pmin(lo.x, p.x); lo.y = pmin(lo.y, p.y); lo.z = pmin(lo.z, p.z);
    hi.x = pmax(hi.x, p.x); hi.y = pmax(hi.y, p.y); hi.z = pmax(hi.z, p.z);
  }
  void inflate(float m) {
    lo.x -= m; lo.y -= m; lo.z -= m;
    hi.x += m; hi.y += m; hi.z += m;
  }
  Vec3 size() const { return hi - lo; }
  Vec3 center() const { return (lo + hi) * 0.5f; }
  bool contains(Vec3 p) const {
    return p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y && p.z >= lo.z &&
           p.z <= hi.z;
  }
};

// Quaternion, used only at the platform boundary to turn an orientation into an
// object-space gravity vector. The core itself never sees orientation.
struct Quat {
  float x, y, z, w;
};

inline Vec3 rotateByConjugate(Quat q, Vec3 v) {
  // conj(q) * v * q  -- i.e. world vector -> object space.
  const Quat c{-q.x, -q.y, -q.z, q.w};
  const Vec3 u{c.x, c.y, c.z};
  const Vec3 t = cross(u, v) * 2.0f;
  return v + t * c.w + cross(u, t);
}

}  // namespace partsim
