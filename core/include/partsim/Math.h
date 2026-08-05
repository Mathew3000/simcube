#pragma once
// The ONLY file under core/ permitted to include <cmath>, and then only for sqrtf.
//
// Why: cross-target bit-determinism. IEEE-754 requires sqrt to be correctly rounded, so
// sqrtf is identical on newlib (ESP32), musl (Emscripten) and libSystem (macOS). The
// transcendentals are NOT specified to any particular accuracy, so sinf/cosf/expf/powf
// differ in the last bits between those three libms -- enough to diverge a PBF trajectory
// within a few hundred steps. Anything transcendental therefore gets a polynomial
// approximation here, evaluated identically everywhere.
//
// tests/test_math.cpp checks these against libm; scripts/check_no_libm.sh enforces that no
// other core file reaches for <cmath>.
#include <cmath>

namespace partsim {

inline float psqrt(float x) { return sqrtf(x); }
inline float pabs(float x) { return x < 0.0f ? -x : x; }
inline float pmin(float a, float b) { return a < b ? a : b; }
inline float pmax(float a, float b) { return a > b ? a : b; }
inline float pclamp(float x, float lo, float hi) { return pmin(pmax(x, lo), hi); }

inline int imin(int a, int b) { return a < b ? a : b; }
inline int imax(int a, int b) { return a > b ? a : b; }
inline int iclamp(int x, int lo, int hi) { return imin(imax(x, lo), hi); }

// Reciprocal sqrt. Prefer x * prsqrt(d2) over x / psqrt(d2): one Newton-Raphson chain
// instead of a sqrt chain followed by a ~20-cycle Xtensa division sequence.
inline float prsqrt(float x) { return 1.0f / sqrtf(x); }

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 6.28318530717958647692f;

// ---------------------------------------------------------------------------
// Polynomial transcendentals. Deterministic by construction: fixed sequence of
// multiplies and adds, no libm, no table lookups.
// ---------------------------------------------------------------------------

// Sine, ~1.5e-6 max abs error over all reals (argument reduction included).
// Minimax degree-9 odd polynomial on [-pi/2, pi/2], quadrant-folded.
inline float fsin(float x) {
  // Reduce to [-pi, pi] without fmodf (also libm).
  const float inv2pi = 0.15915494309189535f;
  float k = (float)(int)(x * inv2pi + (x >= 0.0f ? 0.5f : -0.5f));
  x -= k * kTwoPi;
  // Fold [-pi,-pi/2] and [pi/2,pi] into [-pi/2,pi/2] via sin(pi-x) == sin(x).
  if (x > 0.5f * kPi) x = kPi - x;
  else if (x < -0.5f * kPi) x = -kPi - x;

  const float x2 = x * x;
  float r = -2.5051132068021698e-8f;
  r = r * x2 + 2.7557314297196670e-6f;
  r = r * x2 - 1.9841269659586505e-4f;
  r = r * x2 + 8.3333333332845893e-3f;
  r = r * x2 - 1.6666666666666490e-1f;
  return x + x * x2 * r;
}

inline float fcos(float x) { return fsin(x + 0.5f * kPi); }

// exp(x) via 2^k * exp(r), r in [-ln2/2, ln2/2]. Max rel error ~1e-6 on [-20, 20].
inline float fexp(float x) {
  if (x > 88.0f) return 3.4028235e38f;   // saturate rather than produce inf
  if (x < -88.0f) return 0.0f;

  const float kLog2e = 1.4426950408889634f;
  const float kLn2 = 0.6931471805599453f;
  int k = (int)(x * kLog2e + (x >= 0.0f ? 0.5f : -0.5f));
  float r = x - (float)k * kLn2;

  // Degree-6 Taylor on the reduced range; plenty at |r| <= 0.347.
  float p = 1.0f / 720.0f;
  p = p * r + 1.0f / 120.0f;
  p = p * r + 1.0f / 24.0f;
  p = p * r + 1.0f / 6.0f;
  p = p * r + 0.5f;
  p = p * r + 1.0f;
  p = p * r + 1.0f;

  // Scale by 2^k by assembling the exponent field directly -- no ldexpf (libm).
  union { float f; unsigned int u; } s;
  s.u = (unsigned int)(127 + k) << 23;
  return p * s.f;
}

}  // namespace partsim
