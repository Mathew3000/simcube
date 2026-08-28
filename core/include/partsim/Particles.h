#pragma once
#include "partsim/Config.h"
#include "partsim/Types.h"

namespace partsim {

// Struct-of-arrays, fixed capacity, no dynamic allocation ever.
//
// SoA rather than AoS because the three hot loops each touch a different subset of fields:
// predict reads/writes position+velocity, the density gather reads only predicted
// positions, the correction pass reads lam + predicted positions. AoS would drag `mat` and
// `lam` through a 32KB cache with no prefetcher on every position read.
//
// This struct is ~700KB at the host capacity, so it must live in static storage or on the
// heap -- never on the stack.
struct Particles {
  alignas(16) float x[kMaxParticles];
  alignas(16) float y[kMaxParticles];
  alignas(16) float z[kMaxParticles];

  alignas(16) float vx[kMaxParticles];
  alignas(16) float vy[kMaxParticles];
  alignas(16) float vz[kMaxParticles];

  // Predicted ("starred") position: where the particle would go before constraints.
  alignas(16) float sx[kMaxParticles];
  alignas(16) float sy[kMaxParticles];
  alignas(16) float sz[kMaxParticles];

  // Holds density during the first solver pass, then is overwritten in place with the
  // Lagrange multiplier. Safe because lambda_i depends on rho_i and on positions, never on
  // rho_j -- which is what lets one array serve both and keeps the ESP32 budget.
  alignas(16) float lam[kMaxParticles];

  uint8_t mat[kMaxParticles];

  int32_t n = 0;

  void clear() { n = 0; }

  bool add(Vec3 p, Vec3 v, uint8_t material) {
    if (n >= kMaxParticles) return false;
    const int i = n++;
    x[i] = p.x; y[i] = p.y; z[i] = p.z;
    vx[i] = v.x; vy[i] = v.y; vz[i] = v.z;
    sx[i] = p.x; sy[i] = p.y; sz[i] = p.z;
    lam[i] = 0.0f;
    mat[i] = material;
    return true;
  }

  // Swap-with-last removal. Order is arbitrary anyway -- the neighbour grid permutes the arrays
  // every step -- so this is O(1) rather than a shift.
  void removeAt(int i) {
    const int last = --n;
    if (i == last) return;
    x[i] = x[last]; y[i] = y[last]; z[i] = z[last];
    vx[i] = vx[last]; vy[i] = vy[last]; vz[i] = vz[last];
    sx[i] = sx[last]; sy[i] = sy[last]; sz[i] = sz[last];
    lam[i] = lam[last];
    mat[i] = mat[last];
  }

  // Read-only view for the renderer. Lets one splat implementation serve both a full simulation
  // and a draw-only node; see RenderState.h.
  struct ParticleView view() const;

  Vec3 pos(int i) const { return Vec3{x[i], y[i], z[i]}; }
  Vec3 vel(int i) const { return Vec3{vx[i], vy[i], vz[i]}; }
  Vec3 pred(int i) const { return Vec3{sx[i], sy[i], sz[i]}; }

  void setPos(int i, Vec3 p) { x[i] = p.x; y[i] = p.y; z[i] = p.z; }
  void setVel(int i, Vec3 v) { vx[i] = v.x; vy[i] = v.y; vz[i] = v.z; }
  void setPred(int i, Vec3 p) { sx[i] = p.x; sy[i] = p.y; sz[i] = p.z; }
};

}  // namespace partsim
