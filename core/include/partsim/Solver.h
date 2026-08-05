#pragma once
#include "partsim/SpatialHash.h"

namespace partsim {

// SPH kernels. Poly6 for density, Spiky for gradients -- the standard PBF pairing: poly6
// has a vanishing gradient at r -> 0 which would let particles clump with no restoring
// force, so the gradient comes from Spiky instead.
struct Kernels {
  float h, h2;
  float poly6C;  // 315 / (64 pi h^9)
  float spikyC;  // -45 / (pi h^6), sign folded in so spikyGrad returns grad(W) directly

  void init(float radius) {
    h = radius;
    h2 = radius * radius;
    const float h3 = h2 * h;
    const float h6 = h3 * h3;
    const float h9 = h6 * h3;
    poly6C = 315.0f / (64.0f * kPi * h9);
    spikyC = -45.0f / (kPi * h6);
  }

  float poly6(float r2) const {
    if (r2 >= h2) return 0.0f;
    const float d = h2 - r2;
    return poly6C * d * d * d;
  }

  // grad(W) with respect to the first argument. Points from i toward j (W decreases
  // outward), which is what makes a negative lambda produce repulsion.
  Vec3 spikyGrad(Vec3 rv, float r) const {
    if (r < 1e-5f || r >= h) return Vec3{0.0f, 0.0f, 0.0f};
    const float t = h - r;
    return rv * (spikyC * t * t / r);
  }
};

// Water and sand parameter table; scenes may substitute their own.
const MaterialParams* defaultMaterials();

class Solver {
 public:
  // Derives particle mass from a rest lattice so that rest density is exactly 1.0, which
  // keeps every lambda and epsilon in this file O(1) and independent of the kernel's
  // normalisation constant. Also builds the wall-density LUT.
  void init();

  // Runtime-tunable so the host bench can sweep them; both default to the Config values.
  void setIterations(int n) { iterations_ = imax(1, n); }
  void setDamping(float d) { damping_ = pclamp(d, 0.0f, 1.0f); }
  void setEpsilon(float e) { epsilon_ = e; }
  void setSCorrK(float k) { sCorrK_ = k; }
  int iterations() const { return iterations_; }

  float mass() const { return mass_; }

  // How many particles a volume holds at rest density. Filling much past this leaves the
  // fluid permanently over-compressed, and above roughly 5% the solver churns instead of
  // settling -- so scenes should treat this as a hard ceiling, not a suggestion.
  int capacity(const SimVolume& v) const {
    const Vec3 s = v.box().size();
    return (int)(s.x * s.y * s.z / mass_);
  }
  float restDensity() const { return 1.0f; }
  const Kernels& kernels() const { return k_; }

  // One fixed-dt PBF step. Builds the neighbour grid internally (on predicted positions,
  // which is the only correct choice) and permutes the particle arrays as a side effect.
  // scratch must be at least kMaxParticles * 4 bytes.
  void step(Particles& p, const SimVolume& v, SpatialHash& h, void* scratch,
            const MaterialParams* mats, Vec3 gravity, float dt);

  // Density at particle i including the wall term. Exposed for tests and diagnostics.
  float densityAt(const Particles& p, const SimVolume& v, const SpatialHash& h, int i,
                  float rho0 = 1.0f) const;
  // Fraction of a neighbourhood hidden by a wall at distance d, in rest-density units.
  // wallFraction(0) == 0.5, wallFraction(h) == 0.
  float wallFraction(float d) const { return wallDensity(d); }

 private:
  float wallDensity(float d) const;
  float wallDensityAt(Vec3 pi, const Aabb& b, float rho0) const;
  void solveIteration(Particles& p, const SimVolume& v, const SpatialHash& h,
                      const MaterialParams* mats);

  Kernels k_;
  int iterations_ = kSolverIterations;
  float damping_ = kVelocityDamping;
  float epsilon_ = kCfmEpsilon;
  float sCorrK_ = kSCorrK;
  float mass_ = 1.0f;
  // Missing density from the half-space beyond a wall, as a function of distance to it.
  float wallLut_[17];
  float wallLutScale_ = 1.0f;
};

}  // namespace partsim
