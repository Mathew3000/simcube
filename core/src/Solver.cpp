#include "partsim/Solver.h"

namespace partsim {
namespace {

// Rest lattice used both to derive particle mass and to estimate the density a wall hides.
// Offsets are on a cubic lattice of spacing kRestSpacing; only sites inside the kernel
// support contribute.
constexpr int kLatticeReach = 3;  // covers h / d = 2 with room to spare

inline Vec3 clampToBox(Vec3 p, const Aabb& b) {
  // A hair inside the far edge so cell indexing never lands on the exclusive upper bound.
  const float e = 1e-3f;
  return Vec3{pclamp(p.x, b.lo.x, b.hi.x - e), pclamp(p.y, b.lo.y, b.hi.y - e),
              pclamp(p.z, b.lo.z, b.hi.z - e)};
}

}  // namespace

const MaterialParams* defaultMaterials() {
  static const MaterialParams mats[kMaterialCount] = {
      // restDensityScale, xsph, friction, staticVelocity
      {1.0f, kXsphC, 0.0f, 0.0f},  // water: freely flowing, mild XSPH cohesion
      {2.0f, 0.0f, 0.7f, 0.35f},   // sand: twice as dense so it sinks, high friction
  };
  return mats;
}

void Solver::init() {
  k_.init(kSmoothRadius);

  // Rest density of the lattice with unit mass, then choose mass so rest density == 1.
  // Doing it numerically rather than analytically means retuning kRestSpacing or
  // kSmoothRadius cannot silently invalidate the constants below.
  float latticeRho = 0.0f;
  for (int i = -kLatticeReach; i <= kLatticeReach; ++i)
    for (int j = -kLatticeReach; j <= kLatticeReach; ++j)
      for (int l = -kLatticeReach; l <= kLatticeReach; ++l) {
        const Vec3 o{(float)i * kRestSpacing, (float)j * kRestSpacing,
                     (float)l * kRestSpacing};
        latticeRho += k_.poly6(length2(o));
      }
  mass_ = 1.0f / latticeRho;

  // Wall density LUT: the fraction of the kernel's mass lying beyond a plane at distance d.
  //
  // A particle near a wall is missing every neighbour the wall displaces; without
  // compensating, that deficit reads as tension and the fluid visibly shrinks away from the
  // glass -- exactly the wrong artifact when the glass is what you are looking at. Worse,
  // under-compensating lets the fluid over-pack against the floor while the measured density
  // still looks correct.
  //
  // Poly6 integrates in closed form. With W = C(h^2-r^2)^3, the marginal over the plane at
  // offset x is M(x) = pi*C*(h^2-x^2)^4 / 4, so
  //     fraction(d) = (pi*C/4) * integral_d^h (h^2-x^2)^4 dx
  // and the antiderivative of the expanded quartic is exact. Sanity: fraction(0) == 0.5 and
  // fraction(h) == 0, both asserted by tests/test_solver.cpp.
  const int last = (int)(sizeof(wallLut_) / sizeof(wallLut_[0])) - 1;
  wallLutScale_ = (float)last / k_.h;
  const float h2 = k_.h2, h4 = h2 * h2, h6 = h4 * h2, h8 = h4 * h4;
  const float pre = kPi * k_.poly6C * 0.25f;
  auto antideriv = [&](float x) {
    const float x2 = x * x, x3 = x2 * x, x5 = x3 * x2, x7 = x5 * x2, x9 = x7 * x2;
    return h8 * x - 4.0f * h6 * x3 / 3.0f + 6.0f * h4 * x5 / 5.0f - 4.0f * h2 * x7 / 7.0f +
           x9 / 9.0f;
  };
  const float atH = antideriv(k_.h);
  for (int q = 0; q <= last; ++q) {
    const float d = (float)q * k_.h / (float)last;
    wallLut_[q] = pre * (atH - antideriv(d));
  }
}

// Fraction of a neighbourhood hidden by a wall at distance d, in units of rest density.
float Solver::wallDensity(float d) const {
  if (d >= k_.h) return 0.0f;
  if (d <= 0.0f) return wallLut_[0];
  const float f = d * wallLutScale_;
  const int q = (int)f;
  const float frac = f - (float)q;
  return wallLut_[q] + (wallLut_[q + 1] - wallLut_[q]) * frac;
}

float Solver::wallDensityAt(Vec3 pi, const Aabb& b, float rho0) const {
  // One axis-aligned wall at a time, so a corner double-counts the overlap slightly.
  // Acceptable, and erring dense in corners is the safe direction.
  float f = wallDensity(pi.x - b.lo.x) + wallDensity(b.hi.x - pi.x);
  f += wallDensity(pi.y - b.lo.y) + wallDensity(b.hi.y - pi.y);
  f += wallDensity(pi.z - b.lo.z) + wallDensity(b.hi.z - pi.z);
  return f * rho0;
}

float Solver::densityAt(const Particles& p, const SimVolume& v, const SpatialHash& h,
                        int i, float rho0) const {
  const Vec3 pi = p.pred(i);
  float rho = mass_ * k_.poly6(0.0f);  // self
  forEachNeighbour(v, h, pi, [&](int j) {
    if (j == i) return;
    rho += mass_ * k_.poly6(length2(p.pred(j) - pi));
  });
  return rho + wallDensityAt(pi, v.box(), rho0);
}

void Solver::solveIteration(Particles& p, const SimVolume& v, const SpatialHash& h,
                            const MaterialParams* mats) {
  const int n = p.n;
  const Aabb& b = v.box();

  // --- pass A+B: density and Lagrange multiplier in ONE neighbour walk -----
  // Fused deliberately. Computing density and sum-of-squared-gradients in separate passes
  // doubles the number of full 27-cell scans, and those scans are the dominant cost of the
  // whole solver (~80 candidate visits per particle per scan).
  for (int i = 0; i < n; ++i) {
    const Vec3 pi = p.pred(i);
    const float rho0 = mats[p.mat[i]].restDensityScale;
    const float w = mass_ / rho0;

    float rho = mass_ * k_.poly6(0.0f);  // self
    Vec3 gradI{0.0f, 0.0f, 0.0f};
    float sumGrad2 = 0.0f;
    forEachNeighbour(v, h, pi, [&](int j) {
      if (j == i) return;
      const Vec3 rv = pi - p.pred(j);
      const float r2 = length2(rv);
      if (r2 >= k_.h2) return;
      rho += mass_ * k_.poly6(r2);
      const Vec3 gj = k_.spikyGrad(rv, psqrt(r2)) * w;
      gradI += gj;
      sumGrad2 += length2(gj);
    });
    rho += wallDensityAt(pi, b, rho0);

    const float C = rho / rho0 - 1.0f;
    if (C <= 0.0f) {
      // Only resolve compression. Pulling a rarefied free surface back together is what
      // produces the clumping and tensile instability PBF is known for.
      p.lam[i] = 0.0f;
      continue;
    }
    sumGrad2 += length2(gradI);
    p.lam[i] = -C / (sumGrad2 + epsilon_);
  }

  // --- pass C: positional correction --------------------------------------
  // Applied in place, i.e. Gauss-Seidel rather than the textbook Jacobi. That costs a
  // 3N delta buffer we cannot afford on the ESP32, converges faster, and stays
  // deterministic because the particle order is a pure function of position.
  const float dq = 0.2f * k_.h;
  const float wq = k_.poly6(dq * dq);
  for (int i = 0; i < n; ++i) {
    const float li = p.lam[i];
    const Vec3 pi = p.pred(i);
    const float w = mass_ / mats[p.mat[i]].restDensityScale;

    Vec3 dp{0.0f, 0.0f, 0.0f};
    forEachNeighbour(v, h, pi, [&](int j) {
      if (j == i) return;
      const Vec3 rv = pi - p.pred(j);
      const float r2 = length2(rv);
      if (r2 >= k_.h2) return;

      // Macklin's artificial pressure. Without it particles cluster into strings, and the
      // effect is worse at low iteration counts, not better.
      float ratio = k_.poly6(r2) / wq;
      float t = 1.0f;
      for (int e = 0; e < kSCorrN; ++e) t *= ratio;
      const float sCorr = -sCorrK_ * t;

      dp += k_.spikyGrad(rv, psqrt(r2)) * (li + p.lam[j] + sCorr);
    });
    dp *= w;

    // Clamp the per-iteration correction. This is the single most effective guard against
    // a blow-up: an over-large delta pushes a particle through its neighbours, which
    // raises the next density even further.
    const float d2 = length2(dp);
    if (d2 > kMaxDeltaP * kMaxDeltaP) dp *= kMaxDeltaP * prsqrt(d2);

    p.setPred(i, clampToBox(pi + dp, b));
  }
}

void Solver::step(Particles& p, const SimVolume& v, SpatialHash& h, void* scratch,
                  const MaterialParams* mats, Vec3 gravity, float dt) {
  const int n = p.n;
  if (n == 0) return;
  const Aabb& b = v.box();

  // --- predict -------------------------------------------------------------
  // CFL: a particle must not cross more than a fraction of the kernel radius per step, or
  // it tunnels past the neighbours that were supposed to stop it.
  const float vMax = 0.4f * k_.h / dt;
  const float vMax2 = vMax * vMax;
  for (int i = 0; i < n; ++i) {
    Vec3 vel = p.vel(i) + gravity * dt;
    const float s2 = length2(vel);
    if (s2 > vMax2) vel *= vMax * prsqrt(s2);
    p.setVel(i, vel);
    p.setPred(i, clampToBox(p.pos(i) + vel * dt, b));
  }

  // Grid is built on PREDICTED positions: those are what the constraints operate on.
  h.build(v, p, scratch);

  for (int it = 0; it < iterations_; ++it) solveIteration(p, v, h, mats);

  // --- velocity from the corrected positions -------------------------------
  const float invDt = 1.0f / dt;
  const float sleep2 = kSleepSpeed * kSleepSpeed;
  for (int i = 0; i < n; ++i) {
    const Vec3 pos = p.pos(i), pred = p.pred(i);
    Vec3 nv = (pred - pos) * (invDt * damping_);
    if (length2(nv) < sleep2) nv = Vec3{0.0f, 0.0f, 0.0f};
    p.setVel(i, nv);
    p.setPos(i, pred);
  }

  // --- XSPH viscosity ------------------------------------------------------
  // In place, for the same memory reason as pass C. At these coefficients the resulting
  // slight asymmetry in diffusion is invisible.
  for (int i = 0; i < n; ++i) {
    const float c = mats[p.mat[i]].xsph;
    if (c <= 0.0f) continue;
    const Vec3 pi = p.pos(i), vi = p.vel(i);
    Vec3 dv{0.0f, 0.0f, 0.0f};
    forEachNeighbour(v, h, pi, [&](int j) {
      if (j == i) return;
      dv += (p.vel(j) - vi) * k_.poly6(length2(p.pos(j) - pi));
    });
    p.setVel(i, vi + dv * (c * mass_));
  }
}

}  // namespace partsim
