#include "partsim/FieldGrid.h"

#include "partsim/RenderState.h"

namespace partsim {
namespace {

// Buoyancy in world units per second, opposite gravity.
constexpr float kBuoyancy = 30.0f;
// Fraction of heat remaining after one second. Cooling is the ONLY sink -- the box is closed, so
// heat that survives the climb piles up against the ceiling and the top face ends up as bright
// as the fire. It has to die on the way up.
constexpr float kCoolPerSecond = 0.03f;
// Swirl amplitude and how fast its phase drifts, so a plume wanders instead of rising as a
// straight column.
constexpr float kSwirl = 7.0f;
constexpr float kSwirlSpeed = 1.7f;

}  // namespace

bool FieldGrid::init(const SimVolume& v) {
  lo_ = v.box().lo;
  // Shared with HeatBuffer, deliberately. A display node receiving this field over the wire must
  // agree byte for byte about the grid layout, and two independent copies of the derivation would
  // fail silently -- as a plume drawn in the wrong place, not as an error.
  cell_ = heatCellSize();
  invCell_ = 1.0f / cell_;
  dim_ = heatGridDim(v, cell_);
  cellCount_ = dim_.x * dim_.y * dim_.z;
  if (cellCount_ > kMaxFieldCells) return false;
  clear();
  return true;
}

void FieldGrid::clear() {
  for (int i = 0; i < cellCount_; ++i) { a_[i] = 0; b_[i] = 0; }
  cur_ = a_;
  nxt_ = b_;
  peak_ = 0;
  time_ = 0.0f;
}

float FieldGrid::sampleCur(float fx, float fy, float fz) const {
  // Trilinear, with clamped edges. Grid coordinates, not world.
  const int x0 = iclamp((int)fx, 0, dim_.x - 1), x1 = imin(x0 + 1, dim_.x - 1);
  const int y0 = iclamp((int)fy, 0, dim_.y - 1), y1 = imin(y0 + 1, dim_.y - 1);
  const int z0 = iclamp((int)fz, 0, dim_.z - 1), z1 = imin(z0 + 1, dim_.z - 1);
  const float tx = pclamp(fx - (float)x0, 0.0f, 1.0f);
  const float ty = pclamp(fy - (float)y0, 0.0f, 1.0f);
  const float tz = pclamp(fz - (float)z0, 0.0f, 1.0f);

  const int sy = dim_.x, sz = dim_.x * dim_.y;
  const uint8_t* c = cur_;
  const float c000 = (float)c[z0 * sz + y0 * sy + x0], c100 = (float)c[z0 * sz + y0 * sy + x1];
  const float c010 = (float)c[z0 * sz + y1 * sy + x0], c110 = (float)c[z0 * sz + y1 * sy + x1];
  const float c001 = (float)c[z1 * sz + y0 * sy + x0], c101 = (float)c[z1 * sz + y0 * sy + x1];
  const float c011 = (float)c[z1 * sz + y1 * sy + x0], c111 = (float)c[z1 * sz + y1 * sy + x1];

  const float x00 = c000 + (c100 - c000) * tx, x10 = c010 + (c110 - c010) * tx;
  const float x01 = c001 + (c101 - c001) * tx, x11 = c011 + (c111 - c011) * tx;
  const float y0v = x00 + (x10 - x00) * ty, y1v = x01 + (x11 - x01) * ty;
  return y0v + (y1v - y0v) * tz;
}

float FieldGrid::sample(Vec3 world) const {
  if (peak_ == 0) return 0.0f;
  return sampleCur((world.x - lo_.x) * invCell_ - 0.5f, (world.y - lo_.y) * invCell_ - 0.5f,
                   (world.z - lo_.z) * invCell_ - 0.5f) * (1.0f / 255.0f);
}

void FieldGrid::step(const SimVolume& v, Vec3 gravity, Vec3 containerAccel,
                     const Emitter* emitters, int emitterCount, float dt, Rng& rng) {
  (void)v;
  // Nothing burning and nothing to light: skip the whole pass. Water-only scenes pay nothing.
  if (peak_ == 0 && emitterCount == 0) return;

  // "Up" is opposite gravity, so flames lean when the cube is tilted and get pressed around
  // when it is shaken -- the same object-space vector the solver uses, no extra plumbing.
  const Vec3 up = normalize(gravity) * -1.0f;
  const Vec3 drift = (up * kBuoyancy - containerAccel) * dt * invCell_;

  const float cool = 1.0f - (1.0f - kCoolPerSecond) * dt;

  // The swirl phase must drift SMOOTHLY. Re-randomising it every step makes the lateral
  // displacement white noise, which averages to nothing and leaves a perfectly straight column;
  // a slowly advancing phase gives coherent wander. Time is a pure function of step count, so
  // this stays deterministic.
  time_ += dt;
  const float phase = time_ * kSwirlSpeed;

  const int sy = dim_.x, sz = dim_.x * dim_.y;
  int peak = 0;

  for (int z = 0; z < dim_.z; ++z) {
    for (int y = 0; y < dim_.y; ++y) {
      for (int x = 0; x < dim_.x; ++x) {
        // Semi-Lagrangian: trace backwards along the velocity and sample where this cell's
        // contents came from. Unconditionally stable regardless of how large the step is,
        // which matters because buoyancy is fast relative to the cell size.
        const float swirl = kSwirl * dt * invCell_;
        const float sx = (float)x - drift.x +
                         swirl * fsin((float)y * 0.7f + (float)z * 0.31f + phase);
        const float syf = (float)y - drift.y;
        const float szf = (float)z - drift.z +
                          swirl * fcos((float)x * 0.63f + (float)y * 0.44f + phase);

        float value = sampleCur(sx, syf, szf) * cool;
        const int vi = (int)(value + 0.5f);
        nxt_[z * sz + y * sy + x] = (uint8_t)iclamp(vi, 0, 255);
        if (vi > peak) peak = vi;
      }
    }
  }

  // Emitters are injected after advection so they always show, even at high buoyancy.
  for (int e = 0; e < emitterCount; ++e) {
    const Emitter& em = emitters[e];
    const Vec3 s = v.box().size();
    const Vec3 centre{lo_.x + em.nx * s.x, lo_.y + em.ny * s.y, lo_.z + em.nz * s.z};
    const float r = em.radius;
    const int add = iclamp((int)(em.rate * 255.0f * dt * 60.0f), 0, 255);

    const int x0 = iclamp((int)((centre.x - r - lo_.x) * invCell_), 0, dim_.x - 1);
    const int x1 = iclamp((int)((centre.x + r - lo_.x) * invCell_), 0, dim_.x - 1);
    const int y0 = iclamp((int)((centre.y - r - lo_.y) * invCell_), 0, dim_.y - 1);
    const int y1 = iclamp((int)((centre.y + r - lo_.y) * invCell_), 0, dim_.y - 1);
    const int z0 = iclamp((int)((centre.z - r - lo_.z) * invCell_), 0, dim_.z - 1);
    const int z1 = iclamp((int)((centre.z + r - lo_.z) * invCell_), 0, dim_.z - 1);

    for (int z = z0; z <= z1; ++z)
      for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) {
          const Vec3 c = cellCentre(x, y, z);
          const float d2 = length2(c - centre);
          if (d2 > r * r) continue;
          // Falls off to the edge of the emitter, with a little flicker so the base of the
          // flame is not a flat disc.
          const float fall = 1.0f - psqrt(d2) / r;
          const int flick = 190 + (int)(rng.next() & 63);
          const int cell = z * sz + y * sy + x;
          const int vi = (int)nxt_[cell] + (int)((float)add * fall * (float)flick / 255.0f);
          nxt_[cell] = (uint8_t)imin(255, vi);
          if (nxt_[cell] > peak) peak = nxt_[cell];
        }
  }

  uint8_t* t = cur_;
  cur_ = nxt_;
  nxt_ = t;
  peak_ = peak;
}

}  // namespace partsim
