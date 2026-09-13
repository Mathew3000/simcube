#include "partsim/InkField.h"

namespace partsim {
namespace {

// Settling speed of dye through the carrier liquid, in cells per second, Q8.8. Slow on purpose:
// this is dye sinking through water, not a flame rising through air, and the reference image is
// defined by how long a structure survives rather than by how fast it travels.
constexpr int32_t kSettleQ8 = 3 * kInkOne;

// How much of the container's acceleration reaches the dye as a bulk impulse. The dye is denser
// than the carrier, so a shake pushes it, but far less than it would push a free particle.
constexpr int32_t kAccelGainQ8 = kInkOne / 2;

// Curl-mode amplitude (Q8.8 cells/s) and how fast its phase advances (LUT steps per second).
// Two modes, both very low frequency: their job is to stop a plume descending as a straight
// column, not to add detail the panels could not resolve anyway.
constexpr int32_t kCurlQ8 = kInkOne / 2;
constexpr uint32_t kCurlPhasePerSec = 12;

// The CFL guard of docs/DESIGN-SUGGESTIONS.md section 4. Semi-Lagrangian advection stays stable
// for a far larger displacement, but a tendril that jumps over a cell disappears instead of
// moving, so the visual limit binds before the numerical one.
constexpr int32_t kMaxDisplacementQ8 = kInkOne;

// Cells of halo around the occupied box. One for the displacement the guard above permits, one
// because trilinear gathers from the cell beyond that.
constexpr int kHalo = 2;

inline int32_t clampQ(int32_t v, int32_t lim) { return v > lim ? lim : (v < -lim ? -lim : v); }

}  // namespace

bool InkField::init(const SimVolume& v) {
  lo_ = v.box().lo;
  // One uniform cell across the largest extent, so the grid covers the whole container and every
  // face samples the SAME object space. Six independent 2D simulations cannot keep a feature
  // whole around a corner; one shared field is the only reason a tendril survives an edge.
  const Vec3 s = v.box().size();
  const float extent = pmax(pmax(s.x, s.y), s.z);
  if (extent <= 0.0f) return false;
  cell_ = extent / (float)kInkDim;
  invCell_ = 1.0f / cell_;

  // (1 - r^2/R^2)^2, tabulated. Zero at the last entry, so a cell just outside a vorton gets
  // exactly nothing rather than a small step -- a discontinuity here reads as a visible shell.
  for (int i = 0; i < 256; ++i) {
    const int32_t t = 255 - i;
    falloff_[i] = (int16_t)((t * t) >> 8);
  }
  // Built through partsim::fsin, not libm: sinf differs in the last bits between newlib, musl
  // and libSystem, and this table feeds the deterministic path.
  for (int i = 0; i < 256; ++i) {
    const float a = (float)i * (2.0f * kPi / 256.0f);
    sin_[i] = (int16_t)(fsin(a) * 255.0f);
  }

  clear();
  return true;
}

void InkField::clear() {
  for (int b = 0; b < 2; ++b)
    for (int c = 0; c < kInkChannels; ++c)
      for (int i = 0; i < kInkCells; ++i) dye_[b][c][i] = 0;
  cur_ = 0;
  phase_ = 0;
  peak_ = 0;
  ++rev_;
  loX_ = loY_ = loZ_ = 0;
  hiX_ = hiY_ = hiZ_ = -1;  // empty
  for (int i = 0; i < kMaxVortons; ++i) vort_[i].life = 0;
}

void InkField::seedFlow(Rng& rng) {
  // Axes only, once. The phase then evolves continuously in step(); choosing a new direction every
  // frame makes the lateral displacement white noise, which averages out and leaves a straight
  // column. core/src/FieldGrid.cpp carries the same warning for the same reason.
  for (int i = 0; i < kMaxVortons; ++i) {
    const Vec3 a = normalize(Vec3{rng.nextSigned(), rng.nextSigned(), rng.nextSigned()});
    vort_[i].ax = (int8_t)(a.x * 127.0f);
    vort_[i].ay = (int8_t)(a.y * 127.0f);
    vort_[i].az = (int8_t)(a.z * 127.0f);
    vort_[i].life = 0;
  }
}

int InkField::vortonCount() const {
  int n = 0;
  for (int i = 0; i < kMaxVortons; ++i)
    if (vort_[i].life > 0) ++n;
  return n;
}

void InkField::inject(Vec3 objectPos, float radius, int channel, uint8_t amount) {
  if (channel < 0 || channel >= kInkChannels || amount == 0) return;

  const float gx = (objectPos.x - lo_.x) * invCell_ - 0.5f;
  const float gy = (objectPos.y - lo_.y) * invCell_ - 0.5f;
  const float gz = (objectPos.z - lo_.z) * invCell_ - 0.5f;
  const float r = radius * invCell_;
  if (r <= 0.0f) return;
  const float invR2 = 1.0f / (r * r);

  const int x0 = imax(0, (int)(gx - r)), x1 = imin(kInkDim - 1, (int)(gx + r) + 1);
  const int y0 = imax(0, (int)(gy - r)), y1 = imin(kInkDim - 1, (int)(gy + r) + 1);
  const int z0 = imax(0, (int)(gz - r)), z1 = imin(kInkDim - 1, (int)(gz + r) + 1);

  uint8_t* dst = dye_[cur_][channel];
  for (int z = z0; z <= z1; ++z) {
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        const float dx = (float)x - gx, dy = (float)y - gy, dz = (float)z - gz;
        const float q = (dx * dx + dy * dy + dz * dz) * invR2;
        if (q >= 1.0f) continue;
        const float w = (1.0f - q) * (1.0f - q);
        const int i = (z * kInkDim + y) * kInkDim + x;
        // Saturating add: dye is a mass, and two pours into the same cell must not wrap it to
        // transparent -- which would read as a hole punched in the plume.
        const int v = (int)dst[i] + (int)((float)amount * w);
        dst[i] = (uint8_t)imin(255, v);
        if (dst[i] > peak_) peak_ = dst[i];
      }
    }
  }
  rebuildBounds();
  ++rev_;
}

void InkField::spawnVortonPair(Vec3 objectPos, Vec3 axisObject, float radius, int strength,
                               int life) {
  if (life <= 0 || radius <= 0.0f) return;

  const float gx = ((objectPos.x - lo_.x) * invCell_ - 0.5f) * (float)kInkOne;
  const float gy = ((objectPos.y - lo_.y) * invCell_ - 0.5f) * (float)kInkOne;
  const float gz = ((objectPos.z - lo_.z) * invCell_ - 0.5f) * (float)kInkOne;
  const float rCells = radius * invCell_;
  const int32_t r2max = (int32_t)(rCells * rCells * (float)kInkOne);
  if (r2max <= 0) return;

  const Vec3 a = normalize(axisObject);
  const bool haveAxis = (a.x != 0.0f || a.y != 0.0f || a.z != 0.0f);

  // Two slots, counter-rotating. A single vorton spreads a blob; a PAIR is what rolls the leading
  // edge of a plume over on itself, which is the structure the reference image is full of.
  for (int k = 0; k < 2; ++k) {
    int slot = -1;
    uint16_t worst = 0xFFFF;
    for (int i = 0; i < kMaxVortons; ++i) {
      if (vort_[i].life == 0) { slot = i; break; }
      if (vort_[i].life < worst) { worst = vort_[i].life; slot = i; }  // evict the most decayed
    }
    if (slot < 0) return;

    Vorton& w = vort_[slot];
    // Offset the pair along the axis so they counter-rotate about a common centre rather than
    // sitting on top of each other and cancelling.
    const float off = (k == 0 ? 0.5f : -0.5f) * rCells * (float)kInkOne;
    w.cx = (int32_t)(gx + a.x * off);
    w.cy = (int32_t)(gy + a.y * off);
    w.cz = (int32_t)(gz + a.z * off);
    if (haveAxis) {
      w.ax = (int8_t)(a.x * 127.0f);
      w.ay = (int8_t)(a.y * 127.0f);
      w.az = (int8_t)(a.z * 127.0f);
    }  // else keep the axis seedFlow() gave this slot
    w.r2max = r2max;
    // idx = (r2 * invR2) >> 16 lands in 0..255 across the support, with no divide in the loop.
    w.invR2 = (uint32_t)((1 << 24) / r2max);
    w.strength = (int16_t)iclamp(k == 0 ? strength : -strength, -32767, 32767);
    w.life = (uint16_t)imin(life, 65535);
  }
}

void InkField::velocityAt(int px, int py, int pz, int32_t& vx, int32_t& vy, int32_t& vz) const {
  vx = driftX_;
  vy = driftY_;
  vz = driftZ_;

  for (int i = 0; i < kMaxVortons; ++i) {
    const Vorton& w = vort_[i];
    if (w.life == 0) continue;
    const int32_t dx = px - w.cx, dy = py - w.cy, dz = pz - w.cz;
    // Q8.8 squared: (Q8.8)^2 is Q16.16, and >>8 brings the sum back into Q8.8 of r^2 in cells.
    const int32_t r2 = (dx * dx + dy * dy + dz * dz) >> kInkFracBits;
    if (r2 >= w.r2max) continue;  // outside the support: one compare, no arithmetic
    const int idx = (int)((r2 * (int32_t)w.invR2) >> 16);
    const int32_t f = (falloff_[idx] * w.strength) >> 8;
    // Tangential: cross(axis, p - centre). Q0.7 x Q8.8 -> Q8.15, back to Q8.8 with >>7.
    const int32_t tx = ((int32_t)w.ay * dz - (int32_t)w.az * dy) >> 7;
    const int32_t ty = ((int32_t)w.az * dx - (int32_t)w.ax * dz) >> 7;
    const int32_t tz = ((int32_t)w.ax * dy - (int32_t)w.ay * dx) >> 7;
    vx += (tx * f) >> 16;
    vy += (ty * f) >> 16;
    vz += (tz * f) >> 16;
  }

  // Two low-frequency curl modes, out of phase with each other so the plume wanders in two axes
  // rather than swinging along one diagonal.
  const uint32_t p = phase_;
  const int32_t s0 = sin_[(uint32_t)((py >> 6) + (int32_t)p) & 255u];
  const int32_t s1 = sin_[(uint32_t)((px >> 6) + (pz >> 7) + (int32_t)p) & 255u];
  vx += (s0 * kCurlQ8) >> 8;
  vz += (s1 * kCurlQ8) >> 8;
}

void InkField::step(Vec3 gravityObject, Vec3 containerAccel, int dtMillis) {
  if (dtMillis <= 0) return;
  // Nothing to move. A clear volume costs one branch, which is the point of tracking the box.
  if (peak_ == 0) return;

  // Per-step scale, computed ONCE: the cell loop must not divide.
  const int32_t dtQ16 = (int32_t)(((int64_t)dtMillis << 16) / 1000);

  // Settling is along gravity; the bulk impulse is the container's own acceleration. Both are
  // filtered upstream by MotionSource, so what arrives here is already a stable direction.
  const Vec3 g = normalize(gravityObject);
  driftX_ = (int32_t)(g.x * (float)kSettleQ8) + (int32_t)(containerAccel.x * (float)kAccelGainQ8);
  driftY_ = (int32_t)(g.y * (float)kSettleQ8) + (int32_t)(containerAccel.y * (float)kAccelGainQ8);
  driftZ_ = (int32_t)(g.z * (float)kSettleQ8) + (int32_t)(containerAccel.z * (float)kAccelGainQ8);

  phase_ = (phase_ + (uint32_t)((kCurlPhasePerSec * (uint32_t)dtMillis) / 1000u)) & 255u;

  // The destination is cleared in full rather than only inside the box. Leaving the outside stale
  // would resurrect dye from two steps ago the moment the box grew over it -- a ghost that
  // appears only when a plume expands, which is exactly when nobody is looking for a bug.
  const int nxt = cur_ ^ 1;
  for (int c = 0; c < kInkChannels; ++c)
    for (int i = 0; i < kInkCells; ++i) dye_[nxt][c][i] = 0;

  int x0 = 0, x1 = kInkDim - 1, y0 = 0, y1 = kInkDim - 1, z0 = 0, z1 = kInkDim - 1;
  if (!forceFull_ && hiX_ >= loX_) {
    x0 = imax(0, loX_ - kHalo); x1 = imin(kInkDim - 1, hiX_ + kHalo);
    y0 = imax(0, loY_ - kHalo); y1 = imin(kInkDim - 1, hiY_ + kHalo);
    z0 = imax(0, loZ_ - kHalo); z1 = imin(kInkDim - 1, hiZ_ + kHalo);
  }

  uint8_t peak = 0;
  int nlo[3] = {kInkDim, kInkDim, kInkDim};
  int nhi[3] = {-1, -1, -1};

  for (int z = z0; z <= z1; ++z) {
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        // Cell x sits AT grid coordinate x, not at x + 0.5. The sampler below splits a coordinate
        // into (index, fraction), and inject() measures its ball the same way, so a half-cell
        // offset here is not a rounding preference -- it makes the zero-velocity gather sample
        // halfway between two cells, which slides the whole field half a cell every step. It
        // looked exactly like a settling bias, and it moved the dye the same way whichever
        // direction gravity pointed.
        const int32_t px = x << kInkFracBits;
        const int32_t py = y << kInkFracBits;
        const int32_t pz = z << kInkFracBits;

        int32_t vx, vy, vz;
        velocityAt(px, py, pz, vx, vy, vz);

        // No flux through the container wall: in the outermost cell the OUTWARD normal component
        // is dropped, leaving the flow tangential there.
        //
        // Without this, dye driven against a wall is duplicated rather than piled up: every
        // destination cell whose backtrace leaves the grid clamps to the same edge sample, so one
        // bright cell is copied into its whole neighbourhood. Measured at a 93x mass gain in ten
        // steps with the plume in a corner -- and it would have read as the ink glowing where it
        // touches the glass, which is far too plausible to be caught by eye.
        if (x == 0 && vx > 0) vx = 0;
        if (x == kInkDim - 1 && vx < 0) vx = 0;
        if (y == 0 && vy > 0) vy = 0;
        if (y == kInkDim - 1 && vy < 0) vy = 0;
        if (z == 0 && vz > 0) vz = 0;
        if (z == kInkDim - 1 && vz < 0) vz = 0;

        // Trace backwards. One source coordinate, shared by every dye channel -- the velocity is
        // a property of the flow, not of the colour being carried.
        const int32_t sx = px - clampQ((vx * dtQ16) >> 16, kMaxDisplacementQ8);
        const int32_t sy = py - clampQ((vy * dtQ16) >> 16, kMaxDisplacementQ8);
        const int32_t sz = pz - clampQ((vz * dtQ16) >> 16, kMaxDisplacementQ8);

        // Clamped trilinear. Every index below is forced into the grid, so no lookup can leave it
        // however far a vorton throws the source coordinate.
        int ix = sx >> kInkFracBits, iy = sy >> kInkFracBits, iz = sz >> kInkFracBits;
        int fx = sx & (kInkOne - 1), fy = sy & (kInkOne - 1), fz = sz & (kInkOne - 1);
        if (ix < 0) { ix = 0; fx = 0; } else if (ix >= kInkDim - 1) { ix = kInkDim - 2; fx = kInkOne - 1; }
        if (iy < 0) { iy = 0; fy = 0; } else if (iy >= kInkDim - 1) { iy = kInkDim - 2; fy = kInkOne - 1; }
        if (iz < 0) { iz = 0; fz = 0; } else if (iz >= kInkDim - 1) { iz = kInkDim - 2; fz = kInkOne - 1; }

        const int i000 = (iz * kInkDim + iy) * kInkDim + ix;
        const int i100 = i000 + 1;
        const int i010 = i000 + kInkDim;
        const int i110 = i010 + 1;
        const int i001 = i000 + kInkDim * kInkDim;
        const int i101 = i001 + 1;
        const int i011 = i001 + kInkDim;
        const int i111 = i011 + 1;

        // Weights that sum to exactly 65536 in the xy plane and 256 in z, and every shift below
        // ROUNDS rather than truncates.
        //
        // Truncation here is not a rounding detail, it is a leak: the error is one-sided, so every
        // cell loses up to one count every step and a still field evaporates. Measured before the
        // fix at 21% of the dye gone in six steps with no flow at all -- which would have looked
        // like the semi-Lagrangian diffusion the design document warns about, and been "solved"
        // by tuning a decay constant that was never the cause.
        const int32_t wx1 = fx, wx0 = kInkOne - fx;
        const int32_t wy1 = fy, wy0 = kInkOne - fy;
        const int32_t wz1 = fz, wz0 = kInkOne - fz;
        const int32_t w00 = wx0 * wy0, w10 = wx1 * wy0;  // Q16, summing to exactly 1<<16
        const int32_t w01 = wx0 * wy1, w11 = wx1 * wy1;

        const int oi = (z * kInkDim + y) * kInkDim + x;
        for (int c = 0; c < kInkChannels; ++c) {
          const uint8_t* src = dye_[cur_][c];
          const int32_t lo = src[i000] * w00 + src[i100] * w10 + src[i010] * w01 + src[i110] * w11;
          const int32_t hi = src[i001] * w00 + src[i101] * w10 + src[i011] * w01 + src[i111] * w11;
          // Down to Q8 first: the full-precision product would overflow int32 against wz.
          const int32_t lo8 = (lo + 128) >> 8;
          const int32_t hi8 = (hi + 128) >> 8;
          const int32_t val = (lo8 * wz0 + hi8 * wz1 + 32768) >> 16;
          const uint8_t out = (uint8_t)iclamp((int)val, 0, 255);
          dye_[nxt][c][oi] = out;
          if (out > peak) peak = out;
          if (out != 0) {
            if (x < nlo[0]) nlo[0] = x;
            if (x > nhi[0]) nhi[0] = x;
            if (y < nlo[1]) nlo[1] = y;
            if (y > nhi[1]) nhi[1] = y;
            if (z < nlo[2]) nlo[2] = z;
            if (z > nhi[2]) nhi[2] = z;
          }
        }
      }
    }
  }

  cur_ = nxt;
  peak_ = peak;
  ++rev_;
  loX_ = nlo[0]; loY_ = nlo[1]; loZ_ = nlo[2];
  hiX_ = nhi[0]; hiY_ = nhi[1]; hiZ_ = nhi[2];

  for (int i = 0; i < kMaxVortons; ++i) {
    if (vort_[i].life == 0) continue;
    // Vortons drift with the bulk flow, so a curl stays with the dye it curled rather than
    // sitting still while the plume descends past it.
    vort_[i].cx += (driftX_ * dtQ16) >> 16;
    vort_[i].cy += (driftY_ * dtQ16) >> 16;
    vort_[i].cz += (driftZ_ * dtQ16) >> 16;
    --vort_[i].life;
  }
}

void InkField::rebuildBounds() {
  int nlo[3] = {kInkDim, kInkDim, kInkDim};
  int nhi[3] = {-1, -1, -1};
  uint8_t peak = 0;
  for (int z = 0; z < kInkDim; ++z) {
    for (int y = 0; y < kInkDim; ++y) {
      for (int x = 0; x < kInkDim; ++x) {
        const int i = (z * kInkDim + y) * kInkDim + x;
        int any = 0;
        for (int c = 0; c < kInkChannels; ++c) {
          const uint8_t v = dye_[cur_][c][i];
          if (v > peak) peak = v;
          any |= v;
        }
        if (any == 0) continue;
        if (x < nlo[0]) nlo[0] = x;
        if (x > nhi[0]) nhi[0] = x;
        if (y < nlo[1]) nlo[1] = y;
        if (y > nhi[1]) nhi[1] = y;
        if (z < nlo[2]) nlo[2] = z;
        if (z > nhi[2]) nhi[2] = z;
      }
    }
  }
  loX_ = nlo[0]; loY_ = nlo[1]; loZ_ = nlo[2];
  hiX_ = nhi[0]; hiY_ = nhi[1]; hiZ_ = nhi[2];
  peak_ = peak;
}

}  // namespace partsim
