#include "partsim/SimFrame.h"

namespace partsim {
namespace {

// Explicit little-endian byte writes. All three targets are little-endian, but spelling it out is
// free and removes the question entirely -- a format whose correctness depends on the host's byte
// order is a format that will eventually be wrong.
inline void put16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}
inline void put32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}
inline uint16_t get16(const uint8_t* p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
inline uint32_t get32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// Fletcher-16 rather than a CRC: two adds per byte against a table lookup, and at ~23KB per frame
// thirty times a second that difference is real on a 240MHz core. It will not catch every error
// class a CRC would, which is an acceptable trade here -- the transport is a short on-board SPI
// link, and the failure mode of a missed error is one bad frame that the node then holds.
uint16_t fletcher16(const uint8_t* d, int n, uint16_t seed = 0) {
  uint32_t s1 = seed & 0xFF, s2 = (uint32_t)(seed >> 8);
  for (int i = 0; i < n; ++i) {
    s1 = (s1 + d[i]) % 255u;
    s2 = (s2 + s1) % 255u;
  }
  return (uint16_t)((s2 << 8) | s1);
}

// Quantise to uint16 across the box. Rounding is explicit and identical everywhere: no libm, no
// implementation-defined float-to-int behaviour on out-of-range values, because the clamp happens
// in float first.
inline uint16_t quant(float v, float lo, float invSize) {
  const float t = pclamp((v - lo) * invSize, 0.0f, 1.0f);
  return (uint16_t)(t * 65535.0f + 0.5f);
}
inline float dequant(uint16_t q, float lo, float size) {
  return lo + ((float)q * (1.0f / 65535.0f)) * size;
}

inline int8_t quantVel(float v) {
  const float t = pclamp(v * (1.0f / kFrameVelScale), -127.0f, 127.0f);
  return (int8_t)(t >= 0.0f ? t + 0.5f : t - 0.5f);
}

}  // namespace

uint16_t geometryHash(const Geometry& g) {
  // Panel count, size and the baked basis of every panel. Enough to distinguish a 32x32 cube from
  // a 64x64 one, a cube from a slab, and any difference in mount geometry -- which is exactly the
  // set of mismatches that would otherwise render silently wrong.
  uint16_t h = (uint16_t)(0xA5A5u ^ (uint16_t)g.count());
  for (int i = 0; i < g.count(); ++i) {
    const Panel& p = g.at(i);
    const uint8_t bytes[] = {
        (uint8_t)(p.w & 0xFF), (uint8_t)(p.w >> 8), (uint8_t)(p.h & 0xFF), (uint8_t)(p.h >> 8),
    };
    h = fletcher16(bytes, 4, h);
    // The basis as raw float bytes: identical builds produce identical bits, and any real
    // difference in pitch or orientation changes them.
    const float f[9] = {p.origin.x, p.origin.y, p.origin.z, p.u.x, p.u.y, p.u.z,
                        p.n.x,      p.n.y,      p.n.z};
    h = fletcher16((const uint8_t*)f, (int)sizeof(f), h);
  }
  return h;
}

int rleEncode(const uint8_t* src, int n, uint8_t* out, int cap) {
  int o = 0, i = 0;
  while (i < n) {
    // How long is the run of identical bytes starting here?
    int run = 1;
    while (i + run < n && src[i + run] == src[i] && run < 127) ++run;

    if (run >= 2) {
      if (o + 2 > cap) return 0;
      out[o++] = (uint8_t)run;  // high bit clear: repeat
      out[o++] = src[i];
      i += run;
    } else {
      // Gather literals until a run of 2 or more appears, or we hit 127.
      int lit = 1;
      while (i + lit < n && lit < 127) {
        if (i + lit + 1 < n && src[i + lit] == src[i + lit + 1]) break;
        ++lit;
      }
      if (o + 1 + lit > cap) return 0;
      out[o++] = (uint8_t)(0x80 | lit);  // high bit set: literal
      for (int k = 0; k < lit; ++k) out[o++] = src[i + k];
      i += lit;
    }
  }
  return o;
}

int rleDecode(const uint8_t* src, int n, uint8_t* dst, int cap) {
  int i = 0, o = 0;
  while (i < n) {
    const uint8_t ctrl = src[i++];
    const int count = ctrl & 0x7F;
    if (count == 0) return -1;  // a zero-length run is malformed, not a no-op
    if (ctrl & 0x80) {
      if (i + count > n || o + count > cap) return -1;
      for (int k = 0; k < count; ++k) dst[o++] = src[i++];
    } else {
      if (i >= n || o + count > cap) return -1;
      const uint8_t v = src[i++];
      for (int k = 0; k < count; ++k) dst[o++] = v;
    }
  }
  return o;
}

int encodeFrame(const FrameHeader& hdr, ParticleView p, HeatView f, const Aabb& box, uint8_t* out,
                int cap) {
  const Vec3 size = box.size();
  if (size.x <= 0.0f || size.y <= 0.0f || size.z <= 0.0f) return 0;
  const float invx = 1.0f / size.x, invy = 1.0f / size.y, invz = 1.0f / size.z;

  const int n = p.n;
  const int bodyStart = kFrameHeaderBytes;
  if (bodyStart + n * kFrameBytesPerParticle > cap) return 0;

  int o = bodyStart;
  for (int i = 0; i < n; ++i) {
    put16(out + o, quant(p.x[i], box.lo.x, invx)); o += 2;
    put16(out + o, quant(p.y[i], box.lo.y, invy)); o += 2;
    put16(out + o, quant(p.z[i], box.lo.z, invz)); o += 2;
    out[o++] = (uint8_t)quantVel(p.vx[i]);
    out[o++] = (uint8_t)quantVel(p.vy[i]);
    out[o++] = (uint8_t)quantVel(p.vz[i]);
    out[o++] = p.mat[i];
  }

  // Heat, RLE'd. An empty view writes nothing and the receiver skips the whole splat pass.
  int heatBytes = 0;
  if (!f.empty) {
    const int cells = f.dim.x * f.dim.y * f.dim.z;
    heatBytes = rleEncode(f.cells, cells, out + o, cap - o);
    if (heatBytes == 0 && cells > 0) return 0;  // did not fit
    o += heatBytes;
  }

  // Header last, so heatBytes is known, then the checksum over everything before it.
  put16(out + 0, kFrameMagic);
  out[2] = kFrameVersion;
  out[3] = hdr.flags;
  put32(out + 4, hdr.step);
  put16(out + 8, (uint16_t)n);
  put16(out + 10, (uint16_t)heatBytes);
  put16(out + 12, hdr.geomHash);
  out[14] = hdr.paletteA;
  out[15] = hdr.paletteB;
  out[16] = hdr.blend;
  out[17] = 0;
  const uint16_t sum = fletcher16(out, 18, 0);
  put16(out + 18, fletcher16(out + bodyStart, o - bodyStart, sum));
  return o;
}

bool decodeFrame(const uint8_t* in, int len, const Aabb& box, uint16_t expectGeomHash,
                 RenderParticles& rp, HeatBuffer& hb, FrameHeader& hdr) {
  if (len < kFrameHeaderBytes) return false;
  if (get16(in + 0) != kFrameMagic) return false;
  if (in[2] != kFrameVersion) return false;

  FrameHeader h;
  h.flags = in[3];
  h.step = get32(in + 4);
  h.particles = get16(in + 8);
  h.heatBytes = get16(in + 10);
  h.geomHash = get16(in + 12);
  h.paletteA = in[14];
  h.paletteB = in[15];
  h.blend = in[16];

  // A mismatched build is rejected before anything is drawn. Without this the two ends would
  // disagree about panel size or pitch and every particle would land in the wrong place -- which
  // looks like a physics bug and is not one.
  if (h.geomHash != expectGeomHash) return false;
  if (h.particles > kMaxParticles) return false;

  const int bodyStart = kFrameHeaderBytes;
  const int particleBytes = (int)h.particles * kFrameBytesPerParticle;
  const int expect = bodyStart + particleBytes + (int)h.heatBytes;
  if (len < expect) return false;  // truncated

  const uint16_t sum = fletcher16(in, 18, 0);
  if (fletcher16(in + bodyStart, expect - bodyStart, sum) != get16(in + 18)) return false;

  // Everything validated: only now is any state touched, so a rejected frame leaves the node
  // holding its previous one.
  const Vec3 size = box.size();
  rp.clear();
  int o = bodyStart;
  for (int i = 0; i < (int)h.particles; ++i) {
    const uint16_t qx = get16(in + o); o += 2;
    const uint16_t qy = get16(in + o); o += 2;
    const uint16_t qz = get16(in + o); o += 2;
    const Vec3 pos{dequant(qx, box.lo.x, size.x), dequant(qy, box.lo.y, size.y),
                   dequant(qz, box.lo.z, size.z)};
    const Vec3 vel{(float)(int8_t)in[o] * kFrameVelScale,
                   (float)(int8_t)in[o + 1] * kFrameVelScale,
                   (float)(int8_t)in[o + 2] * kFrameVelScale};
    o += 3;
    rp.add(pos, vel, in[o++]);
  }

  hb.clear();
  if (h.heatBytes > 0) {
    const int got = rleDecode(in + o, (int)h.heatBytes, hb.cells(), hb.cellCount());
    if (got != hb.cellCount()) return false;  // grid disagreement, or a malformed stream
    uint8_t peak = 0;
    const uint8_t* c = hb.cells();
    for (int i = 0; i < got; ++i)
      if (c[i] > peak) peak = c[i];
    hb.setPeak(peak);
  }

  hdr = h;
  return true;
}

}  // namespace partsim
