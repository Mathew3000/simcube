#include "partsim/Spill.h"

namespace partsim {
namespace {

// Same shape as SimFrame's helpers, and deliberately so: both formats quantise a position across
// the container box and a velocity into a byte, and two conventions for that would be two things
// to keep in step. They are duplicated rather than shared because SimFrame's are file-local and
// exporting them would widen that file's interface for one caller.
uint16_t fletcher16(const uint8_t* d, int n, uint16_t seed = 0) {
  uint16_t s1 = (uint16_t)(seed & 0xFF), s2 = (uint16_t)(seed >> 8);
  for (int i = 0; i < n; ++i) {
    s1 = (uint16_t)((s1 + d[i]) % 255);
    s2 = (uint16_t)((s2 + s1) % 255);
  }
  return (uint16_t)((s2 << 8) | s1);
}

void put16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}
uint16_t get16(const uint8_t* p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
void put32(uint8_t* p, uint32_t v) {
  put16(p, (uint16_t)(v & 0xFFFF));
  put16(p + 2, (uint16_t)(v >> 16));
}
uint32_t get32(const uint8_t* p) { return (uint32_t)get16(p) | ((uint32_t)get16(p + 2) << 16); }

uint16_t quant(float v, float lo, float invSize) {
  const float t = pclamp((v - lo) * invSize, 0.0f, 1.0f);
  return (uint16_t)(t * 65535.0f + 0.5f);
}
float dequant(uint16_t q, float lo, float size) {
  return lo + ((float)q * (1.0f / 65535.0f)) * size;
}

// Velocity scale. A spilled particle is in free fall through the open top, so its speed is bounded
// by the solver's own CFL clamp rather than by anything this format chooses.
constexpr float kSpillVelScale = 0.5f;
int8_t quantVel(float v) {
  const float t = pclamp(v * (1.0f / kSpillVelScale), -127.0f, 127.0f);
  return (int8_t)(t >= 0.0f ? t + 0.5f : t - 0.5f);
}
float dequantVel(int8_t q) { return (float)q * kSpillVelScale; }

}  // namespace

int encodeSpill(const SpillHeader& h, const SpillParticle* items, int n, const Aabb& box,
                uint8_t* out, int cap) {
  if (n < 0 || n > kSpillMaxPerPacket) return 0;
  const int need = kSpillHeaderBytes + n * kSpillBytesPerParticle;
  if (need > cap || need > kSpillMaxPayload) return 0;

  const Vec3 size = box.size();
  if (size.x <= 0.0f || size.y <= 0.0f || size.z <= 0.0f) return 0;
  const float ix = 1.0f / size.x, iy = 1.0f / size.y, iz = 1.0f / size.z;

  int o = kSpillHeaderBytes;
  for (int i = 0; i < n; ++i) {
    const SpillParticle& s = items[i];
    put16(out + o + 0, quant(s.pos.x, box.lo.x, ix));
    put16(out + o + 2, quant(s.pos.y, box.lo.y, iy));
    put16(out + o + 4, quant(s.pos.z, box.lo.z, iz));
    out[o + 6] = (uint8_t)quantVel(s.vel.x);
    out[o + 7] = (uint8_t)quantVel(s.vel.y);
    out[o + 8] = (uint8_t)quantVel(s.vel.z);
#if PARTSIM_ENABLE_CHROMA
    put16(out + o + 9, s.cr);
    put16(out + o + 11, s.cg);
#else
    // The field is absent from the struct in this build, but it stays in the FORMAT: a cube built
    // without chroma must still be decodable by one built with it, or a mixed chain silently
    // misreads every packet from the far side.
    put16(out + o + 9, 0);
    put16(out + o + 11, 0);
#endif
    o += kSpillBytesPerParticle;
  }

  put16(out + 0, kSpillMagic);
  out[2] = kSpillVersion;
  out[3] = h.from;  // the sender's chain position; 0 is what "reserved" used to encode
  put32(out + 4, h.seq);
  // The sender's cumulative spill count, and the field the receiver's shortfall arithmetic is
  // built on: it advances by every particle the sender has ever released, so a receiver that has
  // taken fewer knows exactly how many went missing. Without it a dropped packet is invisible.
  put32(out + 8, h.totalOut);
  put16(out + 12, (uint16_t)n);
  put16(out + 14, fletcher16(out, 14));
  return need;
}

int decodeSpill(const uint8_t* in, int len, const Aabb& box, SpillParticle* out, int cap,
                SpillHeader& h) {
  if (len < kSpillHeaderBytes) return -1;
  if (get16(in) != kSpillMagic) return -1;
  if (in[2] != kSpillVersion) return -1;
  if (fletcher16(in, 14) != get16(in + 14)) return -1;

  h.from = in[3];
  h.seq = get32(in + 4);
  h.totalOut = get32(in + 8);
  h.count = get16(in + 12);
  if (h.count > kSpillMaxPerPacket) return -1;
  const int need = kSpillHeaderBytes + (int)h.count * kSpillBytesPerParticle;
  if (len < need) return -1;

  const int n = (int)h.count < cap ? (int)h.count : cap;
  const Vec3 size = box.size();
  int o = kSpillHeaderBytes;
  for (int i = 0; i < n; ++i) {
    SpillParticle& s = out[i];
    s.pos = Vec3{dequant(get16(in + o + 0), box.lo.x, size.x),
                 dequant(get16(in + o + 2), box.lo.y, size.y),
                 dequant(get16(in + o + 4), box.lo.z, size.z)};
    s.vel = Vec3{dequantVel((int8_t)in[o + 6]), dequantVel((int8_t)in[o + 7]),
                 dequantVel((int8_t)in[o + 8])};
#if PARTSIM_ENABLE_CHROMA
    s.cr = get16(in + o + 9);
    s.cg = get16(in + o + 11);
#endif
    o += kSpillBytesPerParticle;
  }
  return n;
}

}  // namespace partsim
