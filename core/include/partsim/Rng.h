#pragma once
#include <cstddef>
#include <cstdint>

namespace partsim {

// xoshiro128** -- 16 bytes of state, no libc, identical on every target.
// Explicitly not std::mt19937: its distribution helpers go through double, which drags in
// libm and breaks cross-target determinism. Never seeded from a clock.
class Rng {
 public:
  explicit Rng(uint32_t seed = 0x9E3779B9u) { reseed(seed); }

  void reseed(uint32_t seed) {
    // SplitMix32 to spread a single word across the four state words; a state of all
    // zeros is a fixed point of xoshiro and must be avoided.
    for (int i = 0; i < 4; ++i) {
      seed += 0x9E3779B9u;
      uint32_t z = seed;
      z = (z ^ (z >> 16)) * 0x85EBCA6Bu;
      z = (z ^ (z >> 13)) * 0xC2B2AE35u;
      s_[i] = z ^ (z >> 16);
    }
    if ((s_[0] | s_[1] | s_[2] | s_[3]) == 0u) s_[0] = 1u;
  }

  uint32_t next() {
    const uint32_t result = rotl(s_[1] * 5u, 7) * 9u;
    const uint32_t t = s_[1] << 9;
    s_[2] ^= s_[0];
    s_[3] ^= s_[1];
    s_[1] ^= s_[2];
    s_[0] ^= s_[3];
    s_[2] ^= t;
    s_[3] = rotl(s_[3], 11);
    return result;
  }

  // Uniform in [0,1). Built by stuffing 23 random bits into a mantissa rather than
  // dividing or calling ldexp -- exact, and no float division.
  float nextFloat() {
    union { uint32_t u; float f; } v;
    v.u = 0x3F800000u | (next() >> 9);  // [1,2)
    return v.f - 1.0f;
  }

  // Uniform in [-1,1).
  float nextSigned() { return nextFloat() * 2.0f - 1.0f; }

  // Uniform in [0,n) for n > 0.
  int nextInt(int n) { return (int)(((uint64_t)next() * (uint64_t)n) >> 32); }

 private:
  static uint32_t rotl(uint32_t x, int k) { return (x << k) | (x >> (32 - k)); }
  uint32_t s_[4];
};

// FNV-1a over raw bytes. Used by the golden determinism test to compare host and WASM
// particle state bit-for-bit.
inline uint64_t fnv1a(const void* data, size_t n, uint64_t h = 1469598103934665603ull) {
  const uint8_t* p = (const uint8_t*)data;
  for (size_t i = 0; i < n; ++i) {
    h ^= p[i];
    h *= 1099511628211ull;
  }
  return h;
}

}  // namespace partsim
