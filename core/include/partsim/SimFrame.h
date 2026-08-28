#pragma once
#include "partsim/RenderState.h"

namespace partsim {

// The wire format between the simulation master and the display nodes.
//
// Lives in core/, like MotionSource and ChainMap, for the same reason: it must be byte-identical
// on the host, in the browser and on the device, so there can only be one implementation. A second
// copy in the firmware would diverge and the symptom would be a face rendering nonsense.
//
// Two things are deliberately NOT in the payload:
//
//  * The container box. Both ends derive it from their own Geometry, which is identical on every
//    node by construction. Sending it would invite the two sides to disagree about it.
//  * Anything the receiver can recompute. The payload is state, not instructions.
//
// But a hash of the geometry IS sent, so a mismatched pair of builds -- a master compiled for
// 32x32 talking to a display compiled for 64x64 -- is rejected rather than rendering everything in
// the wrong place. That is the failure this format most needs to make loud.

constexpr uint16_t kFrameMagic = 0x5053;  // 'PS'
constexpr uint8_t kFrameVersion = 1;
constexpr int kFrameHeaderBytes = 20;

// Bytes per particle on the wire: 3 x uint16 position, 3 x int8 velocity, 1 byte material.
//
// Velocity at int8 resolution is deliberate. It exists only for sub-frame interpolation over at
// most 1/30 s, so its quantisation step of ~0.57 units/s becomes 0.019 units of position error --
// against a rest spacing of 1.5. Full floats would cost 9 more bytes per particle to carry
// precision the renderer cannot express.
constexpr int kFrameBytesPerParticle = 10;

// Velocity scale: the CFL cap is 0.4*h/dt = 72 units/s, so +-127 covers it with margin.
constexpr float kFrameVelScale = 72.0f / 127.0f;

struct FrameHeader {
  uint32_t step = 0;          // the master's step index -- the authoritative clock
  uint16_t particles = 0;
  uint16_t heatBytes = 0;     // RLE payload length; 0 means nothing is burning
  uint16_t geomHash = 0;      // rejects a mismatched build; see above
  uint8_t paletteA = 0;
  uint8_t paletteB = 0;
  uint8_t blend = 0;          // 0..255 crossfade position between the two palettes
  uint8_t flags = 0;
};

// A cheap identity for the panel table, so both ends can agree they mean the same cube.
uint16_t geometryHash(const Geometry& g);

// Worst-case frame size, for sizing transmit and receive buffers. The heat term allows for RLE
// expansion on incompressible data.
constexpr int frameMaxBytes() {
  return kFrameHeaderBytes + kMaxParticles * kFrameBytesPerParticle + kMaxFieldCells +
         kMaxFieldCells / 127 + 2;
}

// Encodes one frame. Returns bytes written, or 0 if `cap` is too small.
//
// `box` is the container, used to quantise positions; pass the same SimVolume box the receiver
// will use. Heat may be an empty view, in which case heatBytes comes back 0.
int encodeFrame(const FrameHeader& hdr, ParticleView p, HeatView f, const Aabb& box, uint8_t* out,
                int cap);

// Decodes into the draw-only containers. Returns false and touches nothing on a bad magic,
// version, geometry hash, checksum, or a truncated buffer -- so a display node holds its previous
// frame rather than drawing garbage.
//
// `hb` must already be init()ed against the same volume; its cell count is checked against the
// decoded grid.
bool decodeFrame(const uint8_t* in, int len, const Aabb& box, uint16_t expectGeomHash,
                 RenderParticles& rp, HeatBuffer& hb, FrameHeader& hdr);

// --- RLE, exposed for testing -----------------------------------------------------------------
// PackBits-style: a control byte with the high bit set introduces a literal run of (n & 0x7F)
// raw bytes; without it, a repeat run of (n & 0x7F) copies of the byte that follows. Runs are
// 1..127 long.
//
// A mostly-cold field is the normal case and compresses hugely: 10648 zero cells become 84
// two-byte runs, 168 bytes. Incompressible data expands by at most one byte per 127.
int rleEncode(const uint8_t* src, int n, uint8_t* out, int cap);
// Returns bytes decoded into dst, or -1 on a malformed stream or overflow.
int rleDecode(const uint8_t* src, int n, uint8_t* dst, int cap);

}  // namespace partsim
