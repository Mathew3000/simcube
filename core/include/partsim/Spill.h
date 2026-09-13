#pragma once
#include <cstdint>

#include "partsim/Config.h"
#include "partsim/Types.h"

namespace partsim {

// A particle that has left one beaker through its open top, on its way into the next.
//
// This type is defined here, in core/, and not in either of the two places that use it, because
// TWO pieces of work meet on it: the spill queue that produces and consumes these (M4-B) and the
// ESP-NOW link that carries them between cubes (M4-D). Letting each define its own would mean two
// structurally identical types that agree until somebody reorders one -- the same mistake
// Lsm6dsox::Raw was fixed for (DECISIONS.md D42).
//
// Deliberately NOT a Particle. A spilled particle has no predicted position, no Lagrange
// multiplier and no neighbours; it is in flight between two simulations and all the receiver needs
// is where it was, how fast it was going, and what colour it is.
struct SpillParticle {
  // Position and velocity in the SOURCE beaker's object space, which is also the destination's --
  // every cube shares the same 32-unit box, so a particle leaving one corner arrives in the
  // corresponding corner of the next. That is what makes a chain read as pouring rather than as
  // teleporting, and it is why these are not renormalised on the way out.
  Vec3 pos;
  Vec3 vel;
#if PARTSIM_ENABLE_CHROMA
  // Dye, in the same 8.8 fixed point Particles uses. Carried rather than re-derived: the whole
  // point of a chain is that red poured into blue arrives still red.
  uint16_t cr;
  uint16_t cg;
#endif
};

// How many in-flight particles a beaker will hold before it starts dropping them.
//
// A cap rather than a growable list, because core/ allocates nothing after init
// (tests/test_noalloc.cpp). Sized against the worst case that matters: a full beaker inverted
// empties over several frames, not in one, because the open face is one sixth of the surface and
// the solver's velocity clamp bounds how fast anything crosses it.
#ifndef PARTSIM_MAX_SPILL
#define PARTSIM_MAX_SPILL 128
#endif
constexpr int kMaxSpill = PARTSIM_MAX_SPILL;

// The outbound and inbound queue for one beaker.
//
// Dropping on overflow is deliberate and must be COUNTED, not silent: in a closed ring a lost
// particle is volume that never comes back, and the symptom -- beakers slowly emptying over
// minutes -- looks like a physics leak rather than a full buffer. `dropped` is what tells the two
// apart, and what the receiver's shortfall accounting is built on.
struct SpillQueue {
  SpillParticle items[kMaxSpill];
  int count = 0;
  // Cumulative, never reset by drain(): this is a sequence number, so a receiver can tell "nothing
  // spilled" from "I missed a packet". See M4-D.
  uint32_t totalOut = 0;
  uint32_t dropped = 0;

  void clear() { count = 0; }

  bool push(const SpillParticle& s) {
    ++totalOut;
    if (count >= kMaxSpill) {
      ++dropped;
      return false;
    }
    items[count++] = s;
    return true;
  }
};

// --- the wire format between chained cubes ---------------------------------------------------
//
// ESP-NOW carries at most 250 bytes per packet, which is the constraint everything below is shaped
// by. Particles are quantised exactly the way SimFrame does it -- uint16 position across the
// container box, int8 velocity -- because the two formats face the same problem and a second
// convention would be a second thing to get wrong.
//
// Header is 16 bytes: magic(2), version(1), from(1), seq(4), totalOut(4), count(2), and a
// CRC-16(2) over the ENTIRE packet -- header and payload, with the checksum field itself zeroed. Sixteen rather than twelve costs nothing -- both leave
// room for the same 18 particles, because 13 does not divide the difference -- and twelve had no
// room for totalOut, which is the field the entire shortfall mechanism is built on.
constexpr uint16_t kSpillMagic = 0x5350;  // 'SP'
// 2: the checksum covers the whole packet and is a CRC-16 rather than a Fletcher-16. Version 1
// packets are rejected outright, which is the right outcome for a mixed chain -- the alternative
// is one cube reading another's dye through a checksum with a known blind spot.
constexpr uint8_t kSpillVersion = 2;
constexpr int kSpillHeaderBytes = 16;
constexpr int kSpillBytesPerParticle = 6 + 3 + 4;  // pos(3x uint16), vel(3x int8), cr+cg
constexpr int kSpillMaxPayload = 250;              // ESP-NOW's limit, not ours
constexpr int kSpillMaxPerPacket = (kSpillMaxPayload - kSpillHeaderBytes) / kSpillBytesPerParticle;

struct SpillHeader {
  uint32_t seq = 0;       // packet counter, so a receiver can spot a gap
  uint32_t totalOut = 0;  // the SENDER's cumulative spill count at the end of this packet
  uint16_t count = 0;     // particles in this packet
  // The sender's POSITION IN THE CHAIN, in the byte the header has always reserved for flags.
  //
  // The radio is a broadcast: every cube in earshot hears every packet. With two cubes that is
  // harmless, and with three it is a volume leak in the opposite direction from a dropped packet
  // -- each cube would inject what BOTH of the others poured, and the ring would fill up out of
  // nothing. A receiver takes packets from its upstream neighbour and ignores the rest.
  //
  // Zero by default, which is what this byte encoded before it meant anything, so a cube that has
  // never been told its position speaks and is heard exactly as it was.
  uint8_t from = 0;
};

// Returns bytes written, or 0 if it did not fit or n exceeds kSpillMaxPerPacket.
int encodeSpill(const SpillHeader& h, const SpillParticle* items, int n, const Aabb& box,
                uint8_t* out, int cap);

// Returns the number of particles decoded, or -1 if the packet is not one of ours. Validates
// magic, version, length and checksum BEFORE writing anything into `out`.
int decodeSpill(const uint8_t* in, int len, const Aabb& box, SpillParticle* out, int cap,
                SpillHeader& h);

// Tracks one inbound link and reports how many particles went missing.
//
// A dropped packet in a closed ring is volume that never comes back, and the symptom -- beakers
// slowly emptying over minutes -- reads as a physics leak rather than as a lost radio frame. The
// sender's cumulative totalOut is what tells the two apart: it advances by exactly the number of
// particles it has ever spilled, so a receiver that has seen fewer knows the difference and can
// make it up.
// The largest shortfall one packet may report, and therefore the most volume a single arrival can
// ask to have manufactured.
//
// A sender releases at most kMaxSpill particles in a step, so a plausible outage is tens to a few
// hundred. A number far above that is not an outage: it is a corrupted counter or a sender that
// restarted, and one flipped byte in totalOut asked for 65 535 particles -- which the chain would
// then dutifully create, eighteen a frame, for hours, while the beaker stayed plausibly full
// because it poured out of its own top at roughly the rate it manufactured clones.
//
// Making up a plausible loss is worth doing; manufacturing an implausible one is worse than losing
// it. Above this the receiver re-baselines and counts a resync instead.
constexpr uint32_t kMaxShortfallPerPacket = (uint32_t)kMaxSpill;

class SpillReceiver {
 public:
  // Call with each decoded header. Returns how many particles were missed since the last packet --
  // zero in the normal case. The first packet ever seen establishes the baseline and reports 0,
  // because a receiver joining a running chain has not "lost" the history before it.
  uint32_t note(const SpillHeader& h, int decoded) {
    if (!started_) {
      started_ = true;
      seen_ = h.totalOut;
      lastSeq_ = h.seq;
      return 0;
    }
    lastSeq_ = h.seq;
    const uint32_t expected = h.totalOut;         // what the sender has spilled in total
    seen_ += (uint32_t)decoded;                   // what this receiver has actually taken
    if (expected < seen_) {
      // The sender went BACKWARDS, which means it restarted -- init() resets the cumulative
      // counters (D60). Re-baseline rather than returning 0 and keeping a count that is now ahead
      // of the sender forever, which would silently disable the shortfall mechanism for good.
      seen_ = expected;
      ++resyncs_;
      return 0;
    }
    if (expected == seen_) return 0;
    uint32_t missing = expected - seen_;
    seen_ = expected;  // do not report the same shortfall twice
    if (missing > kMaxShortfallPerPacket) {
      // Implausible: a corrupt counter, not a lost packet. See kMaxShortfallPerPacket.
      ++resyncs_;
      missing = kMaxShortfallPerPacket;
    }
    shortfall_ += missing;
    return missing;
  }
  uint32_t shortfall() const { return shortfall_; }
  // Packets whose totalOut could not be believed: a restarted sender, or a corrupted counter.
  uint32_t resyncs() const { return resyncs_; }
  uint32_t lastSeq() const { return lastSeq_; }
  bool started() const { return started_; }

 private:
  bool started_ = false;
  uint32_t seen_ = 0;
  uint32_t lastSeq_ = 0;
  uint32_t shortfall_ = 0;
  uint32_t resyncs_ = 0;
};

}  // namespace partsim
