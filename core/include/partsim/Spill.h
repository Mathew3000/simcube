#pragma once
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

}  // namespace partsim
