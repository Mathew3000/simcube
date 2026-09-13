#pragma once
#include <cstdint>

#include "partsim/Simulation.h"
#include "partsim/Spill.h"

namespace partsim {

// The carrier for spill packets, injected the way Parallel is (see Parallel.h).
//
// core/ has no platform headers by construction, and the chain has to run over three different
// carriers -- ESP-NOW between cubes, a JS array between browser beakers, a loopback in the tests.
// Putting the interface here rather than in platform/app/ is deliberate: the tests link only
// partsim_core, so a seam defined above core would leave the pump below it untestable, which is
// exactly the code that most needs a test. It is the same argument that put SimFrame in core.
class SpillTransport {
 public:
  virtual ~SpillTransport() = default;

  // Fire and forget. False means the carrier refused it outright; a packet lost in the air
  // returns true here and is caught downstream by the receiver's cumulative arithmetic instead.
  virtual bool send(const uint8_t* bytes, int len) = 0;

  // Copies the next packet, oldest first, and returns its length -- 0 when nothing is waiting.
  // Never blocks: a beaker whose upstream has gone quiet simply receives nothing.
  virtual int poll(uint8_t* buf, int cap) = 0;

  virtual const char* name() const { return "null"; }

  // One line of whatever the carrier alone knows -- a radio's refused sends, its ring overruns,
  // who it last heard from. Formatted by the carrier because nothing above it can know what a
  // carrier has to say, and returned as text rather than as counters for the same reason.
  virtual const char* diagnostic() const { return ""; }
};

// A carrier that carries nothing, so a single cube runs the identical code path with no peer.
SpillTransport& nullSpillTransport();

// One beaker's end of the chain: the outbound queue onto the wire, and the wire into the beaker.
//
// Call pump() once per frame, immediately after advance() or the step loop, and BEFORE the next
// one -- Simulation clears its spill queue at the top of each frame, so a pump that runs on a
// lower-priority thread would send whatever happened to survive the race rather than the frame.
class SpillChain {
 public:
  struct Stats {
    uint32_t packetsOut = 0, packetsIn = 0;
    uint32_t particlesOut = 0, particlesIn = 0;
    uint32_t bad = 0;       // packets that failed magic, version, length or checksum
    uint32_t rejected = 0;  // arrivals the receiving pool had no room for -- volume lost
    uint32_t madeUp = 0;    // particles re-created to cover a radio loss
    // Particles the beaker spilled that this pump never saw, because the caller cleared the queue
    // between them. NOT a radio fault and not detectable at the far end: the receiver makes them
    // up out of clones and the volume comes out right, so only the sender can notice.
    //
    // It is here because it happened. The master steps twice per frame and stepFixed() clears the
    // queue per STEP, so a pump after the loop caught half the pour -- 98 of 198 -- and every
    // count downstream still added up.
    uint32_t unsent = 0;
    // Packets from a cube that is not this one's upstream neighbour. Expected, not an error: on a
    // broadcast every cube hears the whole ring. A count of zero on a ring of three or more means
    // the addressing is not doing anything, which is worth being able to see.
    uint32_t foreign = 0;
  };

  void pump(Simulation& sim, SpillTransport& t);

  // Where this cube sits in the ring, and how long the ring is.
  //
  // A chain is an ORDER the user configures, not a set of pairings: the radio broadcasts, so this
  // is what decides whose pour a cube is standing under. Unset (or a length below 2) accepts every
  // packet from anyone, which is what a bench with two boards wants and what every existing caller
  // gets without saying anything.
  //
  // Changing it re-baselines the receiver: the cumulative count it was tracking belonged to a
  // different sender, and carrying it over would read as one enormous shortfall.
  void setChainPosition(int id, int length);
  int chainId() const { return id_; }
  int chainLength() const { return len_; }
  // Whose packets this cube takes. -1 when it takes everyone's.
  int upstream() const { return len_ > 1 ? (id_ + len_ - 1) % len_ : -1; }

  const Stats& stats() const { return stats_; }
  // Particles known lost and not yet re-created. Drains over subsequent frames rather than all at
  // once: an outage of a few seconds is hundreds of particles, and injecting them in one step
  // would put a slug of fluid in the beaker instead of a pour.
  uint32_t owed() const { return owed_; }
  uint32_t shortfall() const { return rx_.shortfall(); }
  uint32_t lastSeq() const { return rx_.lastSeq(); }

 private:
  SpillReceiver rx_;
  Stats stats_;
  uint32_t seq_ = 0;
  int id_ = 0;
  int len_ = 0;
  uint32_t packetised_ = 0;  // cumulative particles taken out of the outbound queue
  uint32_t owed_ = 0;
  // The last arrival seen, which is the template a made-up particle is copied from: the lost
  // packet came from the same stream as this one, so its colour and its trajectory are the best
  // available guess and far better than a default.
  SpillParticle last_{};
  bool haveLast_ = false;

  // Members rather than locals: core/ allocates nothing after init and the step task's stack is
  // 6KB, so a 750-byte frame in the middle of it is worth avoiding.
  uint8_t packet_[kSpillMaxPayload];
  SpillParticle decoded_[kSpillMaxPerPacket];
};

}  // namespace partsim
