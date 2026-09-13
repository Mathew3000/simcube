#include "partsim/SpillChain.h"

namespace partsim {
namespace {

// A carrier that carries nothing. send() reports success because "the radio refused it" is a
// different fact from "there is no radio", and a single cube must not log a failure every frame.
class NullTransport final : public SpillTransport {
 public:
  bool send(const uint8_t*, int) override { return true; }
  int poll(uint8_t*, int) override { return 0; }
};
NullTransport g_null;

}  // namespace

SpillTransport& nullSpillTransport() { return g_null; }

void SpillChain::pump(Simulation& sim, SpillTransport& t) {
  // No open face is no beaker: nothing can leave, nothing can be let in, and an arrival would be
  // rejected by injectSpill anyway. Returning here rather than polling-and-discarding means a
  // cube in a normal scene simply ignores a chain running around it.
  if (sim.openFace() == kOpenNone) return;

  const Aabb& box = sim.volume().box();
  SpillQueue& q = sim.spill();

  // --- outbound: this frame's spill, in packets of at most kSpillMaxPerPacket ----------------
  for (int i = 0; i < q.count; i += kSpillMaxPerPacket) {
    const int n = (q.count - i < kSpillMaxPerPacket) ? q.count - i : kSpillMaxPerPacket;
    SpillHeader h;
    h.seq = ++seq_;
    // The sender's cumulative count AT THE END OF THIS PACKET, not at the end of the frame --
    // otherwise every packet but the last would claim particles it had not sent yet and the
    // receiver would report a shortfall that closed itself one packet later.
    //
    // q.totalOut counts pushes the queue REFUSED as well as ones it took, which is what makes a
    // sender-side overflow show up on the receiver as a loss to be made up rather than as volume
    // that quietly never existed.
    h.totalOut = q.totalOut - (uint32_t)(q.count - (i + n));
    h.count = (uint16_t)n;
    const int len = encodeSpill(h, q.items + i, n, box, packet_, (int)sizeof packet_);
    if (len <= 0) continue;
    if (t.send(packet_, len)) {
      ++stats_.packetsOut;
      stats_.particlesOut += (uint32_t)n;
    }
  }

  // Everything the beaker has ever spilled is either in a packet, refused by its own queue, or
  // was never offered to this pump at all. The third case is the caller's bug and nothing
  // downstream can see it -- see Stats::unsent.
  packetised_ += (uint32_t)q.count;
  stats_.unsent = q.totalOut - q.dropped - packetised_;

  // --- inbound ------------------------------------------------------------------------------
  int len;
  while ((len = t.poll(packet_, (int)sizeof packet_)) > 0) {
    SpillHeader h;
    const int m = decodeSpill(packet_, len, box, decoded_, kSpillMaxPerPacket, h);
    if (m < 0) {
      ++stats_.bad;  // not ours, or damaged -- decodeSpill validated before writing anything
      continue;
    }
    ++stats_.packetsIn;
    owed_ += rx_.note(h, m);
    for (int k = 0; k < m; ++k) {
      if (sim.injectSpill(decoded_[k]))
        ++stats_.particlesIn;
      else
        ++stats_.rejected;
      last_ = decoded_[k];
      haveLast_ = true;
    }
  }

  // --- making up what the radio lost ---------------------------------------------------------
  //
  // A dropped packet in a closed ring is volume that never comes back, and the symptom -- every
  // beaker slowly emptying -- reads as a physics leak. The lost particles came from the same
  // stream as the ones that did arrive, so the last arrival is the best available template for
  // colour and trajectory; anything else would show as a wrong-coloured slug.
  //
  // Rate-limited to one packet's worth per frame. An outage of a few seconds is hundreds of
  // particles and dumping them into one step reads as a block of fluid appearing, not as a pour.
  if (owed_ > 0 && haveLast_) {
    const int a = sim.volume().openAxis();
    uint32_t budget = owed_ < (uint32_t)kSpillMaxPerPacket ? owed_ : (uint32_t)kSpillMaxPerPacket;
    while (budget-- > 0) {
      // Spread across a lattice in the plane of the open face. NOT cosmetic: PBF's density
      // gradient is zero between two particles at identical positions, so a stack of exact copies
      // would never push itself apart and would sit there as one permanent lump.
      SpillParticle s = last_;
      float p[3] = {s.pos.x, s.pos.y, s.pos.z};
      const uint32_t j = stats_.madeUp;
      int c = 0;
      for (int k = 0; k < 3; ++k) {
        if (k == a) continue;
        const uint32_t lane = (c == 0) ? (j % 5u) : ((j / 5u) % 5u);
        // Offset by a HALF lane, so no member of the lattice lands on the template itself. The
        // whole-lane version had one: the centre cell reproduced the last arrival exactly, and
        // those two then sat on each other permanently.
        p[k] += ((float)(int)lane - 1.5f) * 0.5f * kRestSpacing;
        ++c;
      }
      s.pos = Vec3{p[0], p[1], p[2]};
      if (!sim.injectSpill(s)) break;  // pool full: stay owed and try again next frame
      ++stats_.madeUp;
      --owed_;
    }
  }
}

}  // namespace partsim
