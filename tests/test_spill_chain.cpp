// M4-D: the pump between a beaker's spill queue and the wire.
//
// The wire format itself is covered by test_spill.cpp and the open face by test_beaker_spill.cpp.
// What is only testable here is the thing between them -- who sends what, when, and what a
// receiver does about a packet that never arrived. That last one is the whole reason the chain
// carries a cumulative count: a lost packet in a closed ring is volume that never comes back, and
// the symptom is every beaker slowly emptying, which reads as a physics leak rather than as radio.
#include "check.h"
#include "partsim/SpillChain.h"

using namespace partsim;

namespace {

// A one-way pipe, and a carrier that reads one and writes another.
//
// Two pipes rather than one shared object, because a chain is DIRECTED: ESP-NOW does not loop a
// broadcast back to its sender, so a fixture where both ends pump the same queue would have the
// sender swallow its own packet before the receiver ever polled. (It did, first time round.)
struct Pipe {
  static constexpr int kSlots = 64;
  uint8_t slot[kSlots][kSpillMaxPayload] = {};
  int slotLen[kSlots] = {};
  int head = 0, count = 0;

  bool put(const uint8_t* b, int len) {
    if (count >= kSlots || len > kSpillMaxPayload) return false;
    const int s = (head + count) % kSlots;
    for (int i = 0; i < len; ++i) slot[s][i] = b[i];
    slotLen[s] = len;
    ++count;
    return true;
  }
  int take(uint8_t* out, int cap) {
    if (count == 0) return 0;
    const int len = slotLen[head];
    if (len > cap) return 0;
    for (int i = 0; i < len; ++i) out[i] = slot[head][i];
    head = (head + 1) % kSlots;
    --count;
    return len;
  }
};

// `dropEvery` = 3 loses every third packet ON THE AIR: send() accepts it, the sender counts it,
// and it simply never arrives. That is what ESP-NOW does, and it is why a return value could
// never have detected it.
class Wire final : public SpillTransport {
 public:
  Wire(Pipe* tx, Pipe* rx) : tx_(tx), rx_(rx) {}
  bool send(const uint8_t* b, int len) override {
    ++sent;
    if (dropEvery > 0 && (sent % (uint32_t)dropEvery) == 0) {
      ++dropped;
      return true;  // the radio took it; the air lost it
    }
    return !tx_ || tx_->put(b, len);
  }
  int poll(uint8_t* out, int cap) override { return rx_ ? rx_->take(out, cap) : 0; }

  int dropEvery = 0;
  uint32_t sent = 0, dropped = 0;

 private:
  Pipe* tx_;
  Pipe* rx_;
};

// A broadcast medium: what one cube sends, every OTHER cube hears. That is what the radio does,
// and it is the whole reason a packet has to say who it came from.
struct Bus {
  static constexpr int kNodes = 3;
  Pipe in[kNodes];
  void broadcast(int from, const uint8_t* b, int len) {
    for (int i = 0; i < kNodes; ++i)
      if (i != from) in[i].put(b, len);
  }
};

class BusWire final : public SpillTransport {
 public:
  BusWire(Bus* bus, int me) : bus_(bus), me_(me) {}
  bool send(const uint8_t* b, int len) override {
    bus_->broadcast(me_, b, len);
    return true;
  }
  int poll(uint8_t* out, int cap) override { return bus_->in[me_].take(out, cap); }

 private:
  Bus* bus_;
  int me_;
};

// ~3.2MB each; static storage only.
Simulation g_a, g_b;
Pipe g_pipe;          // A's outbound is B's inbound, and nothing comes back
Wire g_wa{&g_pipe, nullptr};
Wire g_wb{nullptr, &g_pipe};
SpillChain g_ca, g_cb;

// The same deterministic sweep the B tests use: a pure function of the step index, no clock and
// no RNG, so a failure reproduces exactly.
Vec3 tiltAt(int s) {
  const float t = (float)s * 0.031f;
  return normalize(Vec3{fsin(t), fcos(t * 0.41f), fsin(t * 0.67f) * 0.5f}) * kGravityMag;
}

// One frame of a two-cube chain: A pours through the wire into B. A's own inbound is a carrier
// that carries nothing, which is the single-cube path and must not disturb anything.
void chainStep(int s) {
  g_a.setGravityObject(tiltAt(s));
  g_a.stepFixed();
  g_ca.pump(g_a, g_wa);            // A sends into the wire
  g_b.stepFixed();
  g_cb.pump(g_b, g_wb);            // B takes what is waiting on it
}

void resetChain(int dropEvery) {
  CHECK(g_a.init(Simulation::kCube, particlesForFill(600), 0xB0A7u, 32));
  CHECK(g_b.init(Simulation::kCube, particlesForFill(120), 0xB0A8u, 32));
  g_a.setOpenFace(kOpenPosY);
  g_b.setOpenFace(kOpenPosY);
  g_pipe = Pipe{};
  g_wa = Wire{&g_pipe, nullptr};
  g_wb = Wire{nullptr, &g_pipe};
  g_wa.dropEvery = dropEvery;
  g_ca = SpillChain{};
  g_cb = SpillChain{};
}

}  // namespace

TEST(chain_carries_a_pour_from_one_beaker_into_the_next) {
  resetChain(0);
  const int startA = g_a.particleCount(), startB = g_b.particleCount();
  // B sits upright and closed to the world: it only ever receives.
  g_b.setGravityObject(Vec3{0.0f, -kGravityMag, 0.0f});

  for (int s = 0; s < settleSteps(900); ++s) chainStep(s);

  const SpillChain::Stats& sa = g_ca.stats();
  const SpillChain::Stats& sb = g_cb.stats();
  std::printf("       chain: A %d->%d sent %u in %u packets; B %d->%d took %u, rejected %u\n",
              startA, g_a.particleCount(), sa.particlesOut, sa.packetsOut, startB,
              g_b.particleCount(), sb.particlesIn, sb.rejected);

  CHECK(sa.packetsOut > 0u);
  CHECK(sa.particlesOut > 0u);
  // Everything A put on the wire reached B, because nothing was dropped.
  CHECK(sb.particlesIn + sb.rejected == sa.particlesOut);
  CHECK(sb.bad == 0u);
  CHECK(g_cb.shortfall() == 0u);
  CHECK(g_cb.stats().madeUp == 0u);
  // A lost exactly what it spilled, and B holds what it started with plus what it accepted minus
  // what went back out of its own open top -- B is the end of the chain here, so its spill is
  // lost the way a single cube's is.
  CHECK((uint32_t)(startA - g_a.particleCount()) == g_a.spill().totalOut);
  CHECK((uint32_t)g_b.particleCount() + g_b.spill().totalOut
        == (uint32_t)startB + sb.particlesIn);
  // A's own pump found nothing waiting for it -- the wire is one-way in this fixture.
  CHECK(sa.particlesIn == 0u);
}

TEST(chain_splits_a_big_frame_and_its_totals_stay_monotone) {
  // A frame can spill more than one packet holds, and the header's cumulative count has to mean
  // "at the end of THIS packet". Getting that wrong is invisible until a receiver reports a
  // shortfall that closes itself again one packet later.
  resetChain(0);
  SpillQueue q;
  for (int i = 0; i < kSpillMaxPerPacket * 2 + 3; ++i) {
    SpillParticle p{};
    p.pos = Vec3{-10.0f + (float)i * 0.25f, 15.0f, 0.0f};
    p.vel = Vec3{0.0f, 6.0f, 0.0f};
    CHECK(q.push(p));
  }
  g_a.spill() = q;
  g_ca.pump(g_a, g_wa);
  CHECK(g_ca.stats().packetsOut == 3u);
  CHECK(g_ca.stats().particlesOut == (uint32_t)q.count);

  uint32_t last = 0;
  int seen = 0;
  uint8_t buf[kSpillMaxPayload];
  SpillParticle out[kSpillMaxPerPacket];
  int len;
  while ((len = g_pipe.take(buf, sizeof buf)) > 0) {
    SpillHeader h;
    const int m = decodeSpill(buf, len, g_a.volume().box(), out, kSpillMaxPerPacket, h);
    CHECK(m > 0);
    seen += m;
    CHECK(h.totalOut > last);
    CHECK(h.totalOut == (uint32_t)seen);  // cumulative at the end of this packet, exactly
    last = h.totalOut;
  }
  CHECK(seen == q.count);
  CHECK(last == q.totalOut);
}

TEST(chain_makes_up_what_the_air_lost) {
  // Every third packet vanishes. Without the make-up the ring drains; with it the receiver
  // re-creates the missing volume from the stream it can see.
  resetChain(3);
  g_b.setGravityObject(Vec3{0.0f, -kGravityMag, 0.0f});
  for (int s = 0; s < settleSteps(900); ++s) chainStep(s);

  const SpillChain::Stats& sa = g_ca.stats();
  const SpillChain::Stats& sb = g_cb.stats();
  std::printf("       lossy chain: sent %u in %u packets, air dropped %u; B took %u, made up %u,"
              " still owed %u\n",
              sa.particlesOut, sa.packetsOut, g_wa.dropped, sb.particlesIn, sb.madeUp, g_cb.owed());

  CHECK(g_wa.dropped > 0u);
  CHECK(g_cb.shortfall() > 0u);
  CHECK(sb.madeUp > 0u);
  // The receiver knows about every particle the sender ever spilled: what it took, what it made
  // up, what it had no room for, and what it still owes must account for all of it.
  CHECK(sb.particlesIn + sb.madeUp + sb.rejected + g_cb.owed() >= g_cb.shortfall());
  // Made-up particles are bounded by what was actually lost -- it invents volume to replace
  // volume, never more.
  CHECK(sb.madeUp <= g_cb.shortfall());
}

TEST(chain_does_not_stack_made_up_particles_on_one_spot) {
  // PBF's density gradient between two particles at identical positions is zero, so a stack of
  // exact copies would never push itself apart. It would sit in the beaker as one permanent lump
  // and no count-based assertion would ever see it.
  resetChain(0);
  CHECK(g_b.init(Simulation::kCube, 0, 0xB0A8u, 32));
  g_b.setOpenFace(kOpenPosY);

  // One packet claiming a cumulative total far ahead of what it carries: the receiver establishes
  // its baseline on the first packet, so it takes two to report a shortfall.
  SpillParticle p{};
  p.pos = Vec3{-4.0f, 15.0f, 3.0f};
  p.vel = Vec3{0.0f, 5.0f, 0.0f};
  uint8_t buf[kSpillMaxPayload];
  SpillHeader h;
  h.seq = 1; h.totalOut = 1; h.count = 1;
  int len = encodeSpill(h, &p, 1, g_b.volume().box(), buf, sizeof buf);
  CHECK(len > 0);
  CHECK(g_pipe.put(buf, len));
  g_cb.pump(g_b, g_wb);
  h.seq = 2; h.totalOut = 30; h.count = 1;   // 28 particles went missing between the two
  p.pos = Vec3{6.0f, 15.0f, -2.0f};          // a different arrival, so the two real ones differ
  len = encodeSpill(h, &p, 1, g_b.volume().box(), buf, sizeof buf);
  CHECK(g_pipe.put(buf, len));
  g_cb.pump(g_b, g_wb);

  CHECK(g_cb.shortfall() == 28u);
  CHECK(g_cb.stats().madeUp == (uint32_t)kSpillMaxPerPacket);  // rate-limited to one packet's worth
  CHECK(g_cb.owed() == 28u - (uint32_t)kSpillMaxPerPacket);

  // No two particles in the beaker share a position.
  const int n = g_b.particleCount();
  CHECK(n == 2 + kSpillMaxPerPacket);
  int coincident = 0;
  for (int i = 0; i < n; ++i)
    for (int j = i + 1; j < n; ++j) {
      const Vec3 a = g_b.particles().pos(i), b = g_b.particles().pos(j);
      if (a.x == b.x && a.y == b.y && a.z == b.z) ++coincident;
    }
  CHECK(coincident == 0);
}

TEST(chain_counts_a_damaged_packet_and_injects_nothing) {
  resetChain(0);
  CHECK(g_b.init(Simulation::kCube, 0, 0xB0A8u, 32));
  g_b.setOpenFace(kOpenPosY);

  SpillParticle p{};
  p.pos = Vec3{0.0f, 15.0f, 0.0f};
  p.vel = Vec3{0.0f, 5.0f, 0.0f};
  uint8_t buf[kSpillMaxPayload];
  SpillHeader h;
  h.seq = 1; h.totalOut = 1; h.count = 1;
  const int len = encodeSpill(h, &p, 1, g_b.volume().box(), buf, sizeof buf);
  CHECK(len > 0);
  buf[len - 1] ^= 0x5A;  // corrupt a payload byte; the header checksum will not see it
  buf[4] ^= 0x01;        // ...so corrupt the header too, which it will
  CHECK(g_pipe.put(buf, len));
  g_cb.pump(g_b, g_wb);

  CHECK(g_cb.stats().bad == 1u);
  CHECK(g_cb.stats().packetsIn == 0u);
  CHECK(g_b.particleCount() == 0);
}

TEST(chain_leaves_a_closed_cube_alone) {
  // Beaker mode is a mode. A cube in a normal scene must ignore a chain running around it --
  // neither sending nor consuming, so the same firmware can run either.
  resetChain(0);
  CHECK(g_b.init(Simulation::kCube, 64, 0xB0A8u, 32));
  // init() deliberately does NOT close an open face -- a reset refills a beaker rather than
  // sealing it (test_beaker_spill.cpp says so) -- so closing it is an explicit act here.
  g_b.setOpenFace(kOpenNone);
  CHECK(g_b.openFace() == kOpenNone);
  const int before = g_b.particleCount();

  SpillParticle p{};
  p.pos = Vec3{0.0f, 15.0f, 0.0f};
  p.vel = Vec3{0.0f, 5.0f, 0.0f};
  uint8_t buf[kSpillMaxPayload];
  SpillHeader h;
  h.seq = 1; h.totalOut = 1; h.count = 1;
  const int len = encodeSpill(h, &p, 1, g_b.volume().box(), buf, sizeof buf);
  CHECK(g_pipe.put(buf, len));

  for (int s = 0; s < 60; ++s) {
    g_b.stepFixed();
    g_cb.pump(g_b, g_wb);
  }
  CHECK(g_cb.stats().packetsIn == 0u);
  CHECK(g_cb.stats().packetsOut == 0u);
  CHECK(g_pipe.count == 1);  // still waiting, not consumed and discarded
  CHECK(g_b.particleCount() == before);
}

TEST(chain_notices_spill_that_never_reached_a_packet) {
  // The one failure nothing downstream can see. A caller that pumps less often than the queue is
  // cleared loses particles the receiver then re-creates out of clones, so every count at the far
  // end still adds up and the volume is right -- the pour is simply half made of copies.
  //
  // This is not hypothetical: App::masterStep pumped once per frame while stepFixed() cleared the
  // queue twice, and 98 of 198 poured particles were what actually crossed the air.
  resetChain(0);
  SpillParticle p{};
  p.pos = Vec3{0.0f, 15.0f, 0.0f};
  p.vel = Vec3{0.0f, 6.0f, 0.0f};

  SpillQueue& q = g_a.spill();
  q.clear();
  for (int i = 0; i < 5; ++i) CHECK(q.push(p));
  g_ca.pump(g_a, g_wa);
  CHECK(g_ca.stats().unsent == 0u);

  // A frame whose spill the caller never pumped: the queue is cleared with five still in it, and
  // totalOut -- which is cumulative and never reset -- remembers them.
  q.clear();
  for (int i = 0; i < 5; ++i) CHECK(q.push(p));
  q.clear();
  for (int i = 0; i < 5; ++i) CHECK(q.push(p));
  g_ca.pump(g_a, g_wa);
  CHECK(g_ca.stats().unsent == 5u);
  CHECK(g_ca.stats().particlesOut == 10u);
}

// --- addressing: a broadcast is heard by everyone, and only one of them is downstream ---------

namespace {
Simulation g_c;
Bus g_bus;
BusWire g_bw0{&g_bus, 0}, g_bw1{&g_bus, 1}, g_bw2{&g_bus, 2};
SpillChain g_k0, g_k1, g_k2;

Simulation* const g_ring[3] = {&g_a, &g_b, &g_c};
SpillChain* const g_chains[3] = {&g_k0, &g_k1, &g_k2};
SpillTransport* const g_wires[3] = {&g_bw0, &g_bw1, &g_bw2};

int ringTotal() { return g_a.particleCount() + g_b.particleCount() + g_c.particleCount(); }

// `addressed` false is the state a cube ships in: it has never been told where it stands, so it
// takes every packet it hears.
void resetRing(bool addressed) {
  g_bus = Bus{};
  for (int i = 0; i < 3; ++i) {
    CHECK(g_ring[i]->init(Simulation::kCube, particlesForFill(400), 0xB0A7u + (uint32_t)i, 32));
    g_ring[i]->setOpenFace(kOpenPosY);
    g_ring[i]->setGravityObject(Vec3{0.0f, -kGravityMag, 0.0f});
    *g_chains[i] = SpillChain{};
    if (addressed) g_chains[i]->setChainPosition(i, 3);
  }
}

// Cube 0 is the only one tilted, so every particle on the air came from it.
void ringStep(int s) {
  g_a.setGravityObject(tiltAt(s));
  for (int i = 0; i < 3; ++i) {
    g_ring[i]->stepFixed();
    g_chains[i]->pump(*g_ring[i], *g_wires[i]);
  }
}
}  // namespace

TEST(chain_a_ring_of_three_takes_only_from_upstream) {
  resetRing(true);
  const int start = ringTotal();
  CHECK(g_k1.upstream() == 0);
  CHECK(g_k2.upstream() == 1);
  CHECK(g_k0.upstream() == 2);

  for (int s = 0; s < settleSteps(600); ++s) ringStep(s);

  const uint32_t out0 = g_k0.stats().particlesOut;
  std::printf("       ring of three: 0 poured %u; 1 took %u (foreign %u), 2 took %u (foreign %u),"
              " total %d -> %d\n",
              out0, g_k1.stats().particlesIn, g_k1.stats().foreign, g_k2.stats().particlesIn,
              g_k2.stats().foreign, start, ringTotal());

  CHECK(out0 > 0u);
  // Cube 1 is downstream of cube 0 and takes the pour.
  CHECK(g_k1.stats().particlesIn > 0u);
  // Cube 2 HEARD every one of those packets and took none of them. Without the address it would
  // have injected the same particles cube 1 did, and the ring would have gained volume out of
  // nothing -- a leak in the opposite direction from a dropped packet, and far harder to notice.
  CHECK(g_k2.stats().foreign > 0u);
  CHECK(g_k2.stats().packetsIn == 0u);
  CHECK(g_k2.stats().particlesIn == 0u);
  // Nothing was created. Particles still in flight or refused are accounted for either way.
  CHECK(ringTotal() <= start);
}

TEST(chain_an_unaddressed_ring_duplicates_what_it_hears) {
  // The behaviour the address exists to prevent, asserted rather than described. With no chain
  // position every cube takes every packet, so a single pour is injected TWICE and the ring ends
  // up holding more water than it started with.
  resetRing(false);
  const int start = ringTotal();
  for (int s = 0; s < settleSteps(600); ++s) ringStep(s);

  const uint32_t in1 = g_k1.stats().particlesIn, in2 = g_k2.stats().particlesIn;
  std::printf("       unaddressed ring: 0 poured %u; 1 took %u, 2 took %u; total %d -> %d\n",
              g_k0.stats().particlesOut, in1, in2, start, ringTotal());
  CHECK(in1 > 0u);
  CHECK(in2 > 0u);
  CHECK(in1 + in2 > g_k0.stats().particlesOut);  // the same particles, twice over
}
