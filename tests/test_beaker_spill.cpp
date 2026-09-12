// M4-B: the open face, and what leaves through it.
//
// Two assertions carry this item and they are opposites. A RING of beakers must conserve the
// total particle count however hard it is shaken -- that is what catches a leak in the open-face
// path, and it is why SpillQueue counts `dropped` instead of discarding silently. A SINGLE beaker
// must lose particles monotonically and never get them back, which is the behaviour the mode was
// asked for. A bug that fails only one of the two is a different bug from one that fails both.
#include "check.h"
#include "partsim/Simulation.h"

using namespace partsim;

namespace {

// ~1.2MB each; static storage only. Three, because a ring of two would wrap a pour straight back
// into the cube it came from and a swap is not a chain.
Simulation g_a, g_b, g_c;
Simulation* const g_ring[3] = {&g_a, &g_b, &g_c};

int ringTotal() { return g_a.particleCount() + g_b.particleCount() + g_c.particleCount(); }

// Gravity that sweeps well past horizontal, so the cube spends much of the run actually pouring
// rather than merely leaning. A pure function of the step index, like the golden sequence: no
// clock and no RNG, so a failure reproduces.
Vec3 tiltAt(int s) {
  const float t = (float)s * 0.031f;
  // y sweeps through +1, which is the cube upside down.
  return normalize(Vec3{fsin(t), fcos(t * 0.41f), fsin(t * 0.67f) * 0.5f}) * kGravityMag;
}

// Move everything cube i spilled into cube i+1, and count anything that did not make it.
// Deliberately NOT hidden behind a helper in core/: how a chain is wired is item D's decision on
// the device and item E's in the browser, and the only thing B owes them is that this loop is
// lossless.
int drainInto(Simulation& from, Simulation& to) {
  int rejected = 0;
  const SpillQueue& q = from.spill();
  for (int i = 0; i < q.count; ++i)
    if (!to.injectSpill(q.items[i])) ++rejected;
  from.spill().clear();
  return rejected;
}

}  // namespace

// --- B1: the face itself -----------------------------------------------------------------

TEST(spill_the_box_is_closed_until_someone_opens_it) {
  SimVolume v;
  CHECK(v.build(Geometry::cube(32, 1.0f), kSlabDepth, kCellSize));
  CHECK(v.openFace() == kOpenNone);
  CHECK(!v.isOpen());
  CHECK(v.openAxis() == -1);

  // Far outside on every axis: a closed box pulls all three back.
  const Vec3 in = v.clampInto(Vec3{-999.0f, 999.0f, 999.0f});
  const Aabb& b = v.box();
  CHECK(in.x == b.lo.x);
  CHECK(in.y < b.hi.y && in.y > b.hi.y - 0.01f);
  CHECK(in.z < b.hi.z && in.z > b.hi.z - 0.01f);
  CHECK(!v.pastOpenFace(Vec3{0.0f, 999.0f, 0.0f}));
}

TEST(spill_each_of_the_six_faces_can_be_the_open_one) {
  SimVolume v;
  CHECK(v.build(Geometry::cube(32, 1.0f), kSlabDepth, kCellSize));
  const Aabb& b = v.box();

  for (int f = kOpenNegX; f <= kOpenPosZ; ++f) {
    v.setOpenFace(f);
    CHECK(v.openFace() == f);
    CHECK(v.openAxis() == f / 2);
    CHECK(v.openIsHigh() == ((f & 1) != 0));

    // A point far outside on EVERY axis keeps only its open component; the other five walls
    // still hold. This is the assertion that a one-face hole did not become a one-axis hole.
    const Vec3 far{-999.0f, -999.0f, -999.0f};
    const Vec3 lo = v.clampInto(far);
    const Vec3 hi = v.clampInto(Vec3{999.0f, 999.0f, 999.0f});
    const float loC[3] = {lo.x, lo.y, lo.z};
    const float hiC[3] = {hi.x, hi.y, hi.z};
    const float bLo[3] = {b.lo.x, b.lo.y, b.lo.z};
    for (int a = 0; a < 3; ++a) {
      const bool openLow = (v.openAxis() == a && !v.openIsHigh());
      const bool openHigh = (v.openAxis() == a && v.openIsHigh());
      CHECK(openLow ? (loC[a] == -999.0f) : (loC[a] == bLo[a]));
      CHECK(openHigh ? (hiC[a] == 999.0f) : (hiC[a] < 999.0f));
    }

    // ...and only a crossing of THAT face counts as a spill.
    const Vec3 out = v.clampInto(v.openIsHigh() ? Vec3{999.0f, 999.0f, 999.0f} : far);
    CHECK(v.pastOpenFace(out));
    CHECK(!v.pastOpenFace(Vec3{0.0f, 0.0f, 0.0f}));
  }

  v.setOpenFace(kOpenNone);
  CHECK(!v.isOpen());
  v.setOpenFace(97);  // out of range closes it rather than indexing off the end
  CHECK(v.openFace() == kOpenNone);
}

#if PARTSIM_ENABLE_SAND
TEST(spill_the_friction_pass_does_not_re_imprison_an_escaping_particle) {
  // The clamp call site nothing else in the suite can cover. Friction is skipped entirely for
  // mu == 0, so a beaker of WATER never runs the pass at all -- and beaker mode is water only, so
  // if this clamp were left closed nothing in the shipping configuration would ever notice.
  //
  // Measured, not argued. Same fixture, same seed, the only difference being the friction pass
  // clamping against the closed Aabb instead of SimVolume::clampInto. 1102 grains, gravity
  // tilted `deg` from the cube's own down axis, 900 steps:
  //
  //   deg                     95   112   129   146   163   180
  //   grains left, open        5     4     0     0     0     0
  //   steps to empty, open     -     -   218   145   120    79
  //   grains left, SHUT      978   971   947   955   940   909
  //
  // Not a delay -- a wall. A grain whose predicted position is pulled back inside gets a fresh
  // contact next step and is pulled back again, so an inverted beaker of sand keeps 82% of its
  // contents indefinitely.
  //
  // The pile has to be FULL. A sparse one (47 grains) still discriminates, but only once the
  // population machinery stops refilling behind it: before advanceTransition learned to leave an
  // open vessel alone, both builds emptied this fixture identically and the test proved nothing.
  static Simulation sim;
  // Over capacity on purpose: init caps at 9/10 of it, and a full beaker is what puts grains in
  // contact AT THE RIM, which is where this clamp is observable.
  CHECK(sim.init(Simulation::kCube, kMaxParticles, 0x5A4Du, 32));
  Particles& p = const_cast<Particles&>(sim.particles());
  for (int i = 0; i < p.n; ++i) p.mat[i] = kSand;
  const int before = sim.particleCount();
  CHECK(before > 900);

  sim.setOpenFace(kOpenPosY);
  sim.setGravityObject(Vec3{0.0f, kGravityMag, 0.0f});  // upside down: the widest margin
  const int budget = settleSteps(200);                  // 79 steps open against 909 grains shut
  int emptyAt = -1;
  for (int s = 0; s < budget && emptyAt < 0; ++s) {
    sim.stepFixed();
    if (sim.particleCount() == 0) emptyAt = s;
  }
  std::printf("       %d grains through an open top: empty at step %d of %d, spilled %u\n", before,
              emptyAt, budget, sim.spill().totalOut);
  CHECK(emptyAt >= 0);
  CHECK(sim.spill().totalOut == (uint32_t)before);
  CHECK(sim.spill().dropped == 0u);
}
#endif

// --- B2: one cube, and it does not come back ------------------------------------------------

TEST(spill_a_closed_cube_never_loses_a_particle) {
  // The control. The same shaking that empties an open beaker must leave a closed one untouched,
  // or "the count strictly decreases" is measuring the solver rather than the open face.
  CHECK(g_a.init(Simulation::kCube, particlesForFill(600), 0xB0A7u, 32));
  const int before = g_a.particleCount();
  CHECK(g_a.openFace() == kOpenNone);
  for (int s = 0; s < settleSteps(1200); ++s) {
    g_a.setGravityObject(tiltAt(s));
    if (s % 53 == 52) g_a.addContainerAccel(Vec3{40.0f, 30.0f, -25.0f});
    g_a.stepFixed();
  }
  CHECK(g_a.particleCount() == before);
  CHECK(g_a.spill().totalOut == 0u);
  CHECK(g_a.spill().dropped == 0u);
}

TEST(spill_a_single_cube_count_strictly_decreases_and_never_recovers) {
  CHECK(g_a.init(Simulation::kCube, particlesForFill(600), 0xB0A7u, 32));
  g_a.setOpenFace(kOpenPosY);
  const int before = g_a.particleCount();

  int last = before, everRose = 0, emptyAt = -1;
  const int steps = settleSteps(1200);
  for (int s = 0; s < steps; ++s) {
    g_a.setGravityObject(tiltAt(s));
    if (s % 53 == 52) g_a.addContainerAccel(Vec3{40.0f, 30.0f, -25.0f});
    g_a.stepFixed();
    // Nothing drains the queue, so this is the "spills and does not come back" path exactly as a
    // single-cube caller runs it -- stepFixed clears the queue itself.
    const int now = g_a.particleCount();
    if (now > last) ++everRose;
    if (now == 0 && emptyAt < 0) emptyAt = s;
    last = now;
  }
  std::printf("       one open beaker: %d -> %d over %d steps, empty at %d, spilled %u\n", before,
              last, steps, emptyAt, g_a.spill().totalOut);
  CHECK(everRose == 0);
  CHECK(last < before);
  CHECK((uint32_t)(before - last) == g_a.spill().totalOut);
  // The population machinery must not quietly refill it. Before harvestSpill lowered
  // targetWater_, advanceTransition topped the beaker up at 32 particles a step and the count
  // never fell at all.
  CHECK(!g_a.transitioning());
}

TEST(spill_the_queue_holds_one_step_and_counts_what_it_could_not_hold) {
  CHECK(g_a.init(Simulation::kCube, particlesForFill(600), 0xB0A7u, 32));
  // init() resets the counters but NOT the open face, so a reset refills a beaker rather than
  // sealing it. Both halves matter and neither is obvious.
  CHECK(g_a.openFace() == kOpenPosY);
  CHECK(g_a.spill().totalOut == 0u);
  CHECK(g_a.spill().dropped == 0u);
  const int before = g_a.particleCount();
  g_a.setGravityObject(Vec3{0.0f, kGravityMag, 0.0f});  // straight up: a burst, not a trickle

  uint32_t seenInOneStep = 0;
  for (int s = 0; s < 200; ++s) {
    g_a.stepFixed();
    if ((uint32_t)g_a.spill().count > seenInOneStep) seenInOneStep = (uint32_t)g_a.spill().count;
    CHECK(g_a.spill().count <= kMaxSpill);
  }
  // totalOut is cumulative and survives the per-step clear; count is only the last step's worth.
  std::printf("       burst: totalOut %u, dropped %u, most in one step %u (cap %d)\n",
              g_a.spill().totalOut, g_a.spill().dropped, seenInOneStep, kMaxSpill);
  CHECK(g_a.spill().totalOut > (uint32_t)g_a.spill().count);
  // Whatever overflowed is COUNTED -- the difference between a full buffer and a physics leak.
  CHECK(g_a.spill().totalOut == (uint32_t)(before - g_a.particleCount()));
}

// --- B3: arrivals ---------------------------------------------------------------------------

TEST(spill_an_arrival_enters_at_the_top_where_it_left) {
  CHECK(g_a.init(Simulation::kCube, 0, 1u, 32));
  g_a.setOpenFace(kOpenPosY);
  const Aabb& b = g_a.volume().box();

  SpillParticle s{};
  s.pos = Vec3{-11.0f, 40.0f, 7.0f};   // well above the rim, as a real crossing would be
  s.vel = Vec3{2.0f, 9.0f, -1.0f};     // and moving OUT
  CHECK(g_a.injectSpill(s));
  CHECK(g_a.particleCount() == 1);

  const Vec3 p = g_a.particles().pos(0), v = g_a.particles().vel(0);
  // Horizontal position preserved exactly: object space on both sides is the whole reason
  // SpillParticle is not normalised, and it is what makes a corner pour land in a corner.
  CHECK(p.x == -11.0f);
  CHECK(p.z == 7.0f);
  CHECK_NEAR(p.y, b.hi.y - kRestSpacing, 1e-4);
  // Downward, at the speed it left with. The tangential components are untouched, so a stream
  // that was moving sideways keeps moving sideways.
  CHECK(v.y == -9.0f);
  CHECK(v.x == 2.0f && v.z == -1.0f);
  // ...and it is inside, so the next harvest does not take it straight back out again.
  CHECK(!g_a.volume().pastOpenFace(p));

  // A sender whose horizontal position is off the edge is clamped rather than placed in a wall.
  SpillParticle e{};
  e.pos = Vec3{999.0f, 40.0f, -999.0f};
  e.vel = Vec3{0.0f, 1.0f, 0.0f};
  CHECK(g_a.injectSpill(e));
  const Vec3 q = g_a.particles().pos(1);
  CHECK(q.x < b.hi.x && q.x > b.hi.x - 2.0f * kRestSpacing);
  CHECK(q.z > b.lo.z && q.z < b.lo.z + 2.0f * kRestSpacing);

  // A closed cube has nowhere to put an arrival and says so rather than inventing a face.
  CHECK(g_b.init(Simulation::kCube, 0, 1u, 32));
  CHECK(!g_b.injectSpill(s));
  CHECK(g_b.particleCount() == 0);
}

TEST(spill_an_arrival_falls_in_rather_than_bouncing_out) {
  // "Enters near the top with downward velocity" is only true if it is still there a moment
  // later. An arrival placed on the rim with outward velocity would be harvested by the very
  // next step and the ring would become a bucket brigade with no water in it.
  CHECK(g_a.init(Simulation::kCube, 0, 1u, 32));
  g_a.setOpenFace(kOpenPosY);
  for (int k = 0; k < 32; ++k) {
    SpillParticle s{};
    s.pos = Vec3{(float)(k % 8) * 3.0f - 12.0f, 30.0f, (float)(k / 8) * 3.0f - 6.0f};
    s.vel = Vec3{0.0f, 6.0f, 0.0f};
    CHECK(g_a.injectSpill(s));
  }
  g_a.setGravityObject(Vec3{0.0f, -kGravityMag, 0.0f});  // upright
  g_a.stepFixed();
  CHECK(g_a.spill().count == 0);
  CHECK(g_a.particleCount() == 32);
  for (int s = 0; s < settleSteps(300); ++s) g_a.stepFixed();
  CHECK(g_a.particleCount() == 32);  // upright, so nothing leaves
}

// --- the assertion the whole item rests on ---------------------------------------------------

TEST(spill_a_ring_of_three_conserves_every_particle) {
  const int each = particlesForFill(400);
  for (int k = 0; k < 3; ++k) {
    CHECK(g_ring[k]->init(Simulation::kCube, each, 0x1000u + (uint32_t)k, 32));
    g_ring[k]->setOpenFace(kOpenPosY);
  }
  const int before = ringTotal();
  CHECK(before == 3 * g_a.particleCount());

  int rejected = 0, minTotal = before, maxTotal = before;
  uint32_t dropped = 0;
  const int steps = settleSteps(2000);
  for (int s = 0; s < steps; ++s) {
    // Each cube tilts on its own phase, so they are not all pouring into a full neighbour at
    // the same instant -- which is the case that would overflow a queue or a pool.
    for (int k = 0; k < 3; ++k) {
      g_ring[k]->setGravityObject(tiltAt(s + k * 37));
      if ((s + k * 11) % 53 == 52) g_ring[k]->addContainerAccel(Vec3{35.0f, 25.0f, -20.0f});
      g_ring[k]->stepFixed();
    }
    // Read every queue BEFORE moving anything, so a particle cannot cross two cubes in one step
    // and arrive at a third -- which conserves the count and would hide a real ordering bug.
    for (int k = 0; k < 3; ++k) dropped += g_ring[k]->spill().dropped;
    for (int k = 0; k < 3; ++k) rejected += drainInto(*g_ring[k], *g_ring[(k + 1) % 3]);
    const int t = ringTotal();
    if (t < minTotal) minTotal = t;
    if (t > maxTotal) maxTotal = t;
  }

  std::printf("       ring of 3: %d particles, %d steps, total stayed in [%d, %d], "
              "spilled %u/%u/%u, dropped %u, rejected %d\n",
              before, steps, minTotal, maxTotal, g_a.spill().totalOut, g_b.spill().totalOut,
              g_c.spill().totalOut, dropped, rejected);
  // The point of the fixture: it must actually have poured, or conservation is trivially true.
  CHECK(g_a.spill().totalOut > 0u && g_b.spill().totalOut > 0u && g_c.spill().totalOut > 0u);
  CHECK(dropped == 0u);
  CHECK(rejected == 0);
  CHECK(minTotal == before);
  CHECK(maxTotal == before);
  CHECK(ringTotal() == before);
}

#if PARTSIM_ENABLE_CHROMA
TEST(spill_carries_its_dye_into_the_next_cube) {
  // The reason SpillParticle holds cr/cg at all: red poured into blue must arrive still red.
  CHECK(g_a.init(Simulation::kCube, particlesForFill(400), 0x2222u, 32));
  CHECK(g_b.init(Simulation::kCube, particlesForFill(400), 0x3333u, 32));
  g_a.setOpenFace(kOpenPosY);
  g_b.setOpenFace(kOpenPosY);

  Particles& pa = const_cast<Particles&>(g_a.particles());
  for (int i = 0; i < pa.n; ++i) { pa.cr[i] = (uint16_t)kChromaOne; pa.cg[i] = 0; }  // red
  Particles& pb = const_cast<Particles&>(g_b.particles());
  for (int i = 0; i < pb.n; ++i) { pb.cr[i] = 0; pb.cg[i] = 0; }                     // blue

  double redBefore = 0.0;
  for (int i = 0; i < pb.n; ++i) redBefore += pb.cr[i];
  CHECK(redBefore == 0.0);

  int moved = 0;
  for (int s = 0; s < settleSteps(800); ++s) {
    g_a.setGravityObject(tiltAt(s));
    g_a.stepFixed();
    g_b.setGravityObject(Vec3{0.0f, -kGravityMag, 0.0f});  // upright: it receives, it does not pour
    g_b.stepFixed();
    moved += g_a.spill().count;
    CHECK(drainInto(g_a, g_b) == 0);
    g_b.spill().clear();
  }

  double redAfter = 0.0;
  for (int i = 0; i < pb.n; ++i) redAfter += pb.cr[i];
  std::printf("       %d particles poured red into blue; receiver's red dye %.0f -> %.0f\n",
              moved, redBefore, redAfter);
  CHECK(moved > 0);
  CHECK(redAfter > 0.0);
  // Arrived as dye on particles, then mixed: no particle should still be pure red if mixing ran,
  // and the pool as a whole must be redder than it started.
  CHECK(redAfter < (double)kChromaOne * pb.n);
}
#endif
