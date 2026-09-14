#include "partsim/app/App.h"

#include <cstdlib>

#include "partsim/Scene.h"

namespace partsim {
namespace app {

// See Console.cpp: varargs promote float to double by definition, so every printf call site in
// this file trips -Wdouble-promotion and there is nothing to fix. Scoped to this file rather than
// disabled in the build flags, so the warning keeps catching real double promotions in the solver.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"

bool App::begin(Role role) {
  // The Platform contract, checked rather than trusted.
  //
  // Platform.h states that display, imu and link are never null -- a platform with no such device
  // passes the Null implementation -- and App then dereferences them unguarded in 18 places. Every
  // member nevertheless defaults to nullptr, so `Platform{&console}` compiles and leaves five of
  // them unset. Both platforms today populate all six, so this cannot fire; it exists for the
  // THIRD one, which is the entire reason the seam was built, and where a forgotten member would
  // otherwise surface as a null dereference somewhere far from the omission.
  if (!plat_.console) return false;  // nothing to report a diagnostic through
  {
    Console& c0 = *plat_.console;
    const struct {
      const void* p;
      const char* name;
    } required[] = {{plat_.clock, "clock"},
                    {plat_.display, "display"},
                    {plat_.imu, "imu"},
                    {plat_.link, "link"},
                    {plat_.chain, "chain"},
                    {plat_.hooks, "hooks"}};
    bool ok = true;
    for (const auto& r : required)
      if (!r.p) {
        c0.printf("FATAL: Platform.%s is null -- pass the Null implementation, not a null pointer\n",
                  r.name);
        ok = false;
      }
    if (!ok) return false;
  }

  Console& c = *plat_.console;
  role_ = role;

#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
  // A display node builds only the GEOMETRY and the container box. Every node holds the FULL panel
  // table even though this one draws two faces: Geometry::bounds() derives the container from it
  // and both ends must agree on that container exactly, or the master's quantised positions mean
  // something different here than they did there.
  geom_ = Geometry::cube(kPanelRes, pitchFor(kPanelRes));
  if (geom_.count() != 6 || !vol_.build(geom_, kSlabDepth, kCellSize)) {
    c.println("FATAL: geometry init failed");
    return false;
  }
  c.printf("display node: box %.1f units, geometry hash %04x\n", vol_.box().size().x,
           geometryHash(geom_));

  // Draw-only state. This used to sit inside setup()'s `#if !PARTSIM_QEMU` branch purely because
  // that is where it happened to be written; none of it touches hardware, and a display build has
  // never been combined with the QEMU environment anyway.
  {
    const RoleFaces rf = facesFor(role_);
    if (!rxRenderer_.init(geom_, rf.face, rf.count) || !rxHeat_.init(vol_)) {
      c.println("FATAL: display node state init failed");
      return false;
    }
    rxRenderer_.setExposure(kSplatExposure);
    rxParticles_.clear();
  }
#else
  const int mode = (kFaces == 1) ? Simulation::kSinglePanel : Simulation::kCube;
#if PARTSIM_MULTINODE
  // The master simulates but renders nothing.
  {
    static const int none[1] = {0};
    sim_.setRenderSet(none, 0);
  }
#endif
  if (!sim_.initScene(mode, 0, 1, kPanelRes)) {
    c.println("FATAL: simulation init failed -- capacities too small for this geometry");
    return false;
  }
#ifdef PARTSIM_TIER_BEAKER
  // The beaker tier IS the open vessel -- that is what the tier means -- so the face is opened
  // here rather than by a console command nobody would send. Every other tier leaves it closed
  // and every line of the chain below is then inert, which is what keeps beaker mode a mode.
  sim_.setOpenFace(kOpenPosY);
  // Refilled to a FRACTION of the room available, which is not tuning but a requirement of
  // chaining: the scene preset fills to the brim, and the first hardware run then had every one of
  // 229 arrivals rejected for want of room while the sending beaker emptied. A ring conserves
  // volume only if each beaker can hold its own fill plus whatever is in flight toward it.
  //
  // Sixty percent of WHICHEVER BINDS, and that distinction is the whole correctness of this line.
  // Two different ceilings exist: the particle pool (kMaxParticles, a memory figure) and the
  // vessel (capacity(), how many particles at rest density the box holds). On the device the pool
  // binds at 512 against a vessel of ~2100, so 60% of the pool is a beaker a quarter full and the
  // intent is met. At host capacities the pool is 16384 against the same vessel, 60% of it asks
  // for 9830, init clamps that to nine tenths of the VESSEL -- and the beaker comes up 90% full,
  // which is the exact condition this line exists to prevent. Found by M4-E in the browser.
  const int room = kMaxParticles < sim_.capacity() ? kMaxParticles : sim_.capacity();
  if (!sim_.init(mode, (room * 3) / 5, 0xBEA6u, kPanelRes)) {
    c.println("FATAL: beaker refill failed");
    return false;
  }
  // No scene drift in a beaker: the cycle would refill it mid-pour and the fill level is the
  // thing the user is looking at.
  sim_.setAutoCycle(false);
#else
  sim_.setAutoCycle(true);
#endif
  c.printf("simulation: %d particles, capacity %d\n", sim_.particleCount(), sim_.capacity());
#endif
  return true;
}

// --- IMU ----------------------------------------------------------------------------------------

void App::imuPoll() {
  MotionSensor& imu = *plat_.imu;
  ImuSample r;
  if (!imu.present() || !imu.read(r)) return;
  const uint32_t head = head_;
  const uint32_t nextHead = (head + 1) & (kRingSize - 1);
  if (nextHead != tail_) {
    ring_[head] = r;
    head_ = nextHead;
  } else {
    // Full means the consumer has stalled for a third of a second. Counted rather than silently
    // overwritten, because it would otherwise look like sensor noise.
    ++dropped_;
  }
}

int App::pumpMotion() {
  // Hoisted out of the loop: these are virtual calls, and the answer cannot change between two
  // samples of the same part.
  const float accelScale = plat_.imu->accelScaleG();
  const float gyroScale = plat_.imu->gyroScaleRad();
  int n = 0;
  uint32_t tail = tail_;
  while (tail != head_) {
    const ImuSample& r = ring_[tail];
    const Vec3 accelG{(float)r.ax * accelScale, (float)r.ay * accelScale,
                      (float)r.az * accelScale};
    const Vec3 gyro{(float)r.gx * gyroScale, (float)r.gy * gyroScale, (float)r.gz * gyroScale};
    if (!motion_.seeded()) motion_.seed(accelG);
    motion_.update(accelG, gyro, kImuDt);
    tail = (tail + 1) & (kRingSize - 1);
    ++n;
  }
  tail_ = tail;
  return n;
}

// --- per-frame bodies ---------------------------------------------------------------------------

#if !PARTSIM_MULTINODE
void App::simStep() {
  Clock& clk = *plat_.clock;
  Display& disp = *plat_.display;

  // Orientation and walk hold the display until switched off; see the Mode comment in App.h.
  // Walk needs nothing here at all -- lightOne() already wrote the strip from the console thread,
  // and simStep must not call present() over it.
  if (mode_ == Mode::Orientation) {
    disp.testPattern(sim_.geometry());
    return;
  }
  if (mode_ == Mode::Walk) return;

  const uint32_t t0 = clk.micros();
  pumpMotion();

  if (!paused_) {
    // Object-space gravity and container acceleration: exactly the two vectors the browser
    // supplies from the mouse. Below this line the device and the browser run identical code.
    if (motion_.seeded()) {
      sim_.setGravityObject(motion_.gravityObject());
      sim_.setContainerAccel(motion_.containerAccel());
    } else if (!plat_.imu->present() && !cannedFrozen_) {
      // No IMU exists at all (MINI.md M5) -- present() false is the ONLY branch this seam needs;
      // a board whose IMU answers takes the motion_.seeded() path above instead, unchanged. A
      // deterministic function of the step index, like the golden sequence and the beaker-spill
      // fixture's tiltAt (tests/test_beaker_spill.cpp): no clock, no RNG, so a run reproduces.
      const float t = (float)stats_.frames * 0.017f;
      sim_.setGravityObject(normalize(Vec3{fsin(t), fcos(t * 0.37f), fsin(t * 0.53f) * 0.5f}) *
                             kGravityMag);
    }
    stats_.substeps = sim_.advance(1.0f / (float)kTargetFps);
  }
  // HERE, and not on the console thread where the bring-up scaffolding used to poll the radio.
  // Simulation clears its spill queue at the top of every frame, so a pump running anywhere else
  // sends whatever survived the race rather than what the frame actually spilled.
  chain_.pump(sim_, *plat_.chain);

  const uint32_t t1 = clk.micros();
  sim_.accumulate();
  const uint32_t t2 = clk.micros();
  disp.present(sim_.renderer(), sim_.geometry());
  const uint32_t t3 = clk.micros();

  const float k = 0.1f;
  stats_.simMs += ((float)(t1 - t0) * 0.001f - stats_.simMs) * k;
  stats_.renderMs += ((float)(t2 - t1) * 0.001f - stats_.renderMs) * k;
  stats_.blitMs += ((float)(t3 - t2) * 0.001f - stats_.blitMs) * k;
  ++stats_.frames;
  ++framesSince_;

  const uint32_t now = clk.millis();
  if (!reportSeeded_) {
    lastReport_ = now;
    reportSeeded_ = true;
  } else if (now - lastReport_ >= 1000) {
    stats_.fps = (float)framesSince_ * 1000.0f / (float)(now - lastReport_);
    lastReport_ = now;
    framesSince_ = 0;
  }
}
#endif  // !PARTSIM_MULTINODE

#ifdef PARTSIM_PROFILE_ESP32_MASTER
void App::masterStep() {
  Clock& clk = *plat_.clock;
  // The master's step index is the authoritative clock, so it steps a FIXED number of times per
  // frame rather than calling advance(wallDt). advance() drops backlog when it saturates, which
  // would make the step count a function of frame timing -- and every display node would then be
  // comparing its own progress against a clock that skips.
  const int substeps = (int)((1.0f / (float)kTargetFps) / kFixedDt + 0.5f);

  const uint32_t t0 = clk.micros();
  pumpMotion();
  if (!paused_) {
    if (motion_.seeded()) {
      sim_.setGravityObject(motion_.gravityObject());
      sim_.setContainerAccel(motion_.containerAccel());
    }
    for (int i = 0; i < substeps; ++i) {
      sim_.stepFixed();
      ++masterStep_;
      // INSIDE the loop, because stepFixed() clears the spill queue per STEP -- unlike advance(),
      // which clears once per frame. Pumping after the loop instead sends only the last substep's
      // spill and silently discards the other half.
      //
      // It did exactly that, on hardware: of 198 particles poured, 98 were sent. The receiver
      // still ended up with the right VOLUME, because the header's cumulative count made the
      // missing half up out of clones -- which is why this cost nothing a particle count could
      // see, and why the first version of this comment confidently explained that one pump per
      // frame was the efficient choice.
      chain_.pump(sim_, *plat_.chain);
    }
    stats_.substeps = substeps;
  }
  const uint32_t t1 = clk.micros();

  FrameHeader h;
  h.step = masterStep_;
  h.geomHash = geometryHash(sim_.geometry());
  h.paletteA = (uint8_t)sim_.scene();
  const int len = encodeFrame(h, sim_.particles().view(), sim_.field().view(),
                              sim_.volume().box(), txFrame_, frameMaxBytes());
  if (len > 0) plat_.link->send(txFrame_, len);
  const uint32_t t2 = clk.micros();

  const float k = 0.1f;
  stats_.simMs += ((float)(t1 - t0) * 0.001f - stats_.simMs) * k;
  stats_.renderMs += ((float)(t2 - t1) * 0.001f - stats_.renderMs) * k;  // encode + send
  ++stats_.frames;
}
#endif  // master

#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
void App::displayStep() {
  Clock& clk = *plat_.clock;
  // Receive, decode, splat its own faces, blit. A poll that finds nothing, or a frame that fails
  // to decode, leaves the previous frame in place -- which is the specified behaviour, not a
  // fallback.
  const uint32_t t0 = clk.micros();
  int len = 0;
  const uint8_t* frame = plat_.link->pollDirect(len);
  if (frame && len > 0) {
    FrameHeader h;
    if (decodeFrame(frame, len, vol_.box(), geometryHash(geom_), rxParticles_, rxHeat_, h)) {
      rxLastStep_ = h.step;
      ++rxAccepted_;
    } else {
      ++rxRejected_;
    }
  } else {
    ++rxStarved_;
  }
  const uint32_t t1 = clk.micros();

  rxRenderer_.accumulate(rxParticles_.view(), rxHeat_.view(), geom_);
  const uint32_t t2 = clk.micros();
  plat_.display->present(rxRenderer_, geom_);
  const uint32_t t3 = clk.micros();

  const float k = 0.1f;
  stats_.simMs += ((float)(t1 - t0) * 0.001f - stats_.simMs) * k;   // receive + decode
  stats_.renderMs += ((float)(t2 - t1) * 0.001f - stats_.renderMs) * k;
  stats_.blitMs += ((float)(t3 - t2) * 0.001f - stats_.blitMs) * k;
  ++stats_.frames;
}
#endif  // display

// --- console ------------------------------------------------------------------------------------

void App::printHelp() {
  Console& c = *plat_.console;
  c.println("partsim console");
  c.println("  ?            this help");
  c.println("  s            list scenes");
  c.println("  s <n>        switch to scene n (gradual, crossfaded)");
  c.println("  c            toggle auto-cycle");
  c.println("  b <0-255>    panel brightness");
  c.println("  m            print the face mount table");
  c.println("  m <f> <r> <x>  set face f to rotation r (0-3), mirror x (0/1); keeps its slot");
  c.println("  m <f> <s> <r> <x>  also move face f to slot s; swaps with whatever was there");
  c.println("  t            toggle orientation mode (stays up until toggled off)");
  c.println("  w            walk: light the next strip index alone (bypasses the mount table)");
  c.println("  w <n>        walk: light strip index n alone; w -1 turns it off, resumes fluid");
  c.println("  i            IMU state");
  c.println("  r            frame timing and memory");
  c.println("  g            run the golden determinism sequence (blocks ~30s)");
  c.println("  p            pause/resume the physics");
  c.println("  f            freeze/unfreeze the canned tilt used when no IMU is present");
  c.println("  x            benchmark: particle sweep, needs no panels attached");
  c.println("  o <x> <y> <z>  tilt: set object-space gravity by hand (whole numbers, 0 0 -1 etc)");
  c.println("  n <id> <len>   this cube's place in the chain (n 1 3 = second of three)");
#if PARTSIM_ENABLE_CHROMA
  c.println("  d <r> <g>    this beaker's dye, 0-255 each (255 0 red, 0 255 green, 0 0 blue)");
#endif
}

void App::printMounts() {
  Console& c = *plat_.console;
  const ChainMap& cm = plat_.display->chain();
  c.printf("chain %dx%d, %d faces\n", cm.chainWidth(), cm.chainHeight(), cm.count());
  for (int i = 0; i < cm.count(); ++i) {
    const FaceMount& m = cm.mount(i);
    const ChainRun r = cm.row(i, 0);
    c.printf("  face %d -> slot %u rot %u mirror %u  (row %s)\n", i, m.slot, m.rotate, m.mirror,
             r.dy == 0 ? "horizontal" : "vertical, slower blit");
  }
  // The mount table this exercise arrives at is the deliverable, and it must not be retyped
  // after every reboot (MINI.md M3, "persisting the result"): paste this back as the profile's
  // defaultMounts. NVS persistence of it is out of scope.
  c.println("paste back as this profile's default mounts:");
  c.println("static const FaceMount kMounts[6] = {");
  for (int i = 0; i < cm.count(); ++i) {
    const FaceMount& m = cm.mount(i);
    c.printf("  {%u, %u, %u},\n", m.slot, m.rotate, m.mirror);
  }
  c.println("};");
}

void App::printImu() {
  Console& c = *plat_.console;
  if (!plat_.imu->present()) {
    c.printf("IMU absent (WHO_AM_I read 0x%02X, expected 0x6C)\n", plat_.imu->whoAmI());
    c.printf("  gravity follows a canned tilt sequence (`f` to freeze it)%s\n",
             cannedFrozen_ ? " -- currently frozen" : "");
    return;
  }
  const Vec3 d = motion_.down();
  const Vec3 b = motion_.gyroBias();
  const Vec3 a = motion_.containerAccel();
  c.printf("down   % .3f % .3f % .3f  (object space, unit)\n", d.x, d.y, d.z);
  c.printf("accel  % .2f % .2f % .2f  (container, sim units)\n", a.x, a.y, a.z);
  c.printf("bias   % .5f % .5f % .5f rad/s\n", b.x, b.y, b.z);
  c.printf("trust  %.2f   seeded %d   dropped samples %u\n", motion_.trust(),
           (int)motion_.seeded(), (unsigned)dropped_);
}

void App::printStats() {
  Console& c = *plat_.console;
  c.printf("fps %.1f   sim %.2f ms   splat %.2f ms   blit %.2f ms   substeps %d\n", stats_.fps,
           stats_.simMs, stats_.renderMs, stats_.blitMs, stats_.substeps);
  c.printf("frames %u, overruns %u (%.1f%%)\n", (unsigned)stats_.frames, (unsigned)overruns_,
           stats_.frames ? 100.0f * (float)overruns_ / (float)stats_.frames : 0.0f);
#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
  // A display node has no scene and no particles of its own: it draws whatever the last accepted
  // frame contained, which is the point of an authoritative master.
  c.printf("received particles %d/%d (drawn from the last accepted frame)\n",
           rxParticles_.count(), kMaxParticles);
#else
  c.printf("particles %d/%d   scene %d (%s)%s\n", sim_.particleCount(), kMaxParticles,
           sim_.scene(), sceneAt(sim_.scene()).name,
           sim_.transitioning() ? "  [transitioning]" : "");
#endif
#ifndef PARTSIM_PROFILE_ESP32_DISPLAY
  if (sim_.openFace() != kOpenNone) {
    const SpillChain::Stats& cs = chain_.stats();
    c.printf("beaker: open face %d, spilled %u (%u dropped by the queue)\n", sim_.openFace(),
             (unsigned)sim_.spill().totalOut, (unsigned)sim_.spill().dropped);
    if (chain_.chainLength() > 1)
      c.printf("chain position: cube %d of %d, taking from cube %d (%u packets ignored as not"
               " upstream)\n",
               chain_.chainId(), chain_.chainLength(), chain_.upstream(), (unsigned)cs.foreign);
    c.printf("chain %s: out %u in %u packets, in %u in %u packets, bad %u\n", plat_.chain->name(),
             (unsigned)cs.particlesOut, (unsigned)cs.packetsOut, (unsigned)cs.particlesIn,
             (unsigned)cs.packetsIn, (unsigned)cs.bad);
    if (cs.unsent)
      c.printf("WARNING: %u spilled particles never reached a packet -- the queue is being"
               " cleared between pumps\n", (unsigned)cs.unsent);
    c.printf("chain losses: shortfall %u, made up %u, owed %u, no room for %u\n",
             (unsigned)chain_.shortfall(), (unsigned)cs.madeUp, (unsigned)chain_.owed(),
             (unsigned)cs.rejected);
    const char* d = plat_.chain->diagnostic();
    if (d && d[0]) c.println(d);
  }
#endif
  c.printf("internal heap free %u B, largest block %u B\n", (unsigned)plat_.hooks->freeHeap(),
           (unsigned)plat_.hooks->largestHeapBlock());
  c.printf("psram free %u B\n", (unsigned)plat_.hooks->freePsram());
#if PARTSIM_MULTINODE
  c.printf("role %s, link %s\n", roleName(role_), plat_.link->name());
#ifdef PARTSIM_PROFILE_ESP32_MASTER
  c.printf("master step %u, frames sent %u, link errors %u\n", (unsigned)masterStep_,
           (unsigned)plat_.link->sent(), (unsigned)plat_.link->errors());
#else
  c.printf("last step %u, accepted %u, rejected %u, starved %u, link errors %u\n",
           (unsigned)rxLastStep_, (unsigned)rxAccepted_, (unsigned)rxRejected_,
           (unsigned)rxStarved_, (unsigned)plat_.link->errors());
#endif
#endif
}

#ifndef PARTSIM_PROFILE_ESP32_DISPLAY
// The third target of the cross-target determinism check. The host and WASM builds already agree
// bit-for-bit; this is the same scripted sequence at the same capacities, so the number below
// must equal the contents of scripts/golden_hash_esp32.txt.
void App::runGolden() {
  Console& c = *plat_.console;
  SuspendSim hold(plat_);  // Simulation::init below would otherwise race the render task
#ifdef PARTSIM_NONDETERMINISTIC_FP
  c.println("NOTE: built with -ffp-contract=fast, so this hash is EXPECTED to differ.");
  c.println("      Flash the `cube` environment to check determinism.");
#endif
  c.println("running golden sequence, this blocks the display for a while...");
  const bool wasPaused = paused_;
  paused_ = true;
  const uint32_t t0 = plat_.clock->millis();
  const uint32_t hash = goldenHash(sim_, kGoldenSteps, kGoldenSeed);
  const uint32_t dt = plat_.clock->millis() - t0;
#if PARTSIM_QEMU
  // Print the hash, and say plainly that the timing is meaningless here rather than leaving a
  // number that looks like a measurement. QEMU drives guest timers from HOST wall-clock, so this
  // reports how fast the emulator ran on whatever machine it ran on -- measured at 83 ms/step on
  // one laptop and 200 ms/step under -icount on the same one. Neither is the S3.
  (void)dt;
  c.printf("state  %08x   (%u steps; timing suppressed -- QEMU is not cycle-accurate)\n", hash,
           2u * kGoldenSteps);
#else
  c.printf("state  %08x   (%u steps in %u ms, %.2f ms/step)\n", hash, 2u * kGoldenSteps, dt,
           (float)dt / (float)(2 * kGoldenSteps));
#endif
  c.println("compare against scripts/golden_hash_esp32.txt (first field)");
  // The sequence left the simulation in whatever state it ended in; put a scene back.
  sim_.initScene(kFaces == 1 ? Simulation::kSinglePanel : Simulation::kCube, 0, 1, kPanelRes);
  sim_.setAutoCycle(true);
  paused_ = wasPaused;
}

// --- benchmark ----------------------------------------------------------------------------------
// The whole point of being able to run this on a bare devkit.
//
// Everything expensive here is computation into internal buffers: the solver, the splat, the
// palette resolve. Even the BLIT is measurable without a panel, because HUB75 is write-only -- the
// panel is a passive shift-register chain with no handshake, so drawPixelRGB888 writes into the
// DMA buffer and the LCD_CAM peripheral clocks it out into nothing. The only thing a missing panel
// costs is light.
//
// So this answers the one question that has never been answered: how many particles actually fit a
// frame. Everything else in the budget is downstream of it.
void App::runBench() {
  Console& c = *plat_.console;
  Clock& clk = *plat_.clock;
  SuspendSim hold(plat_);  // every sim_.init() below rebuilds the renderer the sim task is using
  // Sweep points, sized against the compiled pool rather than hardcoded. They were {320, 640,
  // 960, 1280} for a 1280-particle pool; at 512 all but the first are silently clamped and the
  // sweep reports a single row. Fractions of capacity keep the shape of the curve whatever the
  // profile, and the exponent fit needs at least three spread-out points.
  const int cap = sim_.capacity() < kMaxParticles ? sim_.capacity() : kMaxParticles;
  const int counts[] = {cap / 4, cap / 2, (cap * 3) / 4, cap};
  const int reps = 30;

  c.println("particle sweep, water only (no heat field):");
  c.println("  count    sim/step   splat   resolve    blit    frame    fps   verdict");

  int best = 0;
  for (unsigned ci = 0; ci < sizeof(counts) / sizeof(counts[0]); ++ci) {
    const int n = counts[ci];
    if (n > kMaxParticles) continue;
    if (!sim_.init(kFaces == 1 ? Simulation::kSinglePanel : Simulation::kCube, n, 1, kPanelRes)) {
      c.printf("  %5d    init failed\n", n);
      continue;
    }
    for (int i = 0; i < 20; ++i) sim_.stepFixed();  // settle, so the neighbour grid is realistic

    uint32_t t = clk.micros();
    for (int i = 0; i < reps; ++i) sim_.stepFixed();
    const float simMs = (float)(clk.micros() - t) / 1000.0f / (float)reps;

    t = clk.micros();
    for (int i = 0; i < reps; ++i) sim_.accumulate();
    const float splatMs = (float)(clk.micros() - t) / 1000.0f / (float)reps;

    t = clk.micros();
    for (int i = 0; i < reps; ++i)
      for (int k = 0; k < sim_.geometry().count(); ++k) sim_.renderer().resolve(k, staging_, 3);
    const float resolveMs = (float)(clk.micros() - t) / 1000.0f / (float)reps;

    float blitMs = 0.0f;
    if (plat_.display->ready()) {
      t = clk.micros();
      for (int i = 0; i < reps; ++i) plat_.display->present(sim_.renderer(), sim_.geometry());
      blitMs = (float)(clk.micros() - t) / 1000.0f / (float)reps;
    }

    // A displayed frame is kSubstepsPerFrame physics steps plus one of everything else.
    const float substeps = (1.0f / (float)kTargetFps) / kFixedDt;
    const float frameMs = simMs * substeps + splatMs + blitMs;
    const float budget = 1000.0f / (float)kTargetFps;
    if (frameMs <= budget) best = n;
    c.printf("  %5d    %7.2f %7.2f   %7.2f %7.2f  %7.2f %6.1f   %s\n", n, simMs, splatMs,
             resolveMs, blitMs, frameMs, 1000.0f / frameMs, frameMs <= budget ? "fits" : "OVER");
  }

  c.printf("\n  %.0f substeps/frame at %d fps (kFixedDt = 1/%.0f), budget %.1f ms\n",
           (1.0f / (float)kTargetFps) / kFixedDt, kTargetFps, 1.0f / kFixedDt,
           1000.0f / (float)kTargetFps);
  c.printf("  largest sweep point that fits: %d particles\n", best);

  // The kettle is the measured worst case: water, sand AND an active heat field, so splatField
  // iterates every cell instead of exiting immediately.
  if (sim_.initScene(kFaces == 1 ? Simulation::kSinglePanel : Simulation::kCube, 4, 1,
                     kPanelRes)) {
    for (int i = 0; i < 60; ++i) sim_.stepFixed();
    uint32_t t = clk.micros();
    for (int i = 0; i < reps; ++i) sim_.stepFixed();
    const float simMs = (float)(clk.micros() - t) / 1000.0f / (float)reps;
    t = clk.micros();
    for (int i = 0; i < reps; ++i) sim_.accumulate();
    const float splatMs = (float)(clk.micros() - t) / 1000.0f / (float)reps;
    c.printf("\n  kettle (%d particles + active heat field): sim %.2f ms, splat %.2f ms\n",
             sim_.particleCount(), simMs, splatMs);
    c.println("  the heat field is why splat costs more here -- splatField exits");
    c.println("  immediately when nothing is burning.");
  }

  c.println("\nrestoring scene 0");
  sim_.initScene(kFaces == 1 ? Simulation::kSinglePanel : Simulation::kCube, 0, 1, kPanelRes);
  sim_.setAutoCycle(true);
}
#endif  // !display

void App::handleLine(char* line) {
  Console& c = *plat_.console;
  // Tokenise in place; no String, no allocation.
  // 5, not 4: `m <face> <slot> <rot> <mirror>` (MINI.md M3's slot-swap form) is the first command
  // that needs a 4th argument after the letter.
  char* argv[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};
  int argc = 0;
  for (char* p = line; *p && argc < 5;) {
    while (*p == ' ' || *p == '\t') ++p;
    if (!*p) break;
    argv[argc++] = p;
    while (*p && *p != ' ' && *p != '\t') ++p;
    if (*p) *p++ = '\0';
  }
  if (argc == 0) return;

  switch (argv[0][0]) {
    case '?':
    case 'h':
      printHelp();
      break;

    case 's':
#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
      // Scene is the master's decision and arrives in the frame header. A display node choosing
      // its own would be the one way this architecture could produce disagreeing faces.
      c.println("scene is set on the master; a display node only draws what it receives");
#else
      if (argc >= 2) {
        const int n = atoi(argv[1]);
        if (n >= 0 && n < sceneCount()) {
          sim_.transitionToScene(n);
          c.printf("scene -> %d (%s)\n", n, sceneAt(n).name);
        } else {
          c.printf("scene out of range 0..%d\n", sceneCount() - 1);
        }
      } else {
        for (int i = 0; i < sceneCount(); ++i)
          c.printf("  %d %s%s\n", i, sceneAt(i).name, i == sim_.scene() ? "  <--" : "");
      }
#endif
      break;

    case 'c':
#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
      c.println("auto-cycle is set on the master");
#else
      sim_.setAutoCycle(!sim_.autoCycle());
      c.printf("auto-cycle %s\n", sim_.autoCycle() ? "on" : "off");
#endif
      break;

#if PARTSIM_ENABLE_CHROMA && !defined(PARTSIM_PROFILE_ESP32_DISPLAY)
    case 'd':
      // The colour this cube IS, which is what makes a chain read as mixing: a red beaker pouring
      // into a blue one. Set here rather than only in the browser because a cube on a shelf has no
      // browser attached, and it does NOT survive a reboot yet -- see M4-E, which is where the
      // configuration surface for a chain belongs.
      if (argc >= 3) {
        const int r = atoi(argv[1]), g = atoi(argv[2]);
        int rc = r < 0 ? 0 : (r > 255 ? 255 : r);
        int gc = g < 0 ? 0 : (g > 255 ? 255 : g);
        // R and G are shares of a 255 budget, not independent channels -- blue is whatever is
        // left (Renderer::splatChroma). Clamping each to 0..255 alone still lets r+g exceed 255,
        // which drives the implied blue negative; found via the mini cube's web UI feeding an
        // ordinary colour picker's R/G straight through. Scale both down together so the
        // requested RATIO survives rather than favouring whichever argument came first.
        const int sum = rc + gc;
        if (sum > 255) {
          rc = (rc * 255) / sum;
          gc = (gc * 255) / sum;
        }
        // 8.8 fixed point: a byte per channel loses 98% of the dye to truncation over a few
        // hundred diffusion steps (M4-A measured it), so what the user types is the high byte.
        sim_.setDye((uint16_t)(rc * 256), (uint16_t)(gc * 256));
      }
      c.printf("dye %u %u (of 255; blue is what is left)\n", (unsigned)(sim_.dyeR() / 256),
               (unsigned)(sim_.dyeG() / 256));
      break;
#endif

#ifndef PARTSIM_PROFILE_ESP32_DISPLAY
    case 'n':
      // Which cube this one stands under. The radio broadcasts, so without this every cube in
      // earshot injects every other cube's pour and a ring of three GAINS volume out of nothing --
      // measured at 150 particles becoming 200 (chain_an_unaddressed_ring_duplicates_what_it_hears).
      //
      // Not persisted across a reboot yet; see M4-E, which is where the configuration surface for
      // a chain belongs.
      if (argc >= 3) chain_.setChainPosition(atoi(argv[1]), atoi(argv[2]));
      if (chain_.chainLength() > 1)
        c.printf("chain: cube %d of %d, taking from cube %d\n", chain_.chainId(),
                 chain_.chainLength(), chain_.upstream());
      else
        c.println("chain: unaddressed -- this cube takes packets from anyone");
      break;
#endif

    case 'o':
#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
      c.println("orientation is the master's; a display node only draws what it receives");
#else
      // Tilting by hand, because a board with no IMU can never pour and beaker mode is entirely
      // about pouring. Integers only: the console parses no floats anywhere and one decimal point
      // is not worth an atof in a firmware that has otherwise avoided it. The vector is
      // normalised, so `o 0 1 0` is the cube upside down and `o 1 -1 0` is a 45-degree lean.
      if (argc >= 4) {
        const Vec3 g{(float)atoi(argv[1]), (float)atoi(argv[2]), (float)atoi(argv[3])};
        if (length2(g) < 0.001f) {
          c.println("o: that vector has no direction");
          break;
        }
        sim_.setGravityObject(normalize(g) * kGravityMag);
        if (motion_.seeded())
          c.println("note: the IMU is live and will overwrite this on the next frame");
      }
      {
        const Vec3 g = sim_.gravityObject();
        c.printf("gravity (object) %.2f %.2f %.2f\n", (double)g.x, (double)g.y, (double)g.z);
      }
#endif
      break;

    case 'b':
      if (argc >= 2) {
        const int v = atoi(argv[1]);
        plat_.display->setBrightness((uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v)));
        c.printf("brightness %d\n", v);
      }
      break;

    case 'm': {
      bool changed = false;
      if (argc == 4) {
        // Unchanged: face keeps its chain slot, only rotation/mirror change.
        ChainMap& cm = plat_.display->chain();
        const int f = atoi(argv[1]);
        FaceMount m{(uint8_t)f, (uint8_t)atoi(argv[2]), (uint8_t)atoi(argv[3])};
        if (f >= 0 && f < cm.count()) m.slot = cm.mount(f).slot;
        if (f < 0 || f >= cm.count() || !cm.setMount(f, m)) {
          c.println("rejected: that would not be a valid mount table");
        } else {
          changed = true;
        }
      } else if (argc >= 5) {
        // The slot-moving form (MINI.md M3): which physical matrix is which face is exactly
        // what a wrong chain slot gets, unlike the HUB75 cube where slot was already right and
        // only the gluing was in question.
        ChainMap& cm = plat_.display->chain();
        const int f = atoi(argv[1]);
        const int slot = atoi(argv[2]);
        if (f < 0 || f >= cm.count() || slot < 0 || slot >= cm.count()) {
          c.println("rejected: face or slot out of range");
        } else {
          const FaceMount m{(uint8_t)slot, (uint8_t)atoi(argv[3]), (uint8_t)atoi(argv[4])};
          int occupant = -1;
          for (int i = 0; i < cm.count(); ++i)
            if (i != f && cm.mount(i).slot == (uint8_t)slot) { occupant = i; break; }
          bool ok;
          if (occupant < 0) {
            ok = cm.setMount(f, m);
          } else {
            // The slot f is asking for is taken: swap, rather than reject, because "these two are
            // the wrong way round" is the single most likely thing a `t` walk just found.
            FaceMount occ = cm.mount(occupant);
            occ.slot = cm.mount(f).slot;
            ok = cm.setMounts(f, m, occupant, occ);
            if (ok) c.printf("swapped: face %d <-> face %d\n", f, occupant);
          }
          if (!ok) c.println("rejected: that would not be a valid mount table");
          changed = ok;
        }
      }
      if (changed && plat_.hooks) plat_.hooks->mountsChanged(plat_.display->chain());
      printMounts();
      break;
    }

    case 't':
      mode_ = (mode_ == Mode::Orientation) ? Mode::Fluid : Mode::Orientation;
      if (mode_ == Mode::Orientation) {
        c.println("orientation mode: on -- stays up until `t` again");
        c.println("eight corner colours; every corner must read the same on the three faces");
        c.println("meeting it. correct with `m`, including the slot-swap form; see `?`.");
      } else {
        c.println("orientation mode: off");
      }
      break;

    case 'w': {
      const Display& disp = *plat_.display;
      if (disp.lightCount() <= 0) {
        c.println("walk: this display has no addressable strip");
        break;
      }
      if (argc >= 2) {
        walkIndex_ = atoi(argv[1]);
      } else if (mode_ == Mode::Walk) {
        ++walkIndex_;
      }
      if (walkIndex_ < 0) {
        mode_ = Mode::Fluid;
        plat_.display->lightOne(-1);
        c.println("walk: off, resuming fluid");
      } else {
        mode_ = Mode::Walk;
        if (plat_.display->lightOne(walkIndex_))
          c.printf("walk: index %d of %d\n", walkIndex_, disp.lightCount());
        else
          c.printf("walk: %d is out of range (0..%d)\n", walkIndex_, disp.lightCount() - 1);
      }
      break;
    }

    case 'f':
#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
      c.println("no motion on a display node: gravity arrives in the frame from the master");
#else
      cannedFrozen_ = !cannedFrozen_;
      c.printf("canned tilt %s\n", cannedFrozen_ ? "frozen" : "running");
      if (plat_.imu->present()) c.println("note: an IMU is present, so this has no effect");
#endif
      break;

    case 'i':
      printImu();
      break;

    case 'r':
      printStats();
      break;

    case 'g':
#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
      c.println("no golden sequence on a display node: it runs no solver");
#else
      runGolden();
#endif
      break;

    case 'x':
#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
      c.println("no benchmark on a display node: it runs no solver");
#else
      runBench();
#endif
      break;

    case 'p':
      paused_ = !paused_;
      c.printf("physics %s\n", paused_ ? "paused" : "running");
      break;

    default:
      c.println("unknown command; ? for help");
      break;
  }
}

void App::consolePoll() {
  char buf[64];
  const int n = plat_.console->readLine(buf, sizeof(buf));
  if (n > 0) handleLine(buf);
}

void App::submitCommand(const char* line) {
  char buf[64];
  size_t n = 0;
  while (line[n] && n + 1 < sizeof(buf)) { buf[n] = line[n]; ++n; }
  buf[n] = '\0';
  handleLine(buf);
}

#pragma GCC diagnostic pop

}  // namespace app
}  // namespace partsim
