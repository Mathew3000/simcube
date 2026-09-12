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
  sim_.setAutoCycle(true);
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

  if (showTestPattern_) {
    disp.testPattern(sim_.geometry());
    showTestPattern_ = false;
    return;
  }

  const uint32_t t0 = clk.micros();
  pumpMotion();

  if (!paused_) {
    // Object-space gravity and container acceleration: exactly the two vectors the browser
    // supplies from the mouse. Below this line the device and the browser run identical code.
    if (motion_.seeded()) {
      sim_.setGravityObject(motion_.gravityObject());
      sim_.setContainerAccel(motion_.containerAccel());
    }
    stats_.substeps = sim_.advance(1.0f / (float)kTargetFps);
  }

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
  c.println("  m <f> <r> <x>  set face f to rotation r (0-3), mirror x (0/1)");
  c.println("  t            show the orientation test pattern");
  c.println("  i            IMU state");
  c.println("  r            frame timing and memory");
  c.println("  g            run the golden determinism sequence (blocks ~30s)");
  c.println("  p            pause/resume the physics");
  c.println("  x            benchmark: particle sweep, needs no panels attached");
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
}

void App::printImu() {
  Console& c = *plat_.console;
  if (!plat_.imu->present()) {
    c.printf("IMU absent (WHO_AM_I read 0x%02X, expected 0x6C)\n", plat_.imu->whoAmI());
    c.println("  gravity is held at the default -y, so the sim still runs level");
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
  char* argv[4] = {nullptr, nullptr, nullptr, nullptr};
  int argc = 0;
  for (char* p = line; *p && argc < 4;) {
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

    case 'b':
      if (argc >= 2) {
        const int v = atoi(argv[1]);
        plat_.display->setBrightness((uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v)));
        c.printf("brightness %d\n", v);
      }
      break;

    case 'm':
      if (argc >= 4) {
        ChainMap& cm = plat_.display->chain();
        const int f = atoi(argv[1]);
        FaceMount m{(uint8_t)f, (uint8_t)atoi(argv[2]), (uint8_t)atoi(argv[3])};
        if (f >= 0 && f < cm.count()) m.slot = cm.mount(f).slot;
        if (f < 0 || f >= cm.count() || !cm.setMount(f, m)) {
          c.println("rejected: that would not be a valid mount table");
        }
      }
      printMounts();
      break;

    case 't':
      showTestPattern_ = true;
      c.println("test pattern: white dot at texel (1,1), red arm +x (3), green arm +y (5)");
      c.println("adjust with `m <face> <rot> <mirror>` until every face reads the same");
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

#pragma GCC diagnostic pop

}  // namespace app
}  // namespace partsim
