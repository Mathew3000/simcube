#pragma once
#include <cstdint>

#include "partsim/app/AppConfig.h"
#include "partsim/app/Platform.h"
#include "partsim/app/Role.h"

#include "partsim/MotionSource.h"
#if PARTSIM_MULTINODE
#include "partsim/RenderState.h"
#include "partsim/SimFrame.h"
#endif
#ifndef PARTSIM_PROFILE_ESP32_DISPLAY
#include "partsim/Simulation.h"
#endif

namespace partsim {
namespace app {

// The firmware, minus the hardware and minus the scheduler.
//
// Everything here used to be file-scope statics and free functions in platform/esp32/src/main.cpp,
// which meant it could only be compiled for Xtensa and therefore could only be exercised by
// flashing a board. That is the actual cost being paid down: the simulation master drives no
// panels, so a non-ESP solver board needs no HUB75 driver at all -- but with the application
// logic welded to Arduino it would still have needed a rewrite of all of it.
//
// Two rules keep the seam honest:
//
//   * App does not know FreeRTOS exists. It owns a FRAME; the platform owns WHEN a frame runs.
//     Every task in main.cpp is now a period, a deadline and a call to one of the *Step methods
//     below. That split is deliberate -- the deadline handling is genuinely FreeRTOS-specific
//     (see the vTaskDelayUntil starvation note in main.cpp) and does not generalise.
//
//   * App talks to devices only through Platform. A platform that lacks a device passes the Null
//     implementation, so there is no absent-hardware branch in here to get wrong.
//
// It compiles for the host, and that is the point rather than a bonus: if it only built under
// PlatformIO the coupling would still be there and nobody would find out until the port.
class App {
 public:
  explicit App(const Platform& plat) : plat_(plat) {}

  // Frame timings, exponentially smoothed so the console shows a readable number rather than the
  // jitter of whichever frame happened to be sampled. What each field means depends on the role:
  // on a master `renderMs` is encode-and-send, on a display node `simMs` is receive-and-decode.
  struct Stats {
    float simMs = 0.0f, renderMs = 0.0f, blitMs = 0.0f, fps = 0.0f;
    uint32_t frames = 0;
    int substeps = 0;
  };

  // Builds the state this role needs and reports what it built. False is fatal -- what fatal
  // means is the platform's decision, because on a device it is "print and spin" and on the host
  // it is an exit code.
  bool begin(Role role);

  // --- one frame, per role. The platform calls exactly one of these on a period. -------------

#if !PARTSIM_MULTINODE
  // Single board: solve, splat, blit.
  void simStep();
#else
#ifdef PARTSIM_PROFILE_ESP32_MASTER
  // Solve, encode, send. No panels.
  void masterStep();
#endif
#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
  // Receive, decode, splat, blit. No solver, no field advection, no IMU.
  void displayStep();
#endif
#endif

  // One sensor read into the ring, called from whatever the platform samples with.
  //
  // INTEGER ONLY, by construction -- it stores raw LSB and converts nothing. On Xtensa this is
  // what lets the sampling task be pinned to the other core without the two ever contending for
  // a lazily-switched FPU context. Do not make this convert to g and rad/s "while it is here":
  // the conversion belongs next to the filter, which runs in simStep().
  void imuPoll();

  // At most one console line, read and dispatched. Non-blocking on the input; the command itself
  // may well block for tens of seconds, which is why the caller is the lowest-priority task.
  void consolePoll();

  // A frame the scheduler could not place on time. Counted rather than ignored, because a device
  // silently running at half rate is worth knowing about -- it is the first thing `r` reports.
  void noteOverrun() { ++overruns_; }

#ifndef PARTSIM_PROFILE_ESP32_DISPLAY
  // --- console commands that the platform also wants at boot ---------------------------------
  // QEMU runs the determinism sequence from setup(), before any stepping task exists, because the
  // emulator cannot hold a 30fps cadence and a running task would fight the console all session.
  void runGolden();
#endif

  Console& console() const { return *plat_.console; }

  // The panel table. Bring-up needs it to size the display driver, which is the one thing that
  // has to happen after the application has built its geometry and before any frame runs.
#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
  const Geometry& geometry() const { return geom_; }
  const SimVolume& volume() const { return vol_; }
#else
  const Geometry& geometry() const { return sim_.geometry(); }
  const SimVolume& volume() const { return sim_.volume(); }
#endif

  // The orientation filter's configuration and the IMU axis permutation. Platform-supplied
  // because the axis map is a fact about how the breakout is glued into the object, which no
  // amount of code can work out for itself.
  void initMotion(const MotionConfig& cfg, const AxisMap& axes) { motion_.init(cfg, axes); }

 private:
  void printHelp();
  void printMounts();
  void printImu();
  void printStats();
  void handleLine(char* line);
#ifndef PARTSIM_PROFILE_ESP32_DISPLAY
  void runBench();
#endif

  // Drains the IMU ring into the filter and returns how many samples it consumed.
  //
  // The filter runs once per SAMPLE rather than once per frame, so it sees its designed 208Hz
  // whatever the frame rate -- which is also what makes the host tests in tests/test_motion.cpp
  // representative of the device.
  int pumpMotion();

  // Suspends the stepping task for a scope, around anything that rebuilds the Simulation.
  // See SystemHooks::suspendSim -- this is not optional and pausing is not a substitute.
  struct SuspendSim {
    explicit SuspendSim(const Platform& p) : hooks(p.hooks) { if (hooks) hooks->suspendSim(); }
    ~SuspendSim() { if (hooks) hooks->resumeSim(); }
    SystemHooks* hooks;
  };

  const Platform& plat_;

  // Simulation state, strictly by role.
  //
  // A display node needs the GEOMETRY and the container BOX -- roughly half a kilobyte -- to place
  // splats and to dequantise positions. An earlier draft gave it a whole Simulation to obtain
  // them, which is 136.6KB for 0.5KB of use and exactly the mistake RenderState was written to
  // prevent.
#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
  Geometry geom_;
  SimVolume vol_;
  RenderParticles rxParticles_;
  HeatBuffer rxHeat_;
  Renderer rxRenderer_;
  uint32_t rxLastStep_ = 0;
  uint32_t rxAccepted_ = 0;
  uint32_t rxRejected_ = 0;
  uint32_t rxStarved_ = 0;  // polls that found nothing: the node holds its last frame
#else
  Simulation sim_;
#endif
#ifdef PARTSIM_PROFILE_ESP32_MASTER
  uint8_t txFrame_[frameMaxBytes()];
  uint32_t masterStep_ = 0;
#endif

  MotionSource motion_;
  Role role_ = Role::Master;

  // --- IMU sample ring ---------------------------------------------------------------------
  // Single producer (imuPoll), single consumer (pumpMotion), power-of-two, so the indices wrap by
  // masking and neither side needs a lock.
  static constexpr int kRingSize = 64;  // ~0.3s of slack; far more than a frame
  ImuSample ring_[kRingSize];
  volatile uint32_t head_ = 0;  // written by the producer only
  volatile uint32_t tail_ = 0;  // written by the consumer only
  volatile uint32_t dropped_ = 0;

  Stats stats_;
  volatile bool paused_ = false;
  volatile bool showTestPattern_ = false;
  volatile uint32_t overruns_ = 0;

  // simStep's own fps window, which is per-second rather than per-frame.
  uint32_t lastReport_ = 0;
  uint32_t framesSince_ = 0;
  bool reportSeeded_ = false;

#ifndef PARTSIM_PROFILE_ESP32_DISPLAY
  // One face of RGB, for timing the resolve pass without needing the panel driver. Only the
  // benchmark uses it, and a display node has no benchmark, so it is not compiled there.
  //
  // That guard buys nothing and is kept only because it is true: as a file-scope static the old
  // version was already dead-stripped out of the display build, so the measured RAM does not
  // move. Worth writing down, because "12KB off the display node" is exactly the plausible
  // saving one would otherwise claim here without building it.
  uint8_t staging_[kMaxPanelTexels * 3];
#endif
};

}  // namespace app
}  // namespace partsim
