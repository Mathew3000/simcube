#pragma once
#include <cstdint>

#include "partsim/app/Clock.h"
#include "partsim/app/Console.h"
#include "partsim/app/Display.h"
#include "partsim/app/FrameLink.h"
#include "partsim/app/MotionSensor.h"

#include "partsim/SpillChain.h"

namespace partsim {
namespace app {

// The two things the application needs from its host that are not a device.
class SystemHooks {
 public:
  virtual ~SystemHooks() = default;

  // Memory, as the `r` command reports it. Zero where the concept does not apply -- a host build
  // has a heap, but not one whose exhaustion is the interesting failure.
  virtual uint32_t freeHeap() const { return 0; }
  virtual uint32_t largestHeapBlock() const { return 0; }
  virtual uint32_t freePsram() const { return 0; }

  // Stops and restarts the task that is stepping the simulation.
  //
  // THIS IS LOAD-BEARING, not a tidiness hook. The `g` and `x` commands call Simulation::init,
  // which tears down and rebuilds the renderer's slot tables and every pool. The stepping task
  // runs at a higher priority than the console, so without this it preempts mid-init and renders
  // through a half-built Renderer -- observed on hardware as a LoadProhibited inside
  // Renderer::clear().
  //
  // Pausing is NOT sufficient and never was: the step task calls accumulate() every frame whether
  // or not the physics is paused, and accumulate() is what reads the tables being rebuilt. A
  // platform with no such task (the host, the QEMU build, which runs the sequence before its task
  // exists) correctly implements both as nothing.
  virtual void suspendSim() {}
  virtual void resumeSim() {}
};

// Everything the application is given. Assembled by each platform's main(), which is the only
// place that names a concrete driver.
//
// `display`, `imu` and `link` are never null -- a platform that has none of a thing passes the
// Null implementation rather than a null pointer, so the application has no absent-device branch
// to get wrong. That is not a style preference: the master role, the QEMU environment and the
// single-panel bring-up build each used to carry their own #if around the same call sites.
struct Platform {
  Console* console = nullptr;
  Clock* clock = nullptr;
  Display* display = nullptr;
  MotionSensor* imu = nullptr;
  FrameLink* link = nullptr;
  // The chain between cubes, for beaker mode. Defined in core/ rather than here because the pump
  // above it is core's and the tests link only core -- see SpillChain.h. Null-carrier by default,
  // so a board with no radio and a board whose radio failed to start run the same path.
  SpillTransport* chain = &nullSpillTransport();
  SystemHooks* hooks = nullptr;
};

}  // namespace app
}  // namespace partsim
