#pragma once
#include <cstdint>

namespace partsim {
namespace app {

// Time and yielding.
//
// Microseconds, not milliseconds, because the thing this clock is mostly used for is the frame
// breakdown the `r` and `x` commands print -- and a 2.60 ms resolve pass measured in whole
// milliseconds is the number 3, which is not a measurement.
//
// Both counters are free-running 32-bit and wrap: micros() every ~71 minutes, millis() every ~49
// days. Every user here takes a DIFFERENCE of two samples in unsigned arithmetic, which is
// correct across the wrap; nothing compares two absolute stamps.
class Clock {
 public:
  virtual ~Clock() = default;

  virtual uint32_t micros() const = 0;
  virtual uint32_t millis() const = 0;

  // Yields for at least `ms`. On the device this is a scheduler delay, not a spin -- the console
  // task sits below the simulation and must give its time away.
  virtual void delayMs(uint32_t ms) = 0;
};

}  // namespace app
}  // namespace partsim
