#pragma once

namespace partsim {

// How the solver reaches a second core, without core/ knowing what a core is.
//
// core/ has no platform headers by construction -- it compiles for the host, for WASM, for Xtensa
// and (verified) for Cortex-M and RISC-V. So the threading is injected: the device supplies an
// implementation backed by its own scheduler, and everything else gets the serial default and is
// bit-identical to what it was.
//
// THE CONTRACT, and it is narrow on purpose:
//
//   A range function may be run concurrently on disjoint sub-ranges of [0, n), in any split, and
//   the result must not depend on the split.
//
// That holds only for a loop where element i writes element i's state and reads nothing any other
// element writes. Every use in this project is checked against that by hand and named at the call
// site. It is emphatically NOT true of the solver's correction passes, which are Gauss-Seidel --
// particle i sees the correction already applied to i-1 -- and those need cell colouring instead.
// Handing one of those to forRange would produce a result that changed with the worker count, and
// the golden hashes would catch it on the first run.
using RangeFn = void (*)(void* ctx, int begin, int end);

class Parallel {
 public:
  virtual ~Parallel() = default;

  // Runs fn over [0, n), split however the implementation likes. The default runs it whole, on the
  // calling thread, which is what every host and browser build does.
  virtual void forRange(int n, void* ctx, RangeFn fn) { fn(ctx, 0, n); }

  // Workers available, for callers that want to know whether splitting is worth the sync. 1 means
  // forRange is a plain call.
  virtual int workers() const { return 1; }
};

// The default. A single shared instance so a null check is never needed.
Parallel& serialParallel();

}  // namespace partsim
