#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "partsim/Parallel.h"

// partsim::Parallel backed by the S3's second core.
//
// Measured ceiling before this was written: 1.98x on the solver's gather shape, with the second
// core costing about 1% (DECISIONS.md D44). The SRAM is multi-banked and the bus matrix serves
// both CPUs concurrently, so the contention that was expected is not there.
//
// One PERSISTENT worker, not a task per call. forRange runs several times per step, and task
// creation is hundreds of microseconds against a sync that needs to be a few -- creating one per
// call would spend more than the second core earns.
class CoreParallel : public partsim::Parallel {
 public:
  // `core` is the core the WORKER runs on; the caller keeps its own. Priority should match the
  // calling task so neither starves the other half of a split.
  bool begin(int core, UBaseType_t priority);

  void forRange(int n, void* ctx, partsim::RangeFn fn) override;
  int workers() const override { return worker_ ? 2 : 1; }

 private:
  static void workerEntry(void* self);

  TaskHandle_t worker_ = nullptr;
  TaskHandle_t caller_ = nullptr;
  // The job, published before the worker is notified and read only after. No lock: the
  // notification is the handshake, and FreeRTOS notifications carry the necessary ordering.
  void* ctx_ = nullptr;
  partsim::RangeFn fn_ = nullptr;
  volatile int begin_ = 0, end_ = 0;
};
