#include "CoreParallel.h"

namespace {
// Below this a split costs more in synchronisation than it saves. Two notifications are a few
// microseconds; a hundred particles of density pass is tens.
constexpr int kMinSplit = 96;
}  // namespace

bool CoreParallel::begin(int core, UBaseType_t priority) {
  return xTaskCreatePinnedToCore(workerEntry, "par", 4096, this, priority, &worker_, core) == pdPASS;
}

void CoreParallel::workerEntry(void* selfv) {
  CoreParallel* self = (CoreParallel*)selfv;
  for (;;) {
    // Wait for a job. ulTaskNotifyTake with clear-on-exit is the cheapest handshake FreeRTOS has.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    self->fn_(self->ctx_, self->begin_, self->end_);
    // Hand completion back to whoever posted the job.
    xTaskNotifyGive(self->caller_);
  }
}

void CoreParallel::forRange(int n, void* ctx, partsim::RangeFn fn) {
  if (n <= 0) return;
  if (!worker_ || n < kMinSplit) {
    fn(ctx, 0, n);
    return;
  }

  // The caller takes the FIRST half and the worker the second. Which half goes where does not
  // affect the result -- that is the Parallel.h contract, and tests/test_solver.cpp proves it by
  // running chunks in reverse -- so this is purely about the caller having useful work while the
  // worker starts.
  const int mid = n / 2;
  caller_ = xTaskGetCurrentTaskHandle();
  ctx_ = ctx;
  fn_ = fn;
  begin_ = mid;
  end_ = n;
  xTaskNotifyGive(worker_);

  fn(ctx, 0, mid);

  // Block until the worker reports. A missed notification here would hang the step task, so this
  // deliberately has no timeout: silently continuing while another core still writes the arrays
  // this step is about to read would corrupt the simulation instead of stopping it.
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}
