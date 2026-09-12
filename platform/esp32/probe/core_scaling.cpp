// Does the ESP32-S3's second core actually buy anything for THIS workload?
//
// DECISIONS.md P2 proposes eight-colour cell partitioning to parallelise the solver, and values it
// at ~1.8x. That multiplier is an estimate, and this project has been burned four times by an
// unmeasured multiplier driving a large change (DECISIONS.md §9, R3-R6). So: measure the ceiling
// before building the thing.
//
// There is a specific reason to doubt it. The solver's cost is dominated by a scattered gather --
// 88 candidate reads per particle per step, 27% of them useful -- and the measured CPI of 1.40 is
// largely memory stalls. The S3's internal SRAM is directly addressed with no per-core data cache,
// so two cores gathering from the same arrays contend at the bus matrix rather than in a cache.
// Whether that costs 5% or 50% is not something to reason about from a block diagram.
//
// What this measures: the same gather-and-float kernel on one core, then on both simultaneously,
// against a working set the size of the real solver's. It reports the aggregate speedup.
//
// What it does NOT measure: the real solver. This kernel has no writes to shared state, no
// synchronisation, and no barrier between colour classes. Every one of those makes the real thing
// WORSE, so treat the number here as an upper bound on P2's benefit -- if this says 1.3x, P2
// cannot deliver 1.8x and the estimate is wrong.
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "partsim/Math.h"

using namespace partsim;

namespace {

// Sized to the real solver's hot state at the device profile: ~512 particles of position, plus a
// neighbour list that is the thing actually being streamed. Small enough to sit in internal SRAM,
// which is the case that matters -- the whole question is SRAM contention.
constexpr int kParticles = 512;
constexpr int kNeighbours = 64;

float g_x[kParticles], g_y[kParticles], g_z[kParticles];
uint16_t g_nb[kParticles * kNeighbours];

// Per-core results. Separate cache lines are irrelevant on internal SRAM but the arrays are
// per-core anyway so there is no write sharing to muddy the measurement.
volatile float g_sink[2];
volatile uint32_t g_us[2];
SemaphoreHandle_t g_done[2];
volatile int g_reps = 0;

// The inner loop, deliberately shaped like Solver's pass A+B: walk a neighbour list, read three
// coordinates per neighbour, reject on squared distance, and do poly6-ish arithmetic on the rest.
// No divides or square roots -- those are gone from the real solver too (DECISIONS.md F1).
float gatherKernel(int firstParticle, int stride, int reps) {
  const float h2 = 36.0f;  // kSmoothRadius^2 at the shipping rest spacing
  float acc = 0.0f;
  for (int r = 0; r < reps; ++r) {
    for (int i = firstParticle; i < kParticles; i += stride) {
      const float xi = g_x[i], yi = g_y[i], zi = g_z[i];
      const uint16_t* nb = g_nb + (size_t)i * kNeighbours;
      for (int k = 0; k < kNeighbours; ++k) {
        const int j = nb[k];
        const float dx = xi - g_x[j], dy = yi - g_y[j], dz = zi - g_z[j];
        const float r2 = dx * dx + dy * dy + dz * dz;
        if (r2 >= h2) continue;
        const float d = h2 - r2;
        acc += d * d * d;
      }
    }
  }
  return acc;
}

// Each worker does the SAME amount of work -- every particle -- rather than splitting the set.
// Splitting would measure how fast half the job finishes; running both whole measures whether the
// memory system can serve two gathers at once, which is the question.
void worker(void* arg) {
  const int id = (int)(intptr_t)arg;
  const uint32_t t0 = micros();
  const float v = gatherKernel(0, 1, g_reps);
  g_us[id] = micros() - t0;
  g_sink[id] = v;
  xSemaphoreGive(g_done[id]);
  vTaskDelete(nullptr);
}

void runOnce(int reps) {
  g_reps = reps;

  // One core.
  xTaskCreatePinnedToCore(worker, "solo", 4096, (void*)0, 2, nullptr, 1);
  xSemaphoreTake(g_done[0], portMAX_DELAY);
  const uint32_t solo = g_us[0];

  // Both cores, started as close together as the scheduler allows and timed on wall clock, so any
  // stagger counts against the parallel case rather than being hidden.
  const uint32_t w0 = micros();
  xTaskCreatePinnedToCore(worker, "p0", 4096, (void*)0, 2, nullptr, 0);
  xTaskCreatePinnedToCore(worker, "p1", 4096, (void*)1, 2, nullptr, 1);
  xSemaphoreTake(g_done[0], portMAX_DELAY);
  xSemaphoreTake(g_done[1], portMAX_DELAY);
  const uint32_t wall = micros() - w0;

  Serial.printf("  reps %3d | solo %7u us | dual core0 %7u core1 %7u wall %7u us | speedup %.2fx\n",
                reps, solo, g_us[0], g_us[1], wall, (2.0f * (float)solo) / (float)wall);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println();
  Serial.println("ESP32-S3 two-core scaling probe for the solver's gather");
  Serial.printf("  working set: %u B positions + %u B neighbour lists, all internal SRAM\n",
                (unsigned)(3 * sizeof(g_x)), (unsigned)sizeof(g_nb));
  Serial.printf("  free internal heap %u B\n", (unsigned)ESP.getFreeHeap());

  // A fixed pseudo-random neighbour pattern. Not the real grid, but the real grid's ACCESS shape:
  // each particle reads ~64 scattered indices, which is what the bus sees.
  uint32_t seed = 0x12345678u;
  auto next = [&seed]() {
    seed ^= seed << 13;
    seed ^= seed >> 17;
    seed ^= seed << 5;
    return seed;
  };
  for (int i = 0; i < kParticles; ++i) {
    g_x[i] = (float)(next() % 3200) * 0.01f;
    g_y[i] = (float)(next() % 3200) * 0.01f;
    g_z[i] = (float)(next() % 3200) * 0.01f;
  }
  for (int i = 0; i < kParticles * kNeighbours; ++i) g_nb[i] = (uint16_t)(next() % kParticles);

  for (int i = 0; i < 2; ++i) g_done[i] = xSemaphoreCreateBinary();

  for (int reps : {4, 8, 16, 32}) runOnce(reps);

  Serial.println();
  Serial.println("  speedup is an UPPER BOUND on what P2 can deliver: this kernel has no shared");
  Serial.println("  writes, no barriers between colour classes and no scheduling overhead.");
  Serial.println("PROBE: done");
}

void loop() { delay(1000); }
