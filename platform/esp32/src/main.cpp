// partsim firmware: bring-up and scheduling. The application itself is in platform/app.
//
// What is left in this file is, deliberately, only the two things that cannot be platform-neutral:
// wiring up concrete drivers, and deciding WHEN a frame runs. Everything the firmware actually
// does -- the role logic, the IMU ring, the console, the benchmark, the determinism sequence, the
// per-frame body of every task -- is partsim::app::App, which compiles for the host and is run
// there by platform/host/console_main.cpp. See docs/W3-HANDOFF.md.
//
// Task layout, which is the one structural decision worth explaining:
//
//   core 1, prio 2   simTask   IMU fusion, solver, splat, blit, buffer flip. All the float.
//   core 0, prio 3   imuTask   sensor reads only. Integer-only, by construction.
//   core 1, prio 1   loop()    the serial console. Arduino's loopTask, already on core 1.
//
// FreeRTOS on Xtensa saves FPU context lazily, so float-using tasks must be pinned rather than
// left floating between cores. Both are pinned. imuTask is integer-only (see Lsm6dsox.h) so the
// two cores never contend for the FPU at all, and it runs at a HIGHER priority than the
// simulation despite doing less work -- it has a 4.8ms sample deadline, while a late frame is
// merely a late frame.
//
// simTask blocks in vTaskDelayUntil every frame, which is what lets the console and the idle
// task run at all.
#include <Arduino.h>
#include <cstring>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "Lsm6dsox.h"
#include "CoreParallel.h"
#include "PanelDriver.h"
#include "Pins.h"
#include "RoleStraps.h"
#if !PARTSIM_QEMU
#include "SpiFrameLink.h"
#endif
#include "partsim/app/App.h"

using namespace partsim;
using namespace partsim::app;

namespace {

// --- the platform, which is all this file is ----------------------------------------------------

// The console over UART0. `write` rather than a printf of its own: Console formats into a fixed
// buffer and hands down a finished string, which is what Serial.printf does internally anyway.
class SerialConsole final : public Console {
 public:
  void write(const char* s) override { Serial.print(s); }

  int readLine(char* buf, int cap) override {
    while (Serial.available()) {
      const int ch = Serial.read();
      if (ch == '\r' || ch == '\n') {
        if (len_ == 0) continue;
        const int out = len_ < cap - 1 ? len_ : cap - 1;
        memcpy(buf, line_, (size_t)out);
        buf[out] = '\0';
        len_ = 0;
        return out;
      }
      if (len_ + 1 < (int)sizeof(line_)) line_[len_++] = (char)ch;
    }
    return -1;
  }

 private:
  char line_[64];
  int len_ = 0;
};

class ArduinoClock final : public Clock {
 public:
  uint32_t micros() const override { return ::micros(); }
  uint32_t millis() const override { return ::millis(); }
  void delayMs(uint32_t ms) override { ::delay(ms); }
};

class Esp32Hooks final : public SystemHooks {
 public:
  uint32_t freeHeap() const override {
    return (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  }
  uint32_t largestHeapBlock() const override {
    return (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  }
  uint32_t freePsram() const override {
    return (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  }

  // The load-bearing half. See SystemHooks::suspendSim: a console command that rebuilds the
  // Simulation must stop the task drawing from it, and PAUSING IS NOT ENOUGH, because that task
  // calls accumulate() every frame whether or not the physics is paused. Dropping this
  // reintroduces a LoadProhibited inside Renderer::clear().
  void suspendSim() override { if (g_stepTask) vTaskSuspend(g_stepTask); }
  void resumeSim() override { if (g_stepTask) vTaskResume(g_stepTask); }

  // The stepping task's handle, whichever role's task it is. Null until setup() creates it, which
  // is why the QEMU build can run the determinism sequence from setup() at all.
  static TaskHandle_t g_stepTask;
};
TaskHandle_t Esp32Hooks::g_stepTask = nullptr;

SerialConsole g_console;
ArduinoClock g_clock;
Esp32Hooks g_hooks;
Lsm6dsox g_imu;

// PanelDriver is safe to hand over unconditionally: every entry point is guarded on its DMA
// pointer, so a board that never calls begin() -- a master, the QEMU build -- gets a driver that
// does nothing and reports ready() == false, which is precisely what the benchmark asks it.
PanelDriver g_panels;

// A link that carries nothing, so a single-board build and the QEMU environment run the same code
// paths as a real one without a peer. setup() swaps in the SPI carrier where there is one.
NullFrameLink g_nullLink;
#if PARTSIM_MULTINODE && !PARTSIM_QEMU
SpiMasterLink g_spiMaster;
SpiDisplayLink g_spiDisplay;
#endif

CoreParallel g_coreParallel;
Platform g_plat{&g_console, &g_clock, &g_panels, &g_imu, &g_nullLink, &g_hooks};
App g_app(g_plat);  // ~137KB of pools, so global rather than anywhere near a stack

Role g_role = Role::Master;

// --- scheduling ---------------------------------------------------------------------------------

constexpr TickType_t kFramePeriod = pdMS_TO_TICKS(1000 / kTargetFps);

// Waits for the next frame boundary, or gives up on catching up if the boundary has gone.
//
// vTaskDelayUntil returns IMMEDIATELY once the deadline has already passed, so a task that cannot
// hit its frame period stops yielding entirely and the priority-1 console task never runs again --
// the device looks hung when it is merely late. Found by running under QEMU, which is slow enough
// to trigger it every frame, but a real board that falls behind (too many particles, a slow blit)
// would starve exactly the same way, and would do it precisely when someone needs the console to
// find out why.
void frameYield(TickType_t& next) {
  if ((int32_t)(xTaskGetTickCount() - next) >= 0) {
    next = xTaskGetTickCount();  // give up on catching up rather than spinning
    g_app.noteOverrun();
    vTaskDelay(1);               // one tick to anything below us
  } else {
    vTaskDelayUntil(&next, kFramePeriod);
  }
}

void imuTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(5);  // 208Hz is 4.8ms; the tick is 1ms
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    g_app.imuPoll();
    vTaskDelayUntil(&next, period);
  }
}

#if !PARTSIM_MULTINODE
void simTask(void*) {
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    g_app.simStep();
    frameYield(next);
  }
}
#else
#ifdef PARTSIM_PROFILE_ESP32_MASTER
void masterTask(void*) {
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    g_app.masterStep();
    frameYield(next);
  }
}
#endif
#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
void displayTask(void*) {
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    g_app.displayStep();
    frameYield(next);
  }
}
#endif
#endif

[[noreturn]] void fatal() {
  for (;;) delay(1000);
}

}  // namespace

// --- bring-up -----------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  // Long enough for the USB CDC to come up on a devkit, short enough not to matter when the
  // cube is running standalone off a power supply with nothing listening.
  const uint32_t deadline = millis() + 1500;
  while (!Serial && millis() < deadline) delay(10);

  g_console.println();
  g_console.println("partsim firmware");
  g_console.printf("faces %d, %dx%d, %d-bit colour, target %d fps\n", kFaces, kPanelRes, kPanelRes,
                   kColourDepthBits, kTargetFps);
#ifdef PARTSIM_NONDETERMINISTIC_FP
  g_console.println("build: cube-fast -- FMA contraction on, determinism check will NOT match");
#endif

  // Radio off before anything else claims memory, EXCEPT on a node that chains beakers.
  //
  // The ~55KB this is said to cost is a HEAP figure and the linker cannot see it: building with
  // WiFi STA and esp_now_init() moves the static total by 68 bytes, because WiFi allocates its
  // buffers at esp_wifi_init(). So the cost has to be read off a running board, and
  // PARTSIM_ENABLE_RADIO exists to make that measurable rather than assumed.
  //
  // Display nodes stay silent whatever the flag says: they sit inside the panel stack where HUB75
  // emissions are worst, they are the boards with the tightest budget, and a radio ISR against a
  // 16MHz display clock is the jitter this project spent M2 avoiding (DECISIONS.md D34).
#if PARTSIM_ENABLE_RADIO && !defined(PARTSIM_PROFILE_ESP32_DISPLAY)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();  // STA for the PHY, not for an access point
  if (esp_now_init() != ESP_OK) g_console.println("WARNING: esp_now_init failed; no chaining");
#else
  WiFi.mode(WIFI_OFF);
  esp_wifi_deinit();
#endif
#if CONFIG_BT_ENABLED
  btStop();
#endif

#if PARTSIM_MULTINODE
  g_role = readRole(pins::kRoleA, pins::kRoleB);
  g_console.printf("role: %s  (straps IO%d/IO%d)\n", roleName(g_role), pins::kRoleA, pins::kRoleB);
#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
  if (digitalRead(pins::kRoleA) && digitalRead(pins::kRoleB)) {
    g_console.println("  straps are open -- defaulted to display0. Jumper IO38 and/or IO39 to GND");
    g_console.println("  to select display1 (IO38=GND) or display2 (IO39=GND).");
  }
#endif
#endif

  if (!g_app.begin(g_role)) fatal();

  // --- panels ---
#if PARTSIM_MULTINODE
  if (!roleDrivesPanels(g_role)) {
    g_console.println("master: no panels on this board");
  } else
#endif
#if PARTSIM_QEMU
  // No panels under emulation. QEMU models the CPU, RAM, flash, timers and UART, but not LCD_CAM
  // or GDMA -- and the HUB75 library spins waiting for a DMA completion that never arrives, which
  // starves setup() before it can even report the hang. Nothing about the display path is
  // testable here; everything above it is.
  g_console.println("QEMU build: panel driver skipped (no LCD_CAM/GDMA model)");
#else
  {
#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
    // Drive only this node's own faces. The panel table has six entries; this board has two tiles.
    const RoleFaces rf = facesFor(g_role);
    const bool ok = g_panels.begin(g_app.geometry(), rf.face, rf.count, kColourDepthBits,
                                   kDefaultBrightness);
#else
    const bool ok = g_panels.begin(g_app.geometry(), kColourDepthBits, kDefaultBrightness);
#endif
    if (!ok) {
      g_console.println("FATAL: HUB75 init failed -- check Pins.h against the wiring");
      fatal();
    }
    // Two separate facts, and the second is a self-test rather than a claim: allRunsHorizontal
    // says the MOUNT TABLE permits the row-walking blit, fastBlit says the DMA buffer layout
    // actually verified against drawPixelRGB888 at boot (PanelFramebuffer.h).
    g_console.printf("panels: chain %dx%d, rows %s, blit %s\n", g_panels.chain().chainWidth(),
                     g_panels.chain().chainHeight(),
                     g_panels.allRunsHorizontal(g_app.geometry()) ? "all horizontal"
                                                                 : "some vertical",
                     g_panels.fastBlit() ? "row-walking" : "per-texel");
  }
#endif

  // --- IMU ---
  {
    MotionConfig mcfg = MotionConfig::defaults();
    // The axis map is the one thing that cannot be guessed: it depends on how the breakout is
    // glued in. Identity until the real object exists -- use the `i` command with the cube resting
    // on each face in turn to work out the permutation, then set it here.
    g_app.initMotion(mcfg, AxisMap::identity());
  }
  if (!g_imu.begin(pins::kSda, pins::kScl, pins::kI2cHz)) {
    // Not fatal on purpose: a cube with a dead IMU should still be a lamp. Gravity stays at the
    // default -y and the fluid simply sits at the bottom.
    g_console.printf("WARNING: no LSM6DSOX (WHO_AM_I 0x%02X); running with fixed gravity\n",
                     g_imu.whoAmI());
  } else {
    g_console.println("IMU: LSM6DSOX at 208 Hz, +-8 g / +-500 dps");
    xTaskCreatePinnedToCore(imuTask, "imu", 2560, nullptr, 3, nullptr, 0);
  }

#if PARTSIM_QEMU
  // Run it here, before the stepping task exists. The emulator cannot keep a 30fps cadence, so a
  // running task would fight the console for the whole session -- and the determinism sequence is
  // the only thing this environment is for.
  g_console.println("QEMU: running the determinism sequence");
  g_app.runGolden();
  g_console.println("QEMU: done");
#endif

  // --- the stepping task. One image, every board: which one runs is a fact about the straps. ---
#if PARTSIM_MULTINODE
#ifdef PARTSIM_PROFILE_ESP32_MASTER
#if !PARTSIM_QEMU
  {
    const int cs[3] = {pins::kSpiCsOut[0], pins::kSpiCsOut[1], pins::kSpiCsOut[2]};
    if (g_spiMaster.begin(pins::kSpiSck, pins::kSpiMosi, pins::kSpiMiso, cs, 3, 20 * 1000 * 1000)) {
      g_plat.link = &g_spiMaster;
    } else {
      g_console.println("WARNING: SPI host init failed; running with a null link");
    }
  }
#endif
  xTaskCreatePinnedToCore(masterTask, "master", 6144, nullptr, 2, &Esp32Hooks::g_stepTask, 1);
#endif
#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
#if !PARTSIM_QEMU
  if (g_spiDisplay.begin(pins::kSpiSck, pins::kSpiMosi, pins::kSpiMiso, pins::kSpiCsIn)) {
    g_plat.link = &g_spiDisplay;
  } else {
    g_console.println("WARNING: SPI device init failed; this node will never receive a frame");
  }
#endif
  xTaskCreatePinnedToCore(displayTask, "display", 6144, nullptr, 2, &Esp32Hooks::g_stepTask, 1);
#endif
#else
  // 6KB of stack: the solver recurses nowhere and every pool is static, so this is generous.
  // The solver's second core. Started before the step task so the worker is already parked on its
  // notification when the first frame runs, and pinned to core 0, which otherwise holds only the
  // IMU poll. Priority matches the step task: a lower one would let imuTask stall half of every
  // split, which shows up as a frame time that is occasionally double.
  if (g_coreParallel.begin(0, 2)) {
    g_app.setParallel(&g_coreParallel);
  } else {
    g_console.println("WARNING: second-core worker failed to start; solver stays single-core");
  }
  xTaskCreatePinnedToCore(simTask, "sim", 6144, nullptr, 2, &Esp32Hooks::g_stepTask, 1);
#endif

  g_console.printf("internal heap free after init: %u B\n", (unsigned)g_hooks.freeHeap());
  g_console.println("ready -- ? for help");
}

void loop() {
  // The console. Runs as Arduino's loopTask on core 1 at priority 1, below the stepping task, so
  // it only gets time while the simulation is blocked in vTaskDelayUntil.
  g_app.consolePoll();
  delay(10);
}
