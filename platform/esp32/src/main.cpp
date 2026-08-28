// partsim firmware: the simulation from the browser, running inside the object.
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
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <esp_wifi.h>

#include "Lsm6dsox.h"
#include "PanelDriver.h"
#include "Pins.h"
#include "partsim/MotionSource.h"
#include "partsim/Simulation.h"

using namespace partsim;

namespace {

// How many faces this build drives. One for bringing up a single panel before six exist; the
// simulation's slab mode is the same 3D code path, just a thin volume.
#ifdef PARTSIM_FACE_COUNT
constexpr int kFaces = PARTSIM_FACE_COUNT;
#else
constexpr int kFaces = 6;
#endif

// Panel resolution comes from core's capacity profile, not a local copy. There used to be a
// duplicate here that was only used for the boot banner -- so it could not break anything, but it
// could print a lie, which is worse in a message someone reads during bring-up.
constexpr uint8_t kColorDepthBits = 6;
constexpr uint8_t kDefaultBrightness = 96;
constexpr int kTargetFps = 30;
constexpr float kImuDt = 1.0f / 208.0f;

// Static storage. ~125KB at the device capacity profile; see scripts/check_esp32_budget.sh,
// which asserts the whole footprint fits internal SRAM before anything is ever flashed.
Simulation g_sim;
PanelDriver g_panels;
MotionSource g_motion;
Lsm6dsox g_imu;

// --- IMU sample ring ---------------------------------------------------------------------------
// Single producer (imuTask), single consumer (simTask), power-of-two, so the indices can wrap by
// masking and neither side needs a lock. The consumer drains everything pending and runs the
// filter once per sample, so the filter sees its designed 208Hz regardless of the frame rate --
// which is also what makes the host tests in tests/test_motion.cpp representative.
constexpr int kRingSize = 64;  // ~0.3s of slack; far more than a frame
Lsm6dsox::Raw g_ring[kRingSize];
volatile uint32_t g_head = 0;  // written by producer only
volatile uint32_t g_tail = 0;  // written by consumer only
volatile uint32_t g_dropped = 0;

// --- stats -------------------------------------------------------------------------------------
struct {
  float simMs, renderMs, blitMs, fps;
  uint32_t frames;
  int substeps;
} g_stats = {};

volatile bool g_paused = false;
volatile bool g_showTestPattern = false;
// Frames that missed their deadline. Reported by `r`, because a device silently running at half
// rate is worth knowing about.
volatile uint32_t g_overruns = 0;

// ------------------------------------------------------------------------------------------------

void imuTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(5);  // 208Hz is 4.8ms; the tick is 1ms
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    Lsm6dsox::Raw r;
    if (g_imu.present() && g_imu.read(r)) {
      const uint32_t head = g_head;
      const uint32_t nextHead = (head + 1) & (kRingSize - 1);
      if (nextHead != g_tail) {
        g_ring[head] = r;
        g_head = nextHead;
      } else {
        // Full means the consumer has stalled for a third of a second. Counted rather than
        // silently overwritten, because it would otherwise look like sensor noise.
        ++g_dropped;
      }
    }
    vTaskDelayUntil(&next, period);
  }
}

// Drains the ring into the filter. Returns how many samples were consumed.
int pumpMotion() {
  int n = 0;
  uint32_t tail = g_tail;
  while (tail != g_head) {
    const Lsm6dsox::Raw& r = g_ring[tail];
    const Vec3 accelG{(float)r.ax * Lsm6dsox::kAccelScaleG, (float)r.ay * Lsm6dsox::kAccelScaleG,
                      (float)r.az * Lsm6dsox::kAccelScaleG};
    const Vec3 gyro{(float)r.gx * Lsm6dsox::kGyroScaleRad, (float)r.gy * Lsm6dsox::kGyroScaleRad,
                    (float)r.gz * Lsm6dsox::kGyroScaleRad};
    if (!g_motion.seeded()) g_motion.seed(accelG);
    g_motion.update(accelG, gyro, kImuDt);
    tail = (tail + 1) & (kRingSize - 1);
    ++n;
  }
  g_tail = tail;
  return n;
}

void simTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(1000 / kTargetFps);
  TickType_t next = xTaskGetTickCount();
  uint32_t lastReport = millis();
  uint32_t framesSince = 0;

  for (;;) {
    if (g_showTestPattern) {
#if !PARTSIM_QEMU
      g_panels.testPattern(g_sim.geometry());
#endif
      g_showTestPattern = false;
      vTaskDelayUntil(&next, period);
      continue;
    }

    const uint32_t t0 = micros();
    pumpMotion();

    if (!g_paused) {
      // Object-space gravity and container acceleration: exactly the two vectors the browser
      // supplies from the mouse. Below this line the device and the browser run identical code.
      if (g_motion.seeded()) {
        g_sim.setGravityObject(g_motion.gravityObject());
        g_sim.setContainerAccel(g_motion.containerAccel());
      }
      g_stats.substeps = g_sim.advance(1.0f / (float)kTargetFps);
    }

    const uint32_t t1 = micros();
    g_sim.accumulate();
    const uint32_t t2 = micros();
#if !PARTSIM_QEMU
    g_panels.present(g_sim.renderer(), g_sim.geometry());
#endif
    const uint32_t t3 = micros();

    // Exponential smoothing, so the console shows a readable number rather than the jitter of
    // whichever frame happened to be sampled.
    const float k = 0.1f;
    g_stats.simMs += ((float)(t1 - t0) * 0.001f - g_stats.simMs) * k;
    g_stats.renderMs += ((float)(t2 - t1) * 0.001f - g_stats.renderMs) * k;
    g_stats.blitMs += ((float)(t3 - t2) * 0.001f - g_stats.blitMs) * k;
    ++g_stats.frames;
    ++framesSince;

    const uint32_t now = millis();
    if (now - lastReport >= 1000) {
      g_stats.fps = (float)framesSince * 1000.0f / (float)(now - lastReport);
      lastReport = now;
      framesSince = 0;
    }

    // vTaskDelayUntil returns IMMEDIATELY once the deadline has already passed, so a simTask that
    // cannot hit its frame period stops yielding entirely and the priority-1 console task never
    // runs again -- the device looks hung when it is merely late. Found by running under QEMU,
    // which is slow enough to trigger it every frame, but a real board that falls behind (too many
    // particles, a slow blit) would starve exactly the same way, and would do it precisely when
    // someone needs the console to find out why.
    if ((int32_t)(xTaskGetTickCount() - next) >= 0) {
      next = xTaskGetTickCount();   // give up on catching up rather than spinning
      ++g_overruns;
      vTaskDelay(1);                // one tick to anything below us
    } else {
      vTaskDelayUntil(&next, period);
    }
  }
}

// --- serial console ----------------------------------------------------------------------------

// -Wdouble-promotion is on for the whole firmware because a stray double in the solver silently
// halves the framerate on a chip that emulates them in software. printf is the one place it
// cannot be obeyed: varargs promote float to double by definition, so there is nothing to fix.
// Scoped to the console rather than disabled globally, so the warning keeps working where it
// matters.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"

void printHelp() {
  Serial.println(F("partsim console"));
  Serial.println(F("  ?            this help"));
  Serial.println(F("  s            list scenes"));
  Serial.println(F("  s <n>        switch to scene n (gradual, crossfaded)"));
  Serial.println(F("  c            toggle auto-cycle"));
  Serial.println(F("  b <0-255>    panel brightness"));
  Serial.println(F("  m            print the face mount table"));
  Serial.println(F("  m <f> <r> <x>  set face f to rotation r (0-3), mirror x (0/1)"));
  Serial.println(F("  t            show the orientation test pattern"));
  Serial.println(F("  i            IMU state"));
  Serial.println(F("  r            frame timing and memory"));
  Serial.println(F("  g            run the golden determinism sequence (blocks ~30s)"));
  Serial.println(F("  p            pause/resume the physics"));
}

void printMounts() {
  const ChainMap& cm = g_panels.chain();
  Serial.printf("chain %dx%d, %d faces\n", cm.chainWidth(), cm.chainHeight(), cm.count());
  for (int i = 0; i < cm.count(); ++i) {
    const FaceMount& m = cm.mount(i);
    const ChainRun r = cm.row(i, 0);
    Serial.printf("  face %d -> slot %u rot %u mirror %u  (row %s)\n", i, m.slot, m.rotate,
                  m.mirror, r.dy == 0 ? "horizontal" : "vertical, slower blit");
  }
}

void printImu() {
  if (!g_imu.present()) {
    Serial.printf("IMU absent (WHO_AM_I read 0x%02X, expected 0x6C)\n", g_imu.whoAmI());
    Serial.println(F("  gravity is held at the default -y, so the sim still runs level"));
    return;
  }
  const Vec3 d = g_motion.down();
  const Vec3 b = g_motion.gyroBias();
  const Vec3 a = g_motion.containerAccel();
  Serial.printf("down   % .3f % .3f % .3f  (object space, unit)\n", d.x, d.y, d.z);
  Serial.printf("accel  % .2f % .2f % .2f  (container, sim units)\n", a.x, a.y, a.z);
  Serial.printf("bias   % .5f % .5f % .5f rad/s\n", b.x, b.y, b.z);
  Serial.printf("trust  %.2f   seeded %d   dropped samples %u\n", g_motion.trust(),
                (int)g_motion.seeded(), (unsigned)g_dropped);
}

void printStats() {
  Serial.printf("fps %.1f   sim %.2f ms   splat %.2f ms   blit %.2f ms   substeps %d\n",
                g_stats.fps, g_stats.simMs, g_stats.renderMs, g_stats.blitMs, g_stats.substeps);
  Serial.printf("frames %u, overruns %u (%.1f%%)\n", (unsigned)g_stats.frames,
                (unsigned)g_overruns,
                g_stats.frames ? 100.0f * (float)g_overruns / (float)g_stats.frames : 0.0f);
  Serial.printf("particles %d/%d   scene %d (%s)%s\n", g_sim.particleCount(), kMaxParticles,
                g_sim.scene(), sceneAt(g_sim.scene()).name,
                g_sim.transitioning() ? "  [transitioning]" : "");
  Serial.printf("internal heap free %u B, largest block %u B\n",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
  Serial.printf("psram free %u B\n", (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

// The third target of the cross-target determinism check. The host and WASM builds already agree
// bit-for-bit; this is the same scripted sequence at the same capacities, so the number below
// must equal the contents of scripts/golden_hash_esp32.txt.
void runGolden() {
#ifdef PARTSIM_NONDETERMINISTIC_FP
  Serial.println(F("NOTE: built with -ffp-contract=fast, so this hash is EXPECTED to differ."));
  Serial.println(F("      Flash the `cube` environment to check determinism."));
#endif
  Serial.println(F("running golden sequence, this blocks the display for a while..."));
  const bool wasPaused = g_paused;
  g_paused = true;
  const uint32_t t0 = millis();
  const uint32_t hash = goldenHash(g_sim, kGoldenSteps, kGoldenSeed);
  const uint32_t dt = millis() - t0;
  Serial.printf("state  %08x   (%u steps in %u ms, %.2f ms/step)\n", hash, 2u * kGoldenSteps, dt,
                (float)dt / (float)(2 * kGoldenSteps));
  Serial.println(F("compare against scripts/golden_hash_esp32.txt (first field)"));
  // The sequence left the simulation in whatever state it ended in; put a scene back.
  g_sim.initScene(kFaces == 1 ? Simulation::kSinglePanel : Simulation::kCube, 0, 1,
                  kPanelRes);
  g_sim.setAutoCycle(true);
  g_paused = wasPaused;
}

void handleLine(char* line) {
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
      if (argc >= 2) {
        const int n = atoi(argv[1]);
        if (n >= 0 && n < sceneCount()) {
          g_sim.transitionToScene(n);
          Serial.printf("scene -> %d (%s)\n", n, sceneAt(n).name);
        } else {
          Serial.printf("scene out of range 0..%d\n", sceneCount() - 1);
        }
      } else {
        for (int i = 0; i < sceneCount(); ++i)
          Serial.printf("  %d %s%s\n", i, sceneAt(i).name, i == g_sim.scene() ? "  <--" : "");
      }
      break;

    case 'c':
      g_sim.setAutoCycle(!g_sim.autoCycle());
      Serial.printf("auto-cycle %s\n", g_sim.autoCycle() ? "on" : "off");
      break;

    case 'b':
      if (argc >= 2) {
        const int v = atoi(argv[1]);
        g_panels.setBrightness((uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v)));
        Serial.printf("brightness %d\n", v);
      }
      break;

    case 'm':
      if (argc >= 4) {
        const int f = atoi(argv[1]);
        FaceMount m{(uint8_t)f, (uint8_t)atoi(argv[2]), (uint8_t)atoi(argv[3])};
        if (f >= 0 && f < g_panels.chain().count()) m.slot = g_panels.chain().mount(f).slot;
        if (f < 0 || f >= g_panels.chain().count() || !g_panels.chain().setMount(f, m)) {
          Serial.println(F("rejected: that would not be a valid mount table"));
        }
      }
      printMounts();
      break;

    case 't':
      g_showTestPattern = true;
      Serial.println(F("test pattern: white dot at texel (1,1), red arm +x (3), green arm +y (5)"));
      Serial.println(F("adjust with `m <face> <rot> <mirror>` until every face reads the same"));
      break;

    case 'i':
      printImu();
      break;

    case 'r':
      printStats();
      break;

    case 'g':
      runGolden();
      break;

    case 'p':
      g_paused = !g_paused;
      Serial.printf("physics %s\n", g_paused ? "paused" : "running");
      break;

    default:
      Serial.println(F("unknown command; ? for help"));
      break;
  }
}

#pragma GCC diagnostic pop

}  // namespace

void setup() {
  Serial.begin(115200);
  // Long enough for the USB CDC to come up on a devkit, short enough not to matter when the
  // cube is running standalone off a power supply with nothing listening.
  const uint32_t deadline = millis() + 1500;
  while (!Serial && millis() < deadline) delay(10);

  Serial.println();
  Serial.println(F("partsim firmware"));
  Serial.printf("faces %d, %dx%d, %d-bit colour, target %d fps\n", kFaces, kPanelRes, kPanelRes,
                kColorDepthBits, kTargetFps);
#ifdef PARTSIM_NONDETERMINISTIC_FP
  Serial.println(F("build: cube-fast -- FMA contraction on, determinism check will NOT match"));
#endif

  // Radio off before anything else claims memory. WiFi and BT together cost ~55KB of internal
  // heap, add ISR jitter to a display clocked at 16MHz, and would be degraded by the panels'
  // own emissions in any case. Nothing here needs a network.
  WiFi.mode(WIFI_OFF);
  esp_wifi_deinit();
#if CONFIG_BT_ENABLED
  btStop();
#endif

  const int mode = (kFaces == 1) ? Simulation::kSinglePanel : Simulation::kCube;
  if (!g_sim.initScene(mode, 0, 1, kPanelRes)) {
    Serial.println(F("FATAL: simulation init failed -- capacities too small for this geometry"));
    for (;;) delay(1000);
  }
  g_sim.setAutoCycle(true);
  Serial.printf("simulation: %d particles, capacity %d\n", g_sim.particleCount(),
                g_sim.capacity());

#if PARTSIM_QEMU
  // No panels under emulation. QEMU models the CPU, RAM, flash, timers and UART, but not LCD_CAM
  // or GDMA -- and the HUB75 library spins waiting for a DMA completion that never arrives, which
  // starves setup() before it can even report the hang. Nothing about the display path is
  // testable here; everything above it is.
  Serial.println(F("QEMU build: panel driver skipped (no LCD_CAM/GDMA model)"));
#else
  if (!g_panels.begin(g_sim.geometry(), kColorDepthBits, kDefaultBrightness)) {
    Serial.println(F("FATAL: HUB75 init failed -- check Pins.h against the wiring"));
    for (;;) delay(1000);
  }
#endif
#if !PARTSIM_QEMU
  Serial.printf("panels: chain %dx%d, rows %s\n", g_panels.chain().chainWidth(),
                g_panels.chain().chainHeight(),
                g_panels.allRunsHorizontal(g_sim.geometry()) ? "all horizontal (fast blit)"
                                                             : "some vertical (slower blit)");
#endif

  MotionConfig mcfg = MotionConfig::defaults();
  // The axis map is the one thing that cannot be guessed: it depends on how the breakout is
  // glued in. Identity until the real object exists -- use the `i` command with the cube resting
  // on each face in turn to work out the permutation, then set it here.
  g_motion.init(mcfg, AxisMap::identity());

  if (!g_imu.begin(pins::kSda, pins::kScl, pins::kI2cHz)) {
    // Not fatal on purpose: a cube with a dead IMU should still be a lamp. Gravity stays at the
    // default -y and the fluid simply sits at the bottom.
    Serial.printf("WARNING: no LSM6DSOX (WHO_AM_I 0x%02X); running with fixed gravity\n",
                  g_imu.whoAmI());
  } else {
    Serial.println(F("IMU: LSM6DSOX at 208 Hz, +-8 g / +-500 dps"));
    xTaskCreatePinnedToCore(imuTask, "imu", 2560, nullptr, 3, nullptr, 0);
  }

#if PARTSIM_QEMU
  // Run it here, before simTask exists. The emulator cannot keep a 30fps cadence, so a running
  // simTask would fight the console for the whole session -- and the determinism sequence is the
  // only thing this environment is for.
  Serial.println(F("QEMU: running the determinism sequence"));
  runGolden();
  Serial.println(F("QEMU: done"));
#endif

  // 6KB of stack: the solver recurses nowhere and every pool is static, so this is generous.
  xTaskCreatePinnedToCore(simTask, "sim", 6144, nullptr, 2, nullptr, 1);

  Serial.printf("internal heap free after init: %u B\n",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  Serial.println(F("ready -- ? for help"));
}

void loop() {
  // The console. Runs as Arduino's loopTask on core 1 at priority 1, below simTask, so it only
  // gets time while the simulation is blocked in vTaskDelayUntil.
  static char buf[64];
  static size_t len = 0;

  while (Serial.available()) {
    const int c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (len) {
        buf[len] = '\0';
        handleLine(buf);
        len = 0;
      }
    } else if (len + 1 < sizeof(buf)) {
      buf[len++] = (char)c;
    }
  }
  delay(10);
}
