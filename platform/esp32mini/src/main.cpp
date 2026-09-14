// partsim firmware for the plain-ESP32 WS2812B bring-up cube. See MINI.md.
//
// Deliberately smaller than platform/esp32/src/main.cpp: single node, no multinode roles, no
// ESP-NOW chain and no IMU wired yet (MINI.md descopes those from the port -- see the commit
// message). What is left is exactly what the brief asked for: wire up the concrete drivers and
// decide when a frame runs; everything the firmware actually DOES is partsim::app::App, shared
// with the S3 firmware and the host build.
//
// WiFi IS on here, in AP mode, for the bring-up web UI (brightness, dye, OTA upload) -- this
// board has no auto-reset circuit, so the alternative to OTA is a BOOT-button jumper every time.
// That does not reopen the ESP-NOW question: the chain (M7) still needs nobody to talk ESP-NOW to
// it, and never having wired that up is unrelated to whether the radio itself is powered.
//
// Task layout is the single-node half of the S3's: one stepping task, pinned, blocking in
// vTaskDelayUntil every frame so the console and the web server (both on Arduino's loopTask) get
// time at all.
#include <Arduino.h>
#include <Preferences.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <cstring>
#include <esp_heap_caps.h>

#include "Pins.h"
#include "WebUi.h"
#include "Ws2812Display.h"
#include "partsim/app/App.h"

using namespace partsim;
using namespace partsim::app;

namespace {

// --- the platform, which is all this file is ----------------------------------------------------

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

// NVS namespace and key for the mount table. 6 faces * 3 bytes -- comfortably inside a page.
constexpr char kPrefsNamespace[] = "partsim";
constexpr char kMountsKey[] = "mounts";

class Esp32Hooks final : public SystemHooks {
 public:
  uint32_t freeHeap() const override {
    return (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  }
  uint32_t largestHeapBlock() const override {
    return (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  }

  // See partsim::app::SystemHooks::suspendSim: load-bearing, not tidiness -- a console command
  // that rebuilds the Simulation must stop the task reading from it, and pausing alone is not
  // enough because accumulate() runs every frame regardless of pause state.
  void suspendSim() override { if (g_stepTask) vTaskSuspend(g_stepTask); }
  void resumeSim() override { if (g_stepTask) vTaskResume(g_stepTask); }

  // The user's ask: correct a face over `m` (or the web UI, once it grows a mount control) and it
  // survives a power cycle, rather than needing retyping every time -- MINI.md's own brief called
  // NVS persistence out of scope for the port; this is the user amending that scope directly.
  void mountsChanged(const ChainMap& cm) override {
    FaceMount m[kMaxPanels];
    const int n = cm.count();
    for (int i = 0; i < n; ++i) m[i] = cm.mount(i);
    Preferences prefs;
    if (!prefs.begin(kPrefsNamespace, false)) return;
    prefs.putBytes(kMountsKey, m, (size_t)n * sizeof(FaceMount));
    prefs.end();
  }

  static TaskHandle_t g_stepTask;
};
TaskHandle_t Esp32Hooks::g_stepTask = nullptr;

// Loads a previously-saved mount table. False if none is saved or the size does not match the
// CURRENT face count -- a stale save from a different geometry must not be applied blindly.
bool loadSavedMounts(FaceMount* out, int count) {
  Preferences prefs;
  if (!prefs.begin(kPrefsNamespace, true)) return false;
  const size_t want = (size_t)count * sizeof(FaceMount);
  const size_t got = prefs.getBytes(kMountsKey, out, want);
  prefs.end();
  return got == want;
}

SerialConsole g_console;
ArduinoClock g_clock;
Esp32Hooks g_hooks;

// Safe to hand over unconditionally: every entry point is guarded on begun_, so a build that
// never calls begin() gets a driver that does nothing and reports ready() == false.
Ws2812Display g_display;

// No IMU exists on this cube yet (MINI.md section 2). App::simStep's canned-motion path is
// exactly the seam this is for: MotionSensor::present() == false is the only branch it needs.
// When the part arrives, platform/esp32/src/Lsm6dsox.h is the model -- it is plain I2C over
// Arduino's Wire, not S3-specific, and MotionSource (core/) already does the fusion and the axis
// permutation, so nothing here should reimplement any of that.
NullMotionSensor g_imu;
NullFrameLink g_nullLink;

Platform g_plat{.console = &g_console,
                .clock = &g_clock,
                .display = &g_display,
                .imu = &g_imu,
                .link = &g_nullLink,
                .chain = &partsim::nullSpillTransport(),
                .hooks = &g_hooks};
App g_app(g_plat);  // static pools, so global rather than anywhere near a stack

// --- the web UI -----------------------------------------------------------------------------------
// Bring-up convenience, not a network feature: an access point rather than joining a router, so
// there is nothing to configure to reach it from a phone standing next to the cube.
constexpr const char* kApSsid = "partsim-mini";
constexpr const char* kApPassword = "partsim123";  // 8+ chars: WPA2's own minimum

WebServer g_server(80);

void handleRoot() { g_server.send_P(200, "text/html", kWebUiHtml); }

// Brightness and dye both turn into the exact console line a human would type and hand it to
// App::submitCommand -- one implementation of what either command means, not two.
void handleSet() {
  char cmd[64];
  if (g_server.hasArg("b")) {
    snprintf(cmd, sizeof(cmd), "b %s", g_server.arg("b").c_str());
    g_app.submitCommand(cmd);
    g_server.send(200, "text/plain", "brightness set");
    return;
  }
  if (g_server.hasArg("dye")) {
    const String v = g_server.arg("dye");  // "R,G"
    const int comma = v.indexOf(',');
    if (comma > 0) {
      snprintf(cmd, sizeof(cmd), "d %s %s", v.substring(0, comma).c_str(),
               v.substring(comma + 1).c_str());
      g_app.submitCommand(cmd);
      g_server.send(200, "text/plain", "dye set");
      return;
    }
  }
  g_server.send(400, "text/plain", "bad request");
}

// OTA over a plain file upload, the standard ESP32 Update.h pattern. The sim task is suspended
// for the duration: it runs at a higher priority than the web server's task and would otherwise
// starve a synchronous WebServer's chunk-by-chunk handling of the upload.
void handleUpdateUpload() {
  HTTPUpload& upload = g_server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    g_console.printf("OTA: receiving %s\n", upload.filename.c_str());
    g_hooks.suspendSim();
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
      Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      g_console.printf("OTA: %u bytes written, rebooting\n", (unsigned)upload.totalSize);
    } else {
      Update.printError(Serial);
      g_hooks.resumeSim();
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.end();
    g_hooks.resumeSim();
  }
}

void handleUpdateDone() {
  const bool ok = !Update.hasError();
  g_server.sendHeader("Connection", "close");
  g_server.send(200, "text/plain", ok ? "ok, rebooting" : "update failed, see the serial log");
  if (ok) {
    delay(200);
    ESP.restart();
  }
}

// --- scheduling ---------------------------------------------------------------------------------

constexpr TickType_t kFramePeriod = pdMS_TO_TICKS(1000 / kTargetFps);

// See platform/esp32/src/main.cpp's frameYield for why this exists: vTaskDelayUntil returns
// immediately once a deadline has passed, so a task that cannot hit its period stops yielding
// entirely and the console never runs again -- indistinguishable from a hang unless this is here.
void frameYield(TickType_t& next) {
  if ((int32_t)(xTaskGetTickCount() - next) >= 0) {
    next = xTaskGetTickCount();
    g_app.noteOverrun();
    vTaskDelay(1);
  } else {
    vTaskDelayUntil(&next, kFramePeriod);
  }
}

void simTask(void*) {
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    g_app.simStep();
    frameYield(next);
  }
}

[[noreturn]] void fatal() {
  for (;;) delay(1000);
}

}  // namespace

// --- bring-up -----------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  const uint32_t deadline = millis() + 1500;
  while (!Serial && millis() < deadline) delay(10);

  g_console.println();
  g_console.println("partsim firmware (mini)");
  g_console.printf("faces %d, %dx%d, %d-bit colour, target %d fps\n", kFaces, kPanelRes, kPanelRes,
                   kColourDepthBits, kTargetFps);

  if (!g_app.begin(Role::Master)) fatal();

  if (!g_display.begin(g_app.geometry())) {
    g_console.println("FATAL: WS2812 init failed -- check Pins.h against the wiring");
    fatal();
  }
  // Restore a mount table saved by a previous session, if one exists and matches this geometry's
  // face count. setAllMounts is the atomic whole-table form: applying a saved permutation one
  // face at a time would collide on shared slots the same way a live swap does (ChainMap.h).
  {
    FaceMount saved[kMaxPanels];
    const int n = g_display.chain().count();
    if (loadSavedMounts(saved, n) && g_display.chain().setAllMounts(saved, n)) {
      g_console.println("mounts: restored from flash");
    }
  }
  g_console.printf("panels: chain %dx%d, rows %s\n", g_display.chain().chainWidth(),
                   g_display.chain().chainHeight(),
                   g_display.allRunsHorizontal(g_app.geometry()) ? "all horizontal"
                                                                 : "some vertical");

  // No axis map to set: there is no IMU to glue in yet (MINI.md section 2). initMotion still
  // needs calling -- MotionSource wants a config even though it will never be seeded -- so the
  // canned-motion path in App::simStep has somewhere consistent to read kGravityMag's default
  // from.
  g_app.initMotion(MotionConfig::defaults(), AxisMap::identity());

  xTaskCreatePinnedToCore(simTask, "sim", 6144, nullptr, 2, &Esp32Hooks::g_stepTask, 1);

  // --- web UI: brightness, dye, OTA -- see the file comment for why WiFi is on at all. ---
  WiFi.mode(WIFI_AP);
  WiFi.softAP(kApSsid, kApPassword);
  g_console.printf("wifi: AP \"%s\", connect and open http://%s/\n", kApSsid,
                   WiFi.softAPIP().toString().c_str());
  g_server.on("/", HTTP_GET, handleRoot);
  g_server.on("/set", HTTP_GET, handleSet);
  g_server.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  g_server.begin();

  g_console.printf("internal heap free after init: %u B\n", (unsigned)g_hooks.freeHeap());
  g_console.println("ready -- ? for help");
}

void loop() {
  // The console and the web server. Both run as Arduino's loopTask on core 1 at priority 1, below
  // the stepping task, so they only get time while the simulation is blocked in vTaskDelayUntil --
  // except during an OTA upload, which explicitly suspends the stepping task (handleUpdateUpload).
  g_app.consolePoll();
  g_server.handleClient();
  delay(10);
}
