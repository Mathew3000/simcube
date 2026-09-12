// The firmware's application layer, running on this machine.
//
// This exists to make one claim checkable: that platform/app really is platform-neutral. A HAL
// that only ever has one implementation is not a HAL, it is a rename -- the coupling stays and
// nobody discovers it until somebody tries the port and finds the application logic still full of
// Serial and FreeRTOS. Compiling and RUNNING App against a second, completely different platform
// is the proof, and it costs the ~100 lines below.
//
// It is also useful on its own: `g` runs the golden determinism sequence through exactly the code
// the `g` console command runs on the device, so tests/CMakeLists.txt can check the application
// layer against scripts/golden_hash.txt without a board attached.
//
// Drive it the same way as the real console:
//     echo g | ./build/platform/host/partsim_console
//     ./build/platform/host/partsim_console          (interactive; ? for help)
#include <poll.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

#include "partsim/app/App.h"

using namespace partsim;
using namespace partsim::app;

namespace {

// stdout, and stdin assembled into lines exactly the way loop() assembles them off Serial: bytes
// arrive when they arrive and a line is only a line once its terminator does.
class StdioConsole final : public Console {
 public:
  void write(const char* s) override {
    std::fputs(s, stdout);
    std::fflush(stdout);  // the device's UART has no buffering to surprise anyone with either
  }

  int readLine(char* buf, int cap) override {
    // Non-blocking, because the interface says so and because this loop has a simulation to step.
    struct pollfd p = {STDIN_FILENO, POLLIN, 0};
    while (::poll(&p, 1, 0) > 0) {
      char ch;
      const ssize_t n = ::read(STDIN_FILENO, &ch, 1);
      if (n <= 0) { eof_ = true; return -1; }
      if (ch == '\r' || ch == '\n') {
        if (len_ == 0) continue;
        const int out = len_ < cap - 1 ? len_ : cap - 1;
        std::memcpy(buf, line_, (size_t)out);
        buf[out] = '\0';
        len_ = 0;
        return out;
      }
      if (len_ + 1 < (int)sizeof(line_)) line_[len_++] = ch;
    }
    return -1;
  }

  bool eof() const { return eof_; }

 private:
  char line_[64];
  int len_ = 0;
  bool eof_ = false;
};

class StdClock final : public Clock {
 public:
  uint32_t micros() const override { return (uint32_t)elapsed<std::micro>(); }
  uint32_t millis() const override { return (uint32_t)elapsed<std::milli>(); }
  void delayMs(uint32_t ms) override {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  }

 private:
  template <typename Ratio>
  long long elapsed() const {
    using namespace std::chrono;
    return duration_cast<duration<long long, Ratio>>(steady_clock::now() - t0_).count();
  }
  std::chrono::steady_clock::time_point t0_ = std::chrono::steady_clock::now();
};

// Nothing to suspend: the host runs the console and the step in one thread, so a command that
// rebuilds the Simulation cannot race a stepping task. This is the case SystemHooks' defaults
// describe, spelled out rather than inherited silently -- the ESP32 version of these two calls is
// load-bearing and it should be visible that skipping them here is a decision.
class HostHooks final : public SystemHooks {};

StdioConsole g_console;
StdClock g_clock;
NullDisplay g_display;
NullMotionSensor g_imu;
NullFrameLink g_link;
HostHooks g_hooks;

const Platform g_plat{&g_console, &g_clock, &g_display, &g_imu, &g_link, &g_hooks};
App g_app(g_plat);  // holds a whole Simulation, so global rather than on main's stack

}  // namespace

int main(int argc, char** argv) {
  const bool stepping = !((argc > 1) && std::strcmp(argv[1], "--no-step") == 0);

  std::printf("partsim console (host build of platform/app)\n");
  std::printf("faces %d, %dx%d, %d-bit colour, target %d fps\n", kFaces, kPanelRes, kPanelRes,
              kColourDepthBits, kTargetFps);
  if (!g_app.begin(Role::Master)) return 1;
  std::printf("ready -- ? for help\n");

  const auto period = std::chrono::milliseconds(1000 / kTargetFps);
  while (!g_console.eof()) {
    g_app.consolePoll();
    if (stepping) g_app.simStep();
    std::this_thread::sleep_for(period);
  }
  return 0;
}
