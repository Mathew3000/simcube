#include "partsim/WaterEffects.h"
#include <chrono>
#include <cstdio>

using namespace partsim;
using Clock = std::chrono::steady_clock;
namespace {
uint8_t pixels[64*64*4];
volatile unsigned sink = 0;
double milliseconds(Clock::time_point begin) {
  return std::chrono::duration<double, std::milli>(Clock::now()-begin).count();
}
}

int main() {
  std::printf("WaterEffects state: %zu bytes (excludes caller-owned pixel buffers)\n",
              sizeof(WaterEffects));
  std::puts("Host timings only; these are NOT ESP32 predictions.");
  const int resolutions[] = {8,32,64};
  for (int res : resolutions) {
    if (res*res > kMaxPanelTexels) continue;
    const Geometry g = Geometry::cube(res, 32.0f/(float)res);
    WaterEffects water;
    water.reset(0.55f, {0.9f,0});
    water.addInk({0,1});
    constexpr int steps = 600;
    auto start = Clock::now();
    for (int i = 0; i < steps; ++i) {
      const float t = (float)i*0.01f;
      water.step(1.0f/120.0f, {fsin(t), -fcos(t), 0.3f});
    }
    const double sim = milliseconds(start)/(double)steps;
    constexpr int frames = 300;
    start = Clock::now();
    for (int i = 0; i < frames; ++i) for (int face = 0; face < 6; ++face) {
      water.renderPanel(g.at(face), 32.0f, pixels);
      sink = sink + pixels[(i%(res*res))*4+2];
    }
    const double render = milliseconds(start)/(double)frames;
    std::printf("%2dx%2d: update %.4f ms; six-face shade %.4f ms; RGBA face scratch %d bytes\n",
                res, res, sim, render, res*res*4);
  }
}
