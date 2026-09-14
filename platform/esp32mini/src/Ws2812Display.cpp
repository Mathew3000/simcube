#include "Ws2812Display.h"

#include "Pins.h"
#include "partsim/Calibration.h"

using namespace partsim;

// Serpentine vs progressive (MINI.md section 0 and M2): WS2812B matrices are usually wired
// boustrophedon (row 0 left-to-right, row 1 right-to-left), but the user's recollection is that
// these are PROGRESSIVE -- every row starts at the same edge, the less common wiring and
// therefore the one worth confirming rather than assuming. Default to progressive per the brief,
// kept a compile-time flag rather than baked in, and settle it for real with the walk (`w`,
// MINI.md 6.3) before trusting this default: at 8 pixels a row, a wrong guess reads as noise, not
// as an obviously-wrong pattern.
#ifndef PARTSIM_MINI_SERPENTINE
#define PARTSIM_MINI_SERPENTINE 0
#endif

// The average-picture-level limiter (MINI.md M4), mandatory rather than a preference: 384 WS2812B
// at full white is ~21 A against a 3 A supply.
//
//   full white, per LED (3 channels, ~18.3 mA each)     ~55 mA
//   all 384 at full white                               ~21.1 A
//   quiescent, LEDs "off"  (~0.9 mA each)                ~0.35 A  -- just to be powered
//   ESP32 + WiFi peaks + margin, reserved                ~0.5 A
//   left for lit output                                  ~2.1 A  -> ~10% average picture level
//
// 2000 of that ~2.1 A, conservatively, until it is checked against a meter (MINI.md section 6.4).
// Raising it when a bigger supply is in hand is then a one-line, informed change.
//
// FastLED.setMaxPowerInVoltsAndMilliamps() IS this limiter already written: it sums accumulated
// current the same way (proportional to r+g+b per LED) and scales the whole frame down by the
// ratio if the budget would be exceeded, inside every show(). Hand-rolling a second copy of that
// arithmetic would just be a second place for the two to drift.
constexpr uint32_t kPowerBudgetMa = 2000;

bool Ws2812Display::begin(const Geometry& g) {
  const int count = g.count();
  if (count != kNumFaces) return false;
  if ((int)g.at(0).w != kMatrixSize || (int)g.at(0).h != kMatrixSize) return false;

  // Straight-through mounts to start with, exactly PanelDriver::begin's reasoning: nothing else
  // is knowable before the object exists, and the `m` console command is how the real
  // arrangement gets recorded.
  ChainMap::defaultMounts(count, mounts_);
  if (!chain_.init(g, mounts_, count)) return false;

  FastLED.addLeds<WS2812B, pins::kData, GRB>(leds_, kNumLeds);
  // Trap #1 (MINI.md section 7): the palette resolve already decided the colour: FastLED's own
  // colour-correction and temperature tables must not second-guess it.
  FastLED.setCorrection(UncorrectedColor);
  FastLED.setTemperature(UncorrectedTemperature);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, kPowerBudgetMa);

  // The first frame after boot must be dark (trap #10): 384 LEDs powering up mid-inrush at
  // whatever was last in their registers is how a 3A supply trips.
  FastLED.clear(true);
  begun_ = true;
  return true;
}

int Ws2812Display::chainPixelToStripIndex(int cx, int cy) const {
  const int slot = cx / kMatrixSize;   // which physical matrix, in wiring order (FaceMount.slot)
  int lx = cx % kMatrixSize;
  const int ly = cy;
#if PARTSIM_MINI_SERPENTINE
  if (ly & 1) lx = kMatrixSize - 1 - lx;
#endif
  return slot * (kMatrixSize * kMatrixSize) + ly * kMatrixSize + lx;
}

void Ws2812Display::blitFace(int face, int w, int h, const uint8_t* rgb) {
  for (int j = 0; j < h; ++j) {
    for (int i = 0; i < w; ++i) {
      int cx = 0, cy = 0;
      chain_.map(face, i, j, cx, cy);
      const uint8_t* px = rgb + (size_t)(j * w + i) * 3u;
      leds_[chainPixelToStripIndex(cx, cy)] = CRGB(px[0], px[1], px[2]);
    }
  }
}

void Ws2812Display::present(const Renderer& r, const Geometry& g) {
  if (!begun_) return;
  for (int f = 0; f < chain_.count(); ++f) {
    const int panel = chain_.panelAt(f);
    const Panel& pan = g.at(panel);
    // Tight RGB, not RGBA -- the strip has nothing to do with the alpha byte.
    r.resolve(panel, staging_, 3);
    blitFace(f, (int)pan.w, (int)pan.h, staging_);
  }
  FastLED.show();
}

void Ws2812Display::testPattern(const Geometry& g) {
  if (!begun_) return;
  for (int f = 0; f < chain_.count(); ++f) {
    const int panel = chain_.panelAt(f);
    const Panel& pan = g.at(panel);
    const int w = (int)pan.w, h = (int)pan.h;
    for (int j = 0; j < h; ++j) {
      for (int i = 0; i < w; ++i) {
        const Rgb8 c = calibrationTexel(g, panel, i, j);
        staging_[(size_t)(j * w + i) * 3 + 0] = c.r;
        staging_[(size_t)(j * w + i) * 3 + 1] = c.g;
        staging_[(size_t)(j * w + i) * 3 + 2] = c.b;
      }
    }
    blitFace(f, w, h, staging_);
  }
  FastLED.show();
}

void Ws2812Display::setBrightness(uint8_t b) { FastLED.setBrightness(b); }

bool Ws2812Display::allRunsHorizontal(const Geometry&) const {
  for (int f = 0; f < chain_.count(); ++f) {
    if (chain_.row(f, 0).dy != 0) return false;
  }
  return true;
}

bool Ws2812Display::lightOne(int index) {
  if (!begun_) return false;
  FastLED.clear();
  if (index < 0) {
    FastLED.show();
    return true;
  }
  if (index >= kNumLeds) {
    FastLED.show();
    return false;
  }
  leds_[index] = CRGB::White;
  FastLED.show();
  return true;
}
