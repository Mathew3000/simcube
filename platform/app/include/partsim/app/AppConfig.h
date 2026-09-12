#pragma once
#include <cstdint>

#include "partsim/Config.h"

namespace partsim {
namespace app {

// How many faces this build drives. One for bringing up a single panel before six exist; the
// simulation's slab mode is the same 3D code path, just a thin volume.
#ifdef PARTSIM_FACE_COUNT
constexpr int kFaces = PARTSIM_FACE_COUNT;
#else
constexpr int kFaces = 6;
#endif

// Panel resolution and colour depth come from core's capacity profile, not a local copy. There
// used to be a duplicate of the resolution here that was only used for the boot banner -- so it
// could not break anything, but it could print a lie, which is worse in a message someone reads
// during bring-up. kColourBits is from Config.h so the firmware and
// scripts/check_esp32_budget.sh cannot disagree about the DMA framebuffer size.
constexpr uint8_t kColourDepthBits = (uint8_t)kColourBits;
constexpr uint8_t kDefaultBrightness = 96;
constexpr int kTargetFps = 30;
constexpr float kImuDt = 1.0f / 208.0f;

// Which node layout this image is built for. A display node must not hold a Simulation -- 136.6KB
// against a 230KB budget -- so this is a compile-time fact, not a runtime one.
//
// The single-board `cube` environment keeps the Milestone 2 behaviour untouched: it is what the
// 32x32 panels will be brought up on, and there is no reason to make that path depend on code no
// hardware has exercised.
#if defined(PARTSIM_PROFILE_ESP32_MASTER) || defined(PARTSIM_PROFILE_ESP32_DISPLAY)
#define PARTSIM_MULTINODE 1
#else
#define PARTSIM_MULTINODE 0
#endif

}  // namespace app
}  // namespace partsim
