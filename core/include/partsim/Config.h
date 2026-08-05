#pragma once
#include <cstdint>

// Compile-time capacities. Defaults suit a host/WASM build; the ESP32 build overrides them
// downward via platformio.ini build_flags so the static pools fit internal SRAM.
#ifndef PARTSIM_MAX_PARTICLES
#define PARTSIM_MAX_PARTICLES 16384
#endif
#ifndef PARTSIM_MAX_PANELS
#define PARTSIM_MAX_PANELS 8
#endif
#ifndef PARTSIM_MAX_PANEL_TEXELS
#define PARTSIM_MAX_PANEL_TEXELS 4096
#endif
#ifndef PARTSIM_MAX_GRID_CELLS
#define PARTSIM_MAX_GRID_CELLS 32768
#endif
#ifndef PARTSIM_MAX_FIELD_CELLS
#define PARTSIM_MAX_FIELD_CELLS 32768
#endif

namespace partsim {

constexpr int kMaxParticles = PARTSIM_MAX_PARTICLES;
constexpr int kMaxPanels = PARTSIM_MAX_PANELS;
constexpr int kMaxPanelTexels = PARTSIM_MAX_PANEL_TEXELS;
constexpr int kMaxGridCells = PARTSIM_MAX_GRID_CELLS;
constexpr int kMaxFieldCells = PARTSIM_MAX_FIELD_CELLS;

// Particle indices are stored as uint16 in the sort/index arrays.
static_assert(kMaxParticles <= 65535, "particle indices are uint16");

// --- world units -----------------------------------------------------------
// One world unit == one panel texel. A 32x32x32 cube volume is therefore 32 units on a
// side, which keeps every tuning constant below readable as "in pixels".
constexpr float kPitch = 1.0f;

// --- solver ----------------------------------------------------------------
constexpr float kRestSpacing = 1.5f;                  // d: particle rest separation
constexpr float kSmoothRadius = 2.0f * kRestSpacing;  // h: SPH kernel support (3.0)
constexpr float kCellSize = kSmoothRadius;            // sort cell; MUST be >= h

constexpr float kFixedDt = 1.0f / 60.0f;
constexpr int kSolverIterations = 3;
constexpr int kMaxSubsteps = 3;

constexpr float kGravityMag = 9.81f * 4.0f;  // scaled: world is 32 units, not 32 metres
constexpr float kCfmEpsilon = 300.0f;        // constraint-force-mixing relaxation
constexpr float kSCorrK = 1.0e-4f;           // Macklin artificial pressure
constexpr int kSCorrN = 4;
constexpr float kMaxDeltaP = 0.5f * kRestSpacing;  // per-iteration correction clamp
constexpr float kXsphC = 0.02f;                    // XSPH viscosity (water)
constexpr float kBoundaryStiffness = 1.0f;

// --- materials -------------------------------------------------------------
enum Material : uint8_t { kWater = 0, kSand = 1, kMaterialCount = 2 };

struct MaterialParams {
  float restDensityScale;  // relative to water; sand ~2.0 so it sinks via the constraint
  float xsph;              // XSPH viscosity coefficient
  float friction;          // tangential correction damping, 0 = frictionless
  float staticVelocity;    // below this speed, treat as at rest (stops pile creep)
};

// --- rendering -------------------------------------------------------------
constexpr float kSplatInfluence = 8.0f;  // depth beyond which a particle lights nothing
constexpr int kSplatFootprint = 2;       // kernel half-width in texels
constexpr int kAttenLutSize = 64;
enum Channel : uint8_t { kChWater = 0, kChSand = 1, kChHeat = 2, kChannelCount = 3 };

}  // namespace partsim
