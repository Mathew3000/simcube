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

// Depth given to a flat panel so it has a volume to simulate in. Must be ~1.5h: at only 2
// units the depth cannot fit two particle layers at rest spacing, and the frustration shows
// up as a slab that never settles (measured: 400 particles churn at depth 2 or 3, settle at
// 4.5). Irrelevant for a closed cube, whose quads already span the box.
constexpr float kSlabDepth = 3.0f * kRestSpacing;

constexpr float kFixedDt = 1.0f / 60.0f;
// 2 beats 3 measurably: it settles faster (mean speed 0.03 vs 0.30 after 500 steps at
// 3000 particles) and costs 28% less. More iterations are not better here.
constexpr int kSolverIterations = 2;
constexpr int kMaxSubsteps = 3;

// Gravity is tuned for stability at 60Hz rather than physical scale: falling the full
// 32-unit height must not exceed the CFL velocity cap of 0.4*h/dt = 72 units/s.
// sqrt(2*g*32) = 59 units/s at g = 55, comfortably inside it.
constexpr float kGravityMag = 55.0f;
// Relaxation in the lambda denominator. Sized against the actual sum-of-squared-gradients,
// which is ~0.7 for a saturated neighbourhood at these kernel constants.
constexpr float kCfmEpsilon = 0.05f;
constexpr float kSCorrK = 5.0e-3f;           // Macklin artificial pressure
constexpr int kSCorrN = 4;
constexpr float kMaxDeltaP = 0.5f * kRestSpacing;  // per-iteration correction clamp
constexpr float kXsphC = 0.02f;                    // XSPH viscosity (water)
// Per-step velocity retention, bleeding off the positional residual that a finite iteration
// count leaves behind. Only a light touch is needed -- the fluid settles even at 1.0 once the
// wall density term is correct (see Solver::init); this just takes the last shimmer off.
constexpr float kVelocityDamping = 0.98f;
// Below this speed a particle is treated as at rest, so piles stop creeping.
constexpr float kSleepSpeed = 0.25f;

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
// Accumulated intensity that maps to the top of a colour ramp. Measured, not guessed: a dense
// water texel peaks around 6500 at these kernel constants, and setting this too low clips
// everything to white and throws the whole ramp away.
constexpr float kSplatExposure = 7200.0f;
enum Channel : uint8_t { kChWater = 0, kChSand = 1, kChHeat = 2, kChannelCount = 3 };

}  // namespace partsim
