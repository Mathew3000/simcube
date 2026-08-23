#pragma once
#include <cstdint>

// Compile-time capacities. Defaults suit a host/WASM build; the ESP32 needs far smaller pools.
//
// The ESP32 numbers live HERE, selected by one -DPARTSIM_PROFILE_ESP32, rather than as a list of
// -DPARTSIM_MAX_* in platformio.ini. Spelling them out in the build file would mean the host
// verification and the firmware each carry their own copy of the budget, and the first time one
// was edited the checks would quietly start measuring a configuration nobody ships.
#ifdef PARTSIM_PROFILE_ESP32

// 1280 particles is a CPU limit, not a memory one. A bottom-up cycle count came to ~5500
// cycles/particle/step, which at 240MHz and 30 FPS is about 1300 -- so a larger pool would
// only buy RAM pressure in exchange for particles the processor cannot integrate anyway.
#define PARTSIM_DEFAULT_MAX_PARTICLES 1280
#define PARTSIM_DEFAULT_MAX_PANELS 6
#define PARTSIM_DEFAULT_MAX_PANEL_TEXELS 1024  // 32x32
// 32-unit cube at kCellSize 3.0, plus the grid's padding: 12^3 with room to spare.
#define PARTSIM_DEFAULT_MAX_GRID_CELLS 4096
// 32-unit cube at kFieldCell 1.5: 22^3 = 10648, and the grid is ping-ponged, so this is the
// single largest pool after the particles. Do not round it up generously.
#define PARTSIM_DEFAULT_MAX_FIELD_CELLS 10648
// No internal RGBA copy of every panel: the firmware resolves one face at a time into a single
// 3KB staging buffer and blits it. Six panels of RGBA is 24KB, which is the difference between
// fitting internal SRAM and not.
#define PARTSIM_DEFAULT_INTERNAL_PIXELS 0

#else

#define PARTSIM_DEFAULT_MAX_PARTICLES 16384
#define PARTSIM_DEFAULT_MAX_PANELS 8
#define PARTSIM_DEFAULT_MAX_PANEL_TEXELS 4096
#define PARTSIM_DEFAULT_MAX_GRID_CELLS 32768
#define PARTSIM_DEFAULT_MAX_FIELD_CELLS 32768
#define PARTSIM_DEFAULT_INTERNAL_PIXELS 1

#endif

#ifndef PARTSIM_MAX_PARTICLES
#define PARTSIM_MAX_PARTICLES PARTSIM_DEFAULT_MAX_PARTICLES
#endif
#ifndef PARTSIM_MAX_PANELS
#define PARTSIM_MAX_PANELS PARTSIM_DEFAULT_MAX_PANELS
#endif
#ifndef PARTSIM_MAX_PANEL_TEXELS
#define PARTSIM_MAX_PANEL_TEXELS PARTSIM_DEFAULT_MAX_PANEL_TEXELS
#endif
#ifndef PARTSIM_MAX_GRID_CELLS
#define PARTSIM_MAX_GRID_CELLS PARTSIM_DEFAULT_MAX_GRID_CELLS
#endif
#ifndef PARTSIM_MAX_FIELD_CELLS
#define PARTSIM_MAX_FIELD_CELLS PARTSIM_DEFAULT_MAX_FIELD_CELLS
#endif
#ifndef PARTSIM_INTERNAL_PIXELS
#define PARTSIM_INTERNAL_PIXELS PARTSIM_DEFAULT_INTERNAL_PIXELS
#endif

namespace partsim {

constexpr int kMaxParticles = PARTSIM_MAX_PARTICLES;
constexpr int kMaxPanels = PARTSIM_MAX_PANELS;
constexpr int kMaxPanelTexels = PARTSIM_MAX_PANEL_TEXELS;
constexpr int kMaxGridCells = PARTSIM_MAX_GRID_CELLS;
constexpr int kMaxFieldCells = PARTSIM_MAX_FIELD_CELLS;
constexpr int kMaxEmitters = 4;

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
// Heat reaches much further than fluid, and has to: a plume in the middle of a 32-unit cube is
// 16 units from every side face, so at the particle influence of 8 only the floor and ceiling
// could see a campfire at all. Fire is emissive and glows through the volume, so a reach that
// spans the box is both cheaper to justify and what actually looks right.
constexpr float kHeatInfluence = 24.0f;
// Radius of a particle's blob, in WORLD units -- not texels. A particle is a physical thing, so
// its apparent size must not change when the panel resolution does. The texel footprint is
// derived from this and the panel pitch in Renderer::init.
//
// 2.5 is exactly the old texel-space value (a footprint of 2 plus the half-texel to the kernel's
// zero crossing), so at pitch 1.0 this reproduces the previous behaviour bit-for-bit.
constexpr float kSplatRadiusWorld = 2.5f;
constexpr int kAttenLutSize = 64;
// Accumulated intensity that maps to the top of a colour ramp. Measured, not guessed: a dense
// water texel peaks around 6500 at these kernel constants, and setting this too low clips
// everything to white and throws the whole ramp away.
constexpr float kSplatExposure = 7200.0f;
enum Channel : uint8_t { kChWater = 0, kChSand = 1, kChHeat = 2, kChannelCount = 3 };
// Heat is a field, not particles, so its accumulated intensity needs its own scale to land in
// the same 0..kSplatExposure range the particle channels use.
constexpr float kHeatGain = 5200.0f;
// Field cells below this are not worth splatting; skipping them is most of the render cost of a
// mostly-cold volume.
constexpr uint8_t kHeatFloor = 6;

}  // namespace partsim
