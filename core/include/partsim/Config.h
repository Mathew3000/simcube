#pragma once
#include <cstdint>

// Compile-time capacities. Defaults suit a host/WASM build; the ESP32 needs far smaller pools.
//
// The device numbers live HERE, selected by a single -DPARTSIM_PROFILE_* per role, rather than as
// a list of -DPARTSIM_MAX_* in platformio.ini. Spelling them out in the build file would mean the
// host verification and the firmware each carry their own copy of the budget, and the first time
// one was edited the checks would quietly start measuring a configuration nobody ships.
//
// Three device roles, because a 64x64 cube cannot be driven by one board:
//   PARTSIM_PROFILE_ESP32          one node, six 32x32 panels   (Milestone 2, still shipping)
//   PARTSIM_PROFILE_ESP32_MASTER   physics + IMU, no panels     (Milestone 3)
//   PARTSIM_PROFILE_ESP32_DISPLAY  two 64x64 faces of six       (Milestone 3, three of these)

// --- shared by every device profile ------------------------------------------------------------
// 1280 particles is a CPU limit, not a memory one. A bottom-up cycle count came to ~5500
// cycles/particle/step, which at 240MHz and 30 FPS is about 1300 -- so a larger pool would
// only buy RAM pressure in exchange for particles the processor cannot integrate anyway.
//
// This is the figure to revisit once the master stops splatting: on a multi-node cube the
// render cost moves to the display nodes, so the master's step budget grows.
#define PARTSIM_DEVICE_MAX_PARTICLES 1280
#define PARTSIM_DEVICE_MAX_PANELS 6
// 32-unit cube at kCellSize 3.0, plus the grid's padding: 12^3 with room to spare.
#define PARTSIM_DEVICE_MAX_GRID_CELLS 4096
// 32-unit cube at kFieldCell 1.5: 22^3 = 10648, and the grid is ping-ponged, so this is the
// single largest pool after the particles. Do not round it up generously. Note it is derived
// from the WORLD size, so it does not grow with panel resolution.
#define PARTSIM_DEVICE_MAX_FIELD_CELLS 10648

#ifdef PARTSIM_PROFILE_ESP32

// Single node driving all six 32x32 panels. This is the Milestone 2 configuration and it stays
// valid: it is what the 32x32 panels currently in transit will be brought up on.
#define PARTSIM_DEFAULT_MAX_PARTICLES PARTSIM_DEVICE_MAX_PARTICLES
#define PARTSIM_DEFAULT_MAX_PANELS PARTSIM_DEVICE_MAX_PANELS
#define PARTSIM_DEFAULT_MAX_PANEL_TEXELS 1024  // 32x32
#define PARTSIM_DEFAULT_PANEL_RES 32
#define PARTSIM_DEFAULT_MAX_RENDER_PANELS 6    // it drives all of them
#define PARTSIM_DEFAULT_DRIVES_PANELS 1
#define PARTSIM_DEFAULT_RUNS_SOLVER 1
#define PARTSIM_DEFAULT_MAX_GRID_CELLS PARTSIM_DEVICE_MAX_GRID_CELLS
#define PARTSIM_DEFAULT_MAX_FIELD_CELLS PARTSIM_DEVICE_MAX_FIELD_CELLS
// No internal RGBA copy of every panel: the firmware resolves one face at a time into a single
// staging buffer and blits it. Six panels of RGBA is 24KB, which is the difference between
// fitting internal SRAM and not.
#define PARTSIM_DEFAULT_INTERNAL_PIXELS 0

#elif defined(PARTSIM_PROFILE_ESP32_MASTER)

// Simulation master on a 64x64 cube: owns the IMU and the physics, drives no panels, and ships
// state to the display nodes. Panel texels are still 4096 because it holds the FULL six-panel
// geometry -- the container AABB has to be identical on every node, so the panel table is never
// trimmed even though nothing is rendered from most of it.
#define PARTSIM_DEFAULT_MAX_PARTICLES PARTSIM_DEVICE_MAX_PARTICLES
#define PARTSIM_DEFAULT_MAX_PANELS PARTSIM_DEVICE_MAX_PANELS
#define PARTSIM_DEFAULT_MAX_PANEL_TEXELS 4096  // 64x64
#define PARTSIM_DEFAULT_PANEL_RES 64
// One render slot, not zero: enough for a diagnostic face, and the master has the memory spare.
#define PARTSIM_DEFAULT_MAX_RENDER_PANELS 1
#define PARTSIM_DEFAULT_MAX_GRID_CELLS PARTSIM_DEVICE_MAX_GRID_CELLS
#define PARTSIM_DEFAULT_RUNS_SOLVER 1
#define PARTSIM_DEFAULT_MAX_FIELD_CELLS PARTSIM_DEVICE_MAX_FIELD_CELLS
#define PARTSIM_DEFAULT_INTERNAL_PIXELS 0
// No HUB75 connector at all. Stated rather than implied by the render slot count, because the
// memory report was silently charging the master 48KB of DMA buffer for a chain it does not have.
#define PARTSIM_DEFAULT_DRIVES_PANELS 0

#elif defined(PARTSIM_PROFILE_ESP32_DISPLAY)

// Display node on a 64x64 cube: two faces of the six. Three of these plus one master cover the
// cube. Two is not a preference -- three faces would need 144KB of DMA plus 73.8KB of
// accumulation, which does not fit.
#define PARTSIM_DEFAULT_MAX_PARTICLES PARTSIM_DEVICE_MAX_PARTICLES
#define PARTSIM_DEFAULT_MAX_PANELS PARTSIM_DEVICE_MAX_PANELS
#define PARTSIM_DEFAULT_MAX_PANEL_TEXELS 4096  // 64x64
#define PARTSIM_DEFAULT_PANEL_RES 64
#define PARTSIM_DEFAULT_MAX_RENDER_PANELS 2
// A display node receives state and draws it -- it never integrates. So it carries RenderState's
// draw-only containers instead of a Simulation, which is what brings it inside the SRAM budget.
#define PARTSIM_DEFAULT_RUNS_SOLVER 0
#define PARTSIM_DEFAULT_MAX_GRID_CELLS PARTSIM_DEVICE_MAX_GRID_CELLS
#define PARTSIM_DEFAULT_MAX_FIELD_CELLS PARTSIM_DEVICE_MAX_FIELD_CELLS
#define PARTSIM_DEFAULT_INTERNAL_PIXELS 0
#define PARTSIM_DEFAULT_DRIVES_PANELS 1

#else

#define PARTSIM_DEFAULT_MAX_PARTICLES 16384
#define PARTSIM_DEFAULT_MAX_PANELS 8
#define PARTSIM_DEFAULT_MAX_PANEL_TEXELS 4096
#define PARTSIM_DEFAULT_PANEL_RES 32
#define PARTSIM_DEFAULT_MAX_RENDER_PANELS 8
#define PARTSIM_DEFAULT_DRIVES_PANELS 0
#define PARTSIM_DEFAULT_RUNS_SOLVER 1
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
#ifndef PARTSIM_MAX_RENDER_PANELS
#define PARTSIM_MAX_RENDER_PANELS PARTSIM_DEFAULT_MAX_RENDER_PANELS
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
#ifndef PARTSIM_DRIVES_PANELS
#define PARTSIM_DRIVES_PANELS PARTSIM_DEFAULT_DRIVES_PANELS
#endif
#ifndef PARTSIM_RUNS_SOLVER
#define PARTSIM_RUNS_SOLVER PARTSIM_DEFAULT_RUNS_SOLVER
#endif
#ifndef PARTSIM_PANEL_RES
#define PARTSIM_PANEL_RES PARTSIM_DEFAULT_PANEL_RES
#endif

namespace partsim {

constexpr int kMaxParticles = PARTSIM_MAX_PARTICLES;
constexpr int kMaxPanels = PARTSIM_MAX_PANELS;
constexpr int kMaxPanelTexels = PARTSIM_MAX_PANEL_TEXELS;
// How many panels this process produces pixels for, which is NOT the same as how many panels
// exist. On a multi-node cube every node holds the full panel table -- Geometry::bounds() must
// yield an identical container on all of them -- while allocating accumulation buffers for only
// the faces it physically drives. At 64x64 an accumulator is 24.6KB, so the difference between
// 6 and 2 is 98KB on a node with 230KB to spend.
constexpr int kMaxRenderPanels = PARTSIM_MAX_RENDER_PANELS;
static_assert(kMaxRenderPanels >= 1, "at least one render slot; a renderless node still needs the array");
static_assert(kMaxRenderPanels <= kMaxPanels, "cannot render more panels than exist");
constexpr int kMaxGridCells = PARTSIM_MAX_GRID_CELLS;
constexpr int kMaxFieldCells = PARTSIM_MAX_FIELD_CELLS;
constexpr int kMaxEmitters = 4;

// Particle indices are stored as uint16 in the sort/index arrays.
static_assert(kMaxParticles <= 65535, "particle indices are uint16");

// --- world units -----------------------------------------------------------
// The simulated volume is ALWAYS this many world units on a side, whatever the panel resolution.
// Every solver constant below is tuned against it -- the CFL argument for kGravityMag, the field
// grid's cell count, the rest spacing -- so it is an invariant, not a preference. A 64-unit world
// was measured and rejected: its heat field alone is 155KB.
constexpr float kWorldSize = 32.0f;

// Panel resolution is a DISPLAY choice. It changes how finely the volume is sampled and nothing
// else: tests/test_resolution.cpp asserts the state hash is bit-identical at 32 and 64.
//
// The pitch is DERIVED from it rather than passed alongside, so res 64 at pitch 1.0 -- a 64-unit
// world -- cannot be requested by accident.
constexpr int kPanelRes = PARTSIM_PANEL_RES;
constexpr float pitchFor(int res) { return kWorldSize / (float)res; }
constexpr float kPitch = kWorldSize / (float)kPanelRes;

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
