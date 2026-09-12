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
// A CPU limit, not a memory one -- and MEASURED on hardware, which is the only reason this
// number is trustworthy. The 1280 that stood here came from a bottom-up estimate of ~5500
// cycles/particle/step; the device needs ~51900 (docs/RESOURCES.md section 5.1), and ~211
// particles fit 30 FPS at the old rest spacing. Solver cost scales n^1.77.
//
// 512 is sized against the COARSENED fluid: at kRestSpacing 3.0 the fullest preset (water tank)
// asks for 375 particles, and applySceneTargets caps a cube at 9/10 of capacity, so 512 leaves
// room for a preset to grow without another round of budget work. Raising it does not buy
// particles the processor can integrate.
#define PARTSIM_DEVICE_MAX_PARTICLES 512
#define PARTSIM_DEVICE_MAX_PANELS 6
// The sort cell is kSmoothRadius, which is 2*kRestSpacing, so this shrinks as the particles
// coarsen: a 32-unit cube at kCellSize 6.0 is ceil(32/6) = 6 per axis, 216 cells. 512 covers that
// with room for a slab's different aspect ratio. Was 4096, sized for kCellSize 3.0.
#define PARTSIM_DEVICE_MAX_GRID_CELLS 512
// The heat cell is kSmoothRadius/2 = kRestSpacing (core/src/RenderState.cpp), so this also
// shrinks with the coarsening: a 32-unit cube at cell 3.0 is ceil(32/3) = 11 per axis, 1331 cells.
// The grid is ping-ponged, so it is still the largest pool after the particles -- 1728 (12^3) is
// deliberately tight. Was 10648, sized for a 1.5-unit cell. Derived from the WORLD size, so it
// does not grow with panel resolution.
#define PARTSIM_DEVICE_MAX_FIELD_CELLS 1728

// --- capability tiers ------------------------------------------------------
// A tier is a NAMED point on the ladder, not a free combination of the switches below. Nineteen
// independent knobs is half a million configurations and none of them are tested; a combination
// nobody built will break silently, and there will be no golden hash to catch it. So the tiers are
// enumerated here, each one buildable and verifiable, and anything else is a deliberate override.
//
//   lite    32x32 x6, water only, d=4.0, 4-bit   one S3 with room to spare
//   cube    32x32 x6, everything, d=3.0, 6-bit   what ships today (the plain ESP32 profile)
//   beaker  64x64,    water only, d=2.5, 6-bit   liquid-only, so 30Hz physics is available
//   future  64x64,    everything, d=1.0          host-verified only; no MCU runs it
//
// Each sets only what it means to change; everything else falls through to the defaults below.
//
// A tier says WHAT to simulate and how well. It deliberately does not pick a target profile -- the
// capacities and the panel resolution come from PARTSIM_PROFILE_*, and a PlatformIO environment
// sets both. Folding the profile in here made a host test build think it was firmware and lose
// its RGBA buffers.

#ifdef PARTSIM_TIER_LITE
#define PARTSIM_ENABLE_SAND 0
#define PARTSIM_ENABLE_HEAT 0
#define PARTSIM_REST_SPACING 4.0f
#define PARTSIM_COLOUR_BITS 4
#endif

#ifdef PARTSIM_TIER_BEAKER
#define PARTSIM_ENABLE_SAND 0
#define PARTSIM_ENABLE_HEAT 0
// Dye is the point of this tier, not an option on it: a beaker whose liquid has no colour cannot
// mix, and the orientation gate cannot draw a RED arrow -- with sand and heat compiled out there is
// exactly one accumulation channel and one ramp, so every overlay texel resolves to the same
// colour and the arrows come out white. Measured by M4-C, which asserted both cases rather than
// noting the limitation.
#define PARTSIM_ENABLE_CHROMA 1
#define PARTSIM_REST_SPACING 2.5f
// 60 Hz, NOT the 30 Hz this tier was first written with.
//
// Halving the rate was expected to be available here: it is worth 2.08x and it was rejected for
// the shipping build only because it collapses a sand heap, and there is no sand in a beaker. It
// fails for a second, independent reason at this spacing. Measured, same fixture, 2500 steps:
//
//   d=2.5 @ 60Hz   mean|v| 0.031   12/1310 still moving   settles
//   d=2.5 @ 30Hz   mean|v| 1.011  524/1310 still moving   does not
//   d=4.0 @ 60Hz   mean|v| 0.000    0/384  still moving   settles
//
// Finer particles need more, not fewer, steps to shed momentum, and at 30 Hz this pool is still
// visibly agitated after 5000 steps (0.616). The rate lever does not survive the spacing that
// beaker mode's colour resolution requires.
#endif

#ifdef PARTSIM_TIER_FUTURE
// Deliberately beyond any MCU measured. The host build is where a configuration gets validated
// before the silicon to run it exists; this is the rung that keeps the ladder honest.
//
// BUILD-AND-RUN ONLY. It builds, runs and conserves particles, and it does not pass the physics
// fixtures, because those assert that a pool has reached rest within a step budget and a fine
// fluid needs far more than a linear extension of one: at spacing 1.0 the hydrostatic fixture
// still reads mean|v| 1.69 after 7500 steps where spacing 3.0 reaches 0.05 in 2500. That is a
// property of the fluid, not a fault -- but until a tier this fine is worth the minutes of CPU to
// verify, treat a green run here as "it did not crash" and nothing more.
#define PARTSIM_REST_SPACING 1.0f
#define PARTSIM_MAX_PARTICLES 24000
#define PARTSIM_MAX_GRID_CELLS 8192
#define PARTSIM_MAX_FIELD_CELLS 40000
#endif

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

// Particle indices widen with the pool rather than capping it.
//
// The point is that the capability ladder stays open at the top. A pool of 65535 is far beyond any
// MCU measured here -- an eighth of that is already seconds per step on a 1 GHz Cortex-M7 -- but
// the ceiling should be a property of the hardware, not of a type chosen years earlier. The host
// build is where a future configuration gets validated before the silicon to run it exists.
//
// Below 65536 this is uint16 exactly as before, so nothing changes for anything that ships today:
// SpatialHash's arrays and the neighbour cache keep their current size and the golden hashes do
// not move. Hand-rolled rather than std::conditional because core/ includes no <type_traits>.
template <bool Wide>
struct ParticleIndexFor {
  using type = uint16_t;
};
template <>
struct ParticleIndexFor<true> {
  using type = uint32_t;
};
using ParticleIndex = typename ParticleIndexFor<(kMaxParticles > 65535)>::type;
static_assert(kMaxParticles <= 4294967295u, "particle indices are at most uint32");

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
// d: particle rest separation, and the single strongest lever on cost. Filling a given volume
// needs particles proportional to 1/d^3, so this is what buys frames -- coarsening 1.5 -> 3.0 cuts
// the water tank from 3000 particles to 375 at the SAME waterline (lit fraction 40.0% -> 42.9%).
//
// 3.0 is the measured limit, not a round number. At 2.5 the fluid is clean but costs 648
// particles; at 3.0 the settled lattice becomes visible looking straight down the bottom face,
// which is why kSplatRadiusWorld went from 5/3 of the spacing to 2x (see below). Past 3.0 the
// blob needed to hide the lattice softens the waterline more than the coarser fluid saves.
//
// Everything downstream derives from this -- kSmoothRadius, kCellSize, kSlabDepth, kMaxDeltaP,
// kSplatRadiusWorld, kSplatExposure, the friction contact radius, the grid and field cell counts,
// and the scene fill counts. Changing it moves both golden hashes.
#ifndef PARTSIM_REST_SPACING
#define PARTSIM_REST_SPACING 3.0f
#endif
constexpr float kRestSpacing = PARTSIM_REST_SPACING;

// Scene presets -- and the fixtures in tests/ -- are written as absolute counts against THIS
// spacing. Such a count describes a FILL LEVEL: "water tank" means a waterline, not the number
// 3000, and settle(1500) means a 15%-full box. Coarsening the particles has to scale them by
// 1/d^3 or the intent is lost -- a scene silently becomes a function of the pool size via the
// clamp in Simulation::applySceneTargets, and a test silently starts measuring an overfull box
// that cannot settle by construction.
constexpr float kRefSpacing = 1.5f;
// Exactly 1.0f at the reference spacing (a value divided by itself), so the default build stays
// bit-identical and this scaling cannot move a golden hash on its own.
constexpr float kFillCountScale =
    (kRefSpacing * kRefSpacing * kRefSpacing) / (kRestSpacing * kRestSpacing * kRestSpacing);
// Steps a test fixture needs to reach rest, relative to the reference spacing.
//
// Finer particles shed momentum more slowly: each carries less of it, the corrections per step are
// clamped to 0.5*d and so shrink with d, and there are more layers to settle through. A step count
// written for one spacing therefore measures "has it finished yet" rather than "does it settle" at
// any other. Linear in 1/d, which matches the clamp; verified against the tiers rather than
// derived from first principles.
constexpr int settleSteps(int refSteps) {
  return (int)((float)refSteps * kRefSpacing * 2.0f / kRestSpacing + 0.5f);
}

constexpr int particlesForFill(int refCount) {
  return (int)((float)refCount * kFillCountScale + 0.5f);
}
constexpr float kSmoothRadius = 2.0f * kRestSpacing;  // h: SPH kernel support (3.0)
constexpr float kCellSize = kSmoothRadius;            // sort cell; MUST be >= h

// Depth given to a flat panel so it has a volume to simulate in. Must be ~1.5h: at only 2
// units the depth cannot fit two particle layers at rest spacing, and the frustration shows
// up as a slab that never settles (measured: 400 particles churn at depth 2 or 3, settle at
// 4.5). Irrelevant for a closed cube, whose quads already span the box.
constexpr float kSlabDepth = 3.0f * kRestSpacing;

// The physics rate. The firmware displays at 30 Hz, so 1/60 means two solver steps per displayed
// frame and 1/30 means one -- and the solver is 95.8% of the frame, so this halves the cost
// outright. What it trades is incompressibility, which is a look-and-feel judgement, not a number.
//
// The CFL headroom is unchanged by the move, which is the non-obvious part: the cap is 0.4*h/dt,
// and coarsening the particles doubled h at the same time as this halves the rate. 1/60 at h=3.0
// and 1/30 at h=6.0 are both a cap of 72 units/s, against sqrt(2*g*32) = 59 for a full-height
// fall. This is not a coincidence to rely on -- change kRestSpacing without re-checking it and the
// fluid starts tunnelling through its own neighbours.
//
// STAYS AT 60, and the attempt to halve it is worth recording because the cost was not where the
// numbers said it would be. Measured at equal simulated time:
//
//   dt     iters   ms per simulated second   rho     settled column   SAND HEAP
//   1/60     2            16.32             1.0231       22.63          5.65
//   1/60     1            14.22             1.0362       21.86          0.00
//   1/30     2             7.86             1.0581       20.88          0.01
//   1/30     3             9.51             1.0446       21.54            -
//   1/30     2 (eps .02)   8.10             1.0422       21.73          0.00
//
// On water alone, 30 Hz is 2.08x for an invisible cost: rendered at equal simulated time the
// waterline sits at the same height and the lit fraction moves 43.9% -> 42.4%. The compression it
// adds can be partly bought back by retuning kCfmEpsilon 0.05 -> 0.02 (0.01 is unstable), and
// more iterations do NOT buy it back -- even four leaves rho at 1.036, so it is the timestep, not
// an iteration shortage.
//
// What kills it is SAND. Granular friction is a position-based projection bounded by mu * contact
// overlap, and that bound is a position quantity while the sliding it resists grows with dt. At
// 60 Hz the heap holds at 5.65 while water spreads flat at 0.00 -- the defining difference between
// the two materials. At 30 Hz the heap collapses to 0.00 and sand becomes indistinguishable from
// water. Scaling the Coulomb bound by the timestep ratio does not recover it (measured 0.03).
//
// Dropping to one iteration at 60 Hz collapses the heap the same way, for only 1.11x -- so it is
// the worse deal on both axes and is not the fallback the plan assumed it was.
//
// The lever is real for a liquid-only build and the macro is left here so that build is one flag
// away. It is not a default while sand ships.
#ifndef PARTSIM_FIXED_DT_DEN
#define PARTSIM_FIXED_DT_DEN 60.0f
#endif
constexpr float kFixedDt = 1.0f / PARTSIM_FIXED_DT_DEN;
// 2 beats 3 measurably: it settles faster (mean speed 0.03 vs 0.30 after 500 steps at
// 3000 particles) and costs 28% less. More iterations are not better here.
#ifndef PARTSIM_SOLVER_ITERATIONS
#define PARTSIM_SOLVER_ITERATIONS 2
#endif
constexpr int kSolverIterations = PARTSIM_SOLVER_ITERATIONS;
constexpr int kMaxSubsteps = 3;

// Gravity is tuned for stability at 60Hz rather than physical scale: falling the full
// 32-unit height must not exceed the CFL velocity cap of 0.4*h/dt = 72 units/s.
// sqrt(2*g*32) = 59 units/s at g = 55, comfortably inside it.
constexpr float kGravityMag = 55.0f;
// Relaxation in the lambda denominator. Sized against the actual sum-of-squared-gradients,
// which is ~0.7 for a saturated neighbourhood at these kernel constants.
#ifndef PARTSIM_CFM_EPSILON
#define PARTSIM_CFM_EPSILON 0.05f
#endif
constexpr float kCfmEpsilon = PARTSIM_CFM_EPSILON;
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

// --- neighbour cache -------------------------------------------------------
// The 27-cell gather is a 3h box scanned for an h-radius sphere, so most of what it touches is
// out of range: measured 19.0 useful neighbours per 70.8 candidates, 27%. The solver pays that
// scan FIVE times per step (2 iterations x density+correction, plus XSPH). Building the list once
// and reusing it inside the iteration loop trades memory for four of those scans.
//
// Rejected in the Milestone 3 plan at 384KB -- but that was sized for 4096 particles. At the
// counts that actually run it is a tenth of that.
#ifndef PARTSIM_NEIGHBOUR_CACHE
#define PARTSIM_NEIGHBOUR_CACHE 1
#endif
// 64, against a measured worst case of 50 across every scene at the margin below (mean 26.2).
// Overflow TRUNCATES, deterministically -- dropping the furthest few neighbours of an over-dense
// particle is a far better failure than a pool sized for a worst case that never occurs.
// SpatialHash::truncated() reports it, and it is zero as configured.
#ifndef PARTSIM_MAX_NEIGHBOURS
#define PARTSIM_MAX_NEIGHBOURS 64
#endif
constexpr int kMaxNeighbours = PARTSIM_MAX_NEIGHBOURS;

// Cache radius as a multiple of the smoothing radius, and the term that makes the cache worth
// having. Above 1.0 the list also holds particles just OUTSIDE the kernel, which the correction
// pass can then pull inside without the frozen list being rebuilt.
//
// 1.15 is measured, and the measurement is the argument. A frozen neighbourhood does not give a
// wrong answer, it gives a slowly-settling one: at margin 1.0 the hydrostatic fixture reaches
// mean|v| 0.47 after 1500 steps against 0.053 uncached, and needs 4000 steps rather than 2500 to
// come to rest -- visible as shimmer. At 1.15 it reads 0.053, matching the uncached solver
// exactly. 1.3 is no better and costs 19% more.
//
// Not a proof. The clamp on a single iteration's correction is 0.5*d, so two particles could in
// principle separate by 2*d = 1.0*h over two iterations and need a margin of 2.0. 1.15 covers what
// the fluid actually does, and the failure mode if it ever does not is a marginally stale
// neighbourhood -- which the golden hashes on three targets would catch as a divergence.
#ifndef PARTSIM_NEIGHBOUR_MARGIN
#define PARTSIM_NEIGHBOUR_MARGIN 1.15f
#endif
constexpr float kNeighbourRadius = kSmoothRadius * PARTSIM_NEIGHBOUR_MARGIN;
static_assert(kMaxNeighbours <= 255, "per-particle neighbour counts are uint8");

// --- materials -------------------------------------------------------------
// --- feature switches ------------------------------------------------------
// Sand and fire compile out. This pays in MEMORY, not tidiness: accumulation is
// texels x channels x 2B, so a water-only build is a third of the accumulation buffer, and
// dropping heat also removes the FieldGrid's two buffers and the whole advection pass. At the
// 32x32 six-face profile that is ~25KB on a node with 25.5KB free -- the difference between a
// display node driving two faces and three.
//
// Both default ON, so every existing build is unchanged.
#ifndef PARTSIM_ENABLE_SAND
#define PARTSIM_ENABLE_SAND 1
#endif
#ifndef PARTSIM_ENABLE_HEAT
#define PARTSIM_ENABLE_HEAT 1
#endif

// Per-particle colour, for beaker mode: liquid carries a dye that MIXES, so red poured into blue
// converges to pink. Off by default -- it costs a particle array and two accumulation channels,
// and no scene outside beaker mode has a use for it.
#ifndef PARTSIM_ENABLE_CHROMA
#define PARTSIM_ENABLE_CHROMA 0
#endif

// Material ids stay contiguous from 0, so kMaterialCount is the table size either way and
// Solver::defaultMaterials indexes it directly.
enum Material : uint8_t {
  kWater = 0,
#if PARTSIM_ENABLE_SAND
  kSand,
#endif
  kMaterialCount
};
#if !PARTSIM_ENABLE_SAND
// Named so scene tables and tests can still say kSand without every use needing a guard. It is
// deliberately NOT a valid material index -- anything that tries to spawn it must be rejected,
// not silently turned into water, or a sand scene would quietly become a water scene.
constexpr uint8_t kSand = 0xFF;
#endif

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
// Expressed as a MULTIPLE of the rest spacing, not an absolute size. A blob has to be wider than
// the gap between particles or the fluid reads as separate dots -- 2.5 against a spacing of 1.5 is
// 1.67x, and that ratio is what makes the surface look continuous. Left absolute, coarsening the
// particles would silently break the look while every constant still "looked right".
// As an exact fraction, multiply-then-divide: 1.5 * 5 / 3 is exactly 2.5 in float, whereas
// 1.6667f * 1.5f is 2.50005 and quietly moved the golden PIXEL hash the first time I wrote it.
//
// 2x (6/3) rather than the 5/3 that reproduced the old absolute 2.5. At kRestSpacing 3.0, 5/3
// leaves the settled crystal lattice plainly visible through the bottom face; 2x hides it with the
// waterline still crisp, and 7/3 hides it while visibly softening the waterline. Costs a footprint
// of 6 texels instead of 5 at pitch 1.0 -- 169 scanned texels per particle per face against 121.
#ifndef PARTSIM_SPLAT_RADIUS_NUM
#define PARTSIM_SPLAT_RADIUS_NUM 6.0f
#endif
#ifndef PARTSIM_SPLAT_RADIUS_DEN
#define PARTSIM_SPLAT_RADIUS_DEN 3.0f
#endif
constexpr float kSplatRadiusWorld =
    kRestSpacing * PARTSIM_SPLAT_RADIUS_NUM / PARTSIM_SPLAT_RADIUS_DEN;
#if PARTSIM_ENABLE_CHROMA
// Radius of the chroma splat, as a fraction of the weight splat's.
//
// This is the whole reason beaker mode is affordable. Colour resolution is set by the BLOB, not by
// the particle: chroma splatted through the weight kernel smears over 2*d, giving 32/(2d) ~ 5
// distinguishable colour regions across the entire cube at the shipping spacing, and red pouring
// into blue would read as a handful of lumps converging rather than as mixing (DECISIONS.md F3).
//
// Brightness needs the wide kernel -- a narrower one breaks the continuous surface that coarsening
// the particles was all about (D21). Colour does not. Splitting them doubles colour resolution to
// 32/d at the SAME particle count, which is worth the 8x more particles it would otherwise take.
#ifndef PARTSIM_CHROMA_RADIUS_FRAC
#define PARTSIM_CHROMA_RADIUS_FRAC 0.5f
#endif
constexpr float kChromaRadiusWorld = kSplatRadiusWorld * PARTSIM_CHROMA_RADIUS_FRAC;

// How fast dye equalises between neighbours, per step, riding the XSPH pass.
#ifndef PARTSIM_COLOUR_DIFFUSION
#define PARTSIM_COLOUR_DIFFUSION 0.35f
#endif
constexpr float kColourDiffusion = PARTSIM_COLOUR_DIFFUSION;

// Chroma is stored as 8.8 fixed point -- 0..kChromaOne, where kChromaOne is 255 units scaled by
// 256 -- rather than as a byte.
//
// A byte does not work, and the failure is total rather than marginal. Diffusion moves fractions
// of a unit per particle per step: a lone red particle among blue loses 89 units in one step while
// each of its ~26 neighbours gains 3.4, and truncating every one of those to an integer throws
// away 0.4 each. Measured with uint8 storage: red dye fell 98% over 600 steps and blue rose 100%.
// Dye must be conserved for a mix to converge on the right colour rather than on whatever the
// rounding drifts toward, and 8 fractional bits put the per-step rounding 256x below the smallest
// transfer that matters.
constexpr int kChromaOne = 255 * 256;
#endif

constexpr int kAttenLutSize = 64;
// Accumulated intensity that maps to the top of a colour ramp. Measured, not guessed: a dense
// water texel peaks around 6500 at these kernel constants, and setting this too low clips
// everything to white and throws the whole ramp away.
//
// DERIVED from the blob radius and the rest spacing, not tuned independently. A texel's
// accumulation is the sum over the particles inside the projected splat column, which holds
// pi*R^2 * kSplatInfluence * density particles at a density of 1/d^3 -- so accumulation goes as
// R^2/d^3. Hold 7200 fixed and coarsening from 1.5 to 3.0 measures mean luminance 59.7 -> 13.4
// while the LIT FRACTION stays at 40%: the same waterline, rendered too dark to read. Widen the
// blob ratio instead and it clips to white. Both are the same missing term.
//
// Written as a ratio against the reference configuration so it is exactly 7200 there -- numerator
// and denominator are the identical expression on identical values, so the quotient is exactly
// 1.0f and this cannot perturb a golden hash at the defaults.
// Pinned to the reference RATIO (5/3), not to PARTSIM_SPLAT_RADIUS_NUM -- using the current ratio
// here makes it cancel out of the quotient below, and the exposure stops tracking the blob width
// at all. Identical to kSplatRadiusWorld at the defaults, and only there.
constexpr float kRefSplatRadius = kRefSpacing * 5.0f / 3.0f;
constexpr float kSplatExposure =
    7200.0f *
    ((kSplatRadiusWorld * kSplatRadiusWorld) / (kRestSpacing * kRestSpacing * kRestSpacing)) /
    ((kRefSplatRadius * kRefSplatRadius) / (kRefSpacing * kRefSpacing * kRefSpacing));
// Channels follow the enabled materials and stay contiguous from 0, because the accumulation
// buffer is indexed as texel*kChannelCount + channel and a gap would waste a sixth of it.
enum Channel : uint8_t {
  kChWater = 0,
#if PARTSIM_ENABLE_SAND
  kChSand,
#endif
#if PARTSIM_ENABLE_HEAT
  kChHeat,
#endif
#if PARTSIM_ENABLE_CHROMA
  // Dye, splatted through a NARROWER kernel than the weight above (see kChromaRadiusWorld) and
  // premultiplied by it. All three are stored rather than two with the third implied, because the
  // colour a texel resolves to is chroma divided by the chroma channels' OWN total weight -- and
  // that total is exactly kChCR + kChCG + kChCB. Imply one and the denominator becomes circular.
  //
  // The earlier M4 sketch reinterpreted the dead sand and heat channels to avoid a fourth, on the
  // grounds that it cost +16KB on a display node then sitting at 200KB of 230. W1 made sand and
  // heat compile out and that node now measures 115KB, so the constraint the hack existed for is
  // gone and the channels can say what they mean.
  kChCR,
  kChCG,
  kChCB,
#endif
  kChannelCount
};
// Heat is a field, not particles, so its accumulated intensity needs its own scale to land in
// the same 0..kSplatExposure range the particle channels use.
constexpr float kHeatGain = 5200.0f;
// Field cells below this are not worth splatting; skipping them is most of the render cost of a
// mostly-cold volume.
constexpr uint8_t kHeatFloor = 6;

// --- colour depth ----------------------------------------------------------
// Bits per channel the panels actually display, via HUB75 binary-coded modulation.
//
// It lives here rather than in the firmware because it is a BUDGET number as much as a look: the
// DMA framebuffer is rows x width x 2B x bits, so it scales linearly. At 64x64 with two faces,
// 6-bit is 96KB and 4-bit is 64KB -- and scripts/check_esp32_budget.sh has to model the same value
// the driver is given, or the budget describes a build nobody flashes.
//
// 6 is what the refresh measurement of ~141Hz at HZ_16M was taken at. Lowering it RAISES the
// achievable refresh as well as freeing memory; the cost is visible banding in the dim end of a
// ramp, which is where fluid spends most of its range.
#ifndef PARTSIM_COLOUR_BITS
#define PARTSIM_COLOUR_BITS 6
#endif
constexpr int kColourBits = PARTSIM_COLOUR_BITS;
static_assert(kColourBits >= 1 && kColourBits <= 8, "HUB75 colour depth is 1..8 bits");

// Whether resolve() rounds its 8-bit output down to what the panel can actually show.
//
// Off by default, and deliberately: the panel does this in hardware by taking the top kColourBits,
// so enabling it changes nothing on the device and only makes the BROWSER honest about the
// banding. That is worth having -- the browser is meant to predict the hardware -- but it moves
// the golden pixel hash, so it is a per-tier choice rather than a silent default.
#ifndef PARTSIM_QUANTISE_OUTPUT
#define PARTSIM_QUANTISE_OUTPUT 0
#endif

}  // namespace partsim
