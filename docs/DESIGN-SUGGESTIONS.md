# Design suggestions: MCU-friendly ink-in-water simulation

Status: design proposal. The cost case is **measured** (section 13); the field itself is partly
built -- `core/src/InkField.cpp` covers section 2, and the projections of section 3 are not written
yet.

Everything in section 13 was measured rather than estimated, and it is the reason to take the rest
seriously: the advection step costs 5.9-14.4 ms on an ESP32-S3 against 732 ms/step for the
particles a physically full beaker would need. What this document proposes is not a faster
simulation of the same thing, it is a simulation of a different thing -- the dye, rather than the
water carrying it.

This document targets the visual reference supplied for the project: coloured ink dispersing
through clear liquid, reduced to the resolution and colour depth of the LED cube. The important
features are a dense descending plume, coherent curls, thin tendrils, detached wisps, translucent
overlap, and gradual colour mixing. Reproducing exact fluid pressure or individual water parcels is
not the goal.

The short recommendation is:

> Replace water PBF, for this effect, with a **16x16x16 fixed-point dye field advected by a small
> procedural flow field**, and render it as six low-resolution volume projections. Keep a planar or
> shallow-water surface only if a visible fill level is required, and use a few ballistic parcels
> for material that is actually between beakers.

This is a different choice from a surface-only shallow-water simulation. A height field is a good
match for a waterline, but the supplied reference is defined by structures inside the volume. A
small 3D scalar field is the lowest-cost representation that can retain those structures.

## 1. What the output must preserve

At 32x32 or 64x64 per face, the eye will not judge Navier-Stokes accuracy. It will judge a smaller
set of cues:

- a connected main plume rather than independent dots;
- motion aligned with effective gravity, with visible inertia after the cube moves;
- slowly changing, coherent curls rather than frame-to-frame noise;
- a mix of saturated cores and faint translucent veils;
- branches and tendrils that persist for several frames;
- one shared 3D structure seen from all six sides;
- colour transport and mixing without immediately averaging the whole volume to one colour;
- no discontinuity when a feature crosses a cube edge.

The existing PBF solver spends most of its time on properties that are not in that list: particle
neighbour gathering, incompressibility constraints, wall-density compensation, collision
corrections, and XSPH. The measured result in [`RESOURCES.md`](RESOURCES.md) is 71.56 ms
per solver step at 512 particles on the ESP32-S3; two 60 Hz steps per displayed frame limit that
configuration to about 7 fps. A physically full beaker is 1,905 particles at the old beaker
spacing. The current renderer also ignores water farther than 8 of the cube's 32 world units from
a panel, even though the solver still pays for it.

For the reference effect, the clear carrier liquid can be implicit. Only the coloured material and
the flow that moves it need state.

## 2. Recommended representation

### 2.1 A dye grid, not water particles

Add an `InkField` on a regular object-space grid. Start at `16^3 = 4096` cells. The grid describes
only coloured dye concentration; an empty cell means clear carrier liquid, not empty space.

For the initial red/blue beaker case, store two premultiplied concentrations:

```cpp
template <int N>
struct InkField {
  uint8_t dye[2][2][N * N * N];  // ping/pong, dye A/B
  uint8_t current;
  Vorton vortons[8];
};
```

`dyeA` and `dyeB` are masses, not hue ratios. Opacity comes from their sum and colour comes from
their weighted palette colours. This makes red plus blue become magenta without storing RGB per
cell. A later arbitrary-colour tier can use three concentration channels.

Do not store clear water density. Do not allocate per-cell pressure, divergence, or a 3D velocity
field in the first version.

| Grid | Cells | One ping-ponged byte channel | Two dye channels | Three dye channels |
|---:|---:|---:|---:|---:|
| 12^3 | 1,728 | 3.4 KiB | 6.8 KiB | 10.1 KiB |
| **16^3** | **4,096** | **8 KiB** | **16 KiB** | **24 KiB** |
| 20^3 | 8,000 | 15.6 KiB | 31.3 KiB | 46.9 KiB |
| 24^3 | 13,824 | 27 KiB | 54 KiB | 81 KiB |

Sixteen cells across the cube is intentionally below the panel resolution. Trilinear advection,
projection through several depth cells, and bilinear output scaling hide the grid. If 16^3 is not
enough, test 20^3 before adding more fluid physics.

**And 20^3 genuinely fits.** Measured advection cost per step on an S3, at the pessimistic end of
the two scaling factors in section 13, against a 20 Hz field:

| Grid | ms/step | one core, at 20 Hz |
|---:|---:|---:|
| 12^3 | 6.1 | 12% |
| **16^3** | **14.4** | **29%** |
| 20^3 | 28.0 | 56% |
| 24^3 | 48.4 | 97% |

So the fallback ladder in section 11 is a budgeted ordering rather than a preference: 24^3 is the
wall, and there is room above 16^3 to spend if the projections turn out to need it.

### 2.2 A procedural velocity field

The ink needs coherent motion, but it does not need a fully solved velocity grid. Evaluate velocity
at a cell from a few cheap components:

```text
velocity(position) = bulk drift
                   + gravity-relative settling
                   + angular motion from the IMU
                   + sum of 4-8 decaying vortons
                   + 2-3 very-low-frequency curl modes
```

A `Vorton` is a small flow primitive: centre, axis, radius, strength, and remaining life. Inside its
radius it contributes a tangential velocity proportional to `cross(axis, position - centre)` and a
LUT falloff. It does not collide with anything and has no neighbours. Eight vortons should occupy
well under 256 bytes.

**Their storage is not the cost that matters; their evaluation is.** The velocity above is
evaluated once per cell per step, so eight vortons plus two curl modes is 4096 cross products and
falloff lookups per update. Measured, that is **50% of the whole advection step** -- the single
largest term in this design, and the one an earlier draft of this document budgeted in bytes
rather than in cycles. Size the vorton count against that figure, not against 256 bytes. section 11
has the cheaper evaluation, and what it is actually worth.

Create vortons in deterministic pairs when:

- dye is injected, producing the curled head at the front of a plume;
- high-pass container acceleration exceeds a threshold;
- angular velocity changes sharply;
- an incoming pour strikes the existing liquid.

Move and decay them slowly. Never choose a new random direction every frame: that produces visual
noise which averages to fog. Seed their axes once from the existing deterministic `Rng`, then let
their phase evolve continuously.

The image is of dye that tends to descend. Give the dye a small drift along object-space gravity.
For smoke or hot ink, the sign can be reversed. Make the drift slightly stronger in concentrated
cells so a dense plume forms a rounded leading head while weak wisps lag behind.

### 2.3 Fixed-point semi-Lagrangian advection

For each destination cell, trace backwards through the procedural velocity and gather the old dye:

```text
source = cellCentre - velocity(cellCentre) * dt
newDye = trilinearSample(oldDye, source)
```

This is the same useful stability property already used by
[`core/src/FieldGrid.cpp`](../core/src/FieldGrid.cpp), but the MCU version should use fixed point:

- cell coordinates in Q8.8 or Q12.4;
- 8-bit trilinear weights whose sum is 256;
- 16- or 32-bit intermediates for the eight weighted samples;
- LUTs for radial falloff and periodic flow modes;
- no divide, square root, or transcendental in the cell loop;
- one calculation of the source coordinate shared by every dye channel;
- **round, never truncate, when narrowing the weighted sum.** Truncation error is one-sided, so
  every cell leaks up to one count per step. Measured at 21% of the dye gone in six steps with no
  flow at all -- and it presents as exactly the diffusion this section warns about below, so the
  natural "fix" is to tune a decay constant that was never the cause;
- **agree on where a cell sits.** If the loop puts cell `x` at coordinate `x + 0.5` while the
  sampler and the injector put it at `x`, the zero-velocity gather samples halfway between two
  cells and slides the entire field half a cell every step. It reads as a settling bias, and it
  moves the dye the same way whichever direction gravity points;
- **no flux through the container wall.** Drop the outward normal velocity component in the
  outermost cell. Without it, every backtrace that leaves the grid clamps to the same edge sample,
  so dye driven against a wall is duplicated rather than piled up -- 93x mass gain in ten steps with
  the plume in a corner, which looks like the ink glowing where it touches the glass.

None of those three is visible as a defect in a rendered frame; all three look like plausible
fluid behaviour. They are listed here because a field this small has no other error signal.

Run field physics at 20 Hz initially and render at 30 Hz. Ink movement is slow and continuous; it
does not need the 60 Hz rate required by the current granular contact model. If the field visibly
steps, interpolate the flow phase or the rendered projections, not the entire 3D state.

Semi-Lagrangian advection is deliberately a little diffusive. That helps produce translucent veils
but will eventually erase tendrils. Before adding an expensive higher-order solver, try these
cheaper controls:

1. use a concentration response LUT at render time so faint material remains visible;
2. decay low concentrations more slowly than mid concentrations;
3. inject narrow features continuously while a pour is active;
4. add one bounded six-neighbour sharpening pass only if the 16^3 prototype visibly turns to fog.

MacCormack/BFECC advection can be a host-only comparison, but it should not be the MCU baseline: it
adds passes, scratch storage, and clamping work.

### 2.4 Limit work to the active region

Track the minimum and maximum occupied cell on each axis. Update that box plus a two-cell halo.
Early in a pour, most of the 4096-cell volume is clear and should cost nothing. Fall back to a full
grid walk once the active box covers most of the volume. This is simpler and more predictable than
a sparse cell list.

Zero the destination box explicitly and expand it by the maximum backtrace distance. Otherwise a
fast vorton can sample dye from beyond the tracked bounds and clip a tendril.

## 3. Rendering the field

### 3.1 Render projections, not voxel splats

The existing heat path splats every active field cell onto panels. That is acceptable for the
current sparse `11^3` flame field, but a dense ink field plus the large particle footprint would
recreate a render bottleneck.

Instead, render a `16x16` intermediate image for each driven face. Each intermediate texel walks
one column through the 3D field. Composite front to back with an integer opacity LUT:

```text
transmittance = 255
colour = 0

for each voxel from the panel inward:
    alpha = opacityLut[dyeA + dyeB]
    sampleColour = mix(colourA, colourB, dyeA, dyeB)
    colour += transmittance * alpha * sampleColour
    transmittance *= 255 - alpha
```

The exact scaling should use integer arithmetic and saturation -- **including the colour mix, which
the pseudocode above hides a divide inside.** `mix()` by dye mass wants a division by `dyeA + dyeB`;
take the reciprocal from a 512-entry LUT instead. On a host with hardware divide this is worth only
6%, which understates it: the rule in section 2.3 exists because the target has no such thing.

Opposite faces traverse the same field in opposite depth order. Upscale the `16x16` result to 32x32
or 64x64 with bilinear filtering, then pass it through the existing quantisation and panel mapping.

At 16^3, six complete projections visit only `6 * 16^3 = 24,576` voxel samples per rendered frame.
A display node that owns two faces visits 8,192. This is independent of panel resolution until the
cheap upscale.

### 3.2 Make it read as ink rather than fire

The physical reference has a bright neutral background; an unlit HUB75 panel has a black one.
Orange additive blobs on black will tend to read as flame. The render should therefore provide
some of the missing carrier-liquid cues:

- a dim neutral or cool-blue background, around 5-12% brightness;
- low bloom and no clipping to white;
- a concentration curve with bright cores and still-visible faint wisps;
- slower gravity-driven motion than the existing fire field;
- optional faint edge or meniscus highlights when a free surface is enabled.

[`DECISIONS.md`](DECISIONS.md) D72 already found that strong browser bloom hides beaker
colour. Keep bloom off for comparison images and tune the panel output first.

### 3.3 Preserve cube continuity

All faces must sample the same `InkField` in object coordinates. Precompute the voxel-column order
for each of the six axis-aligned panels at initialisation; do not maintain six independent 2D
simulations.

The projections through adjacent faces are views from different directions and need not have
identical edge colours. They should, however, share the same near-boundary sample. If a visible
one-texel seam remains, blend the outermost projection texel with that shared boundary sample or
average the paired physical edge texels after projection. Add a cube-net regression image before
tuning this by eye.

## 4. Motion input

Reuse the separation already present in `MotionSource`:

- low-pass object-space gravity sets the plume's settling direction;
- high-pass container acceleration adds a decaying bulk impulse and spawns vorton pairs;
- gyroscope angular velocity adds approximate solid-body rotation, `omega x radius` -- **this one
  needs a new accessor.** `MotionSource` consumes the gyro for its complementary filter and bias
  tracking but publishes only `down()`, `gyroBias()`, `trust()` and `containerAccel()`; angular
  velocity never leaves the class. The low-pass/high-pass split this section relies on is genuinely
  already there, but `omega` is not;
- a short impulse envelope preserves follow-through after the user's hand stops.

The bulk field should not rotate instantly with small accelerometer noise. Update its gravity basis
at the field rate and low-pass it. Feed fast motion into vortons instead. This gives the eye both a
stable direction and responsive curls.

Clamp source displacement to at most roughly one grid cell per update. That is the field equivalent
of a CFL guard: semi-Lagrangian advection remains numerically stable for a larger displacement, but
a tendril can jump over cells and disappear.

## 5. Beaker filling, pouring, and chaining

The field above assumes the carrier liquid fills the simulated region. If the installation still
needs a visible amount of liquid and transfer between beakers, keep **volume** separate from
**appearance**.

### 5.1 Cheapest useful surface model

Start with a conserved scalar liquid volume and a gravity-aligned fill plane. Give the plane a
slowly filtered normal plus two or four damped wave modes excited by acceleration. The occupancy
test for a dye cell is then simply whether its centre is below the surface.

This will not produce overturning waves, but it supplies the visible waterline and limits where dye
may exist at a tiny fixed cost. Upgrade it to a `16x16` shallow-water/virtual-pipe surface only if
the planar version fails a rendered comparison. Do not make the shallow-water surface the primary
ink representation; it cannot create the volumetric plume in the reference.

### 5.2 Spill parcels, not PBF particles

Replace particle-identity transport with a small number of aggregate parcels:

```cpp
struct InkSpillParcel {
  uint16_t volume;
  uint16_t dyeA;
  uint16_t dyeB;
  int16_t u;
  int16_t v;
  int8_t velocity[3];
};
```

`InkSpillParcel` carries dye as `dyeA`/`dyeB` **masses**, while `SpillParticle` on the existing
chain carries `cr`/`cg` as a normalised barycentric triple with blue implied. Both conventions are
right where they are -- a field wants masses, because their sum is the opacity, which is exactly the
denominator `M4-PLAN.md` section 2 found goes circular when a component is implied -- but they will
coexist in one binary and a parcel crosses between them. Name the conversion in one place and say
which end is authoritative, or the two will drift the way `Lsm6dsox::Raw` did (`DECISIONS.md` D42).

When the fill surface crosses the open rim, remove a volume proportional to the outward flow and
emit one or a few parcels. The receiver adds volume, injects the parcel's dye mass into cells below
the impact point, and creates a vorton pair from its momentum. This preserves the properties that
matter to the existing chain: amount, colour, exit corner, entry corner, and impact speed.

Use 16-64 non-interacting ballistic visual droplets only while material is actually between
vessels. On impact they disappear into the field. They require position and velocity integration,
but no neighbours, constraints, pressure, or viscosity.

Liquid volume must be exactly conserved by the chain accounting. Dye concentration is visual state
and may lose a little mass to semi-Lagrangian diffusion or decay, but sent and received dye mass
should still be counted so a dropped packet cannot create colour.

## 6. Multi-node wire format

Display nodes need concentration state, not the procedural velocity field. For a 16^3 field:

- two raw dye channels are 8,192 bytes per field frame;
- three raw channels are 12,288 bytes;
- at 20 MHz SPI these take about 3.3 ms and 4.9 ms respectively per broadcast -- and
  `SPI-HANDOFF.md` records **10 MHz as a legitimate fallback**, at which they are 6.6 ms and 9.8
  ms. Both still fit 20 Hz, but the fallback should be planned for rather than discovered;
- at 20 updates/s the raw payload uses about 1.31 or 1.97 Mbit/s before framing.

That fits the present broadcast topology, but it is larger than the current device-side
`frameMaxBytes()` for 512 particles and the existing heat field. The format therefore needs an
explicit version bump rather than overloading `heatBytes`.

Suggested frame changes. Three of these are **already true of `SimFrame` and must be kept rather
than added** -- re-specifying them invites someone to re-litigate a solved problem:

- add grid dimensions, dye channel count, and dye payload length to `FrameHeader`;
- RLE each channel independently, since a young plume is mostly zero;
- allow raw storage when RLE would expand the field -- `rleEncode` currently refuses instead;
- resize the DMA staging buffers from the actual field tier;
- send only the current dye buffers, never ping/pong scratch or vortons;
- *keep* the fletcher16 over header and body (`SimFrame.cpp`);
- *keep* `geomHash`, which already rejects a mismatched pair of builds;
- *keep* holding the last valid frame on a checksum or geometry mismatch.

RLE is an optimisation, not a capacity assumption. A fully mixed field may be incompressible and
must still fit raw. Delta tiles can be considered later if measured SPI time becomes relevant.

## 7. Suggested code changes

Keep the PBF implementation as the reference and as the backend for scenes that genuinely need
particles, especially sand. Add the ink backend beside it first.

| Area | Suggested change |
|---|---|
| `core/include/partsim/Config.h` | Add one named `PARTSIM_TIER_INK`, grid size/capacity, dye-channel count, and a field-backend selection. The tier should disable unused PBF/sand/heat pools rather than creating another arbitrary combination of switches. |
| `core/include/partsim/InkField.h` | Add fixed-size dye buffers, active bounds, vorton state, views, injection, and stepping API. |
| `core/src/InkField.cpp` | Implement deterministic fixed-point advection, vorton evaluation, settling, decay, and boundary handling. |
| `core/include/partsim/RenderState.h` | Add a draw-only `InkBuffer`/`InkView` analogous to `HeatBuffer`, without solver scratch. |
| `core/include/partsim/Renderer.h` and `core/src/Renderer.cpp` | Add the column compositor and low-resolution upscale. Keep the old splat renderer for PBF and heat. |
| `core/include/partsim/Simulation.h` and `core/src/Simulation.cpp` | Own the ink field, route object-space motion to it, and select one liquid backend at compile time. Do not run PBF and ink for the same bulk liquid. |
| `core/include/partsim/SimFrame.h` and `core/src/SimFrame.cpp` | Add a versioned dye-field payload, raw/RLE mode, bounds checks, and round-trip decode into `InkBuffer`. |
| `platform/app/src/App.cpp` | Send field frames from `masterStep()` and render the received field in `displayStep()`. |
| `platform/wasm/bindings.cpp` and `platform/wasm/web/*` | Expose the ink tier, injection controls, and an unbloomed diagnostic view. |
| `platform/host/` | Add a deterministic plume fixture, cube-net dump, benchmark, and memory report for 12^3/16^3/20^3. |
| `tests/` | Add advection, mixing, bounds, determinism, wire-format, seam, no-allocation, and budget tests. |
| `core/src/RenderState.cpp` | The ink grid's dimensions must be derived in ONE place, the way `heatCellSize`/`heatGridDim` already are. `FieldGrid.cpp` carries the warning: two copies of the derivation fail silently, as a plume drawn in the wrong place rather than as an error. |
| `scripts/golden_hash_esp32.txt` and friends | A new tier owes a state and pixel hash on each target -- ROADMAP W2 calls this "the ongoing tax". Budget it rather than discovering it. |
| `core/CMakeLists.txt`, `core/library.json` | Register the new source/header for all three targets. |
| `scripts/check_esp32_budget.sh` | Assert the named ink tier's internal-SRAM pools and DMA staging buffers. |

Do not immediately generalise `FieldGrid` into a large framework. It is the right structural
example, but changing the working fire implementation while exploring ink increases the regression
surface. Copy the small storage/sampling pattern into `InkField`; factor common code only after the
ink prototype passes its visual and device tests.

## 8. Development order

### Phase A: prove the look on the host

1. Implement a 16^3, one-colour `InkField` with fixed-point semi-Lagrangian advection.
2. Add constant gravity settling, one injection source, and four vortons.
3. Add six 16x16 volume projections and bilinear upscale.
4. Dump cube nets for still, tilt, shake, and continuous-injection sequences.
5. Compare with PBF by panel output, not by internal state.

The first prototype should not include a free surface, radio, arbitrary RGB, or pressure solve.

### Phase B: make it interactive

1. Drive gravity, bulk impulse, and angular flow from the existing motion path.
2. Add the second dye channel and verify a red/blue plume retains distinct regions before mixing.
3. Tune the lifetime of vortons and the concentration response LUT.
4. Test 12^3, 16^3, and 20^3 at the same recorded IMU sequence.

Choose the smallest grid whose panel output is not visibly worse at normal viewing distance.

### Phase C: fit and measure on the target

1. Add the named ink tier and compile unused particle storage out.
2. Benchmark field step, projection, upscale, resolve, and blit separately.
3. Start at 20 Hz field / 30 Hz display; increase only with measured headroom.
4. Run from internal SRAM first. PSRAM is storage of last resort, not working memory for the cell
   loop.
5. Record cycles per cell and per active vorton so another MCU can be sized from measurements.

### Phase D: restore beaker semantics

1. Add conserved fill volume and the planar surface.
2. Clip/adapt dye at the occupancy boundary.
3. Add aggregate spill parcels and receiver injection.
4. Add ballistic between-vessel droplets only after the field transfer works.
5. Upgrade the surface model only if a panel render identifies a specific failure.
6. **Re-establish the chain's conservation guarantee -- it is not inherited.** M4's volume
   accounting rests on particle identity, and `test_beaker_spill`, `test_spill_chain` and
   `test_multinode` are all written against it. None of them carries over to a scalar volume plus
   aggregate parcels. The property they protect still matters exactly as much: in a closed ring, a
   lost packet must read as a lost packet and not as beakers quietly emptying over minutes, which
   is what `SpillQueue::dropped` and `totalOut` exist to distinguish.

## 9. Tests and acceptance criteria

### Deterministic unit tests

- a field with no source remains exactly zero;
- a source increases only the selected dye channel;
- with zero velocity, concentration remains in place apart from configured decay;
- gravity in each of the six axis directions moves the plume the expected way;
- two dye channels advect independently and resolve to the expected mixed colour;
- no advection lookup crosses the grid bounds;
- the active-box result matches a forced full-grid update bit for bit;
- repeated runs with the same seed and motion samples hash identically;
- encoded raw and RLE frames decode identically;
- a corrupt or truncated frame changes no display state;
- the ink tier performs no dynamic allocation after `init()`.

### Visual fixtures

Capture at least these as cube nets and short browser recordings:

1. one dense drop descending into a clear, still volume;
2. continuous pour forming a trunk and a curled leading head;
3. a 35-degree tilt with the plume bending smoothly;
4. an abrupt lateral shake followed by settling;
5. slow rotation around two axes;
6. red injection into blue, before, during, and after mixing;
7. a feature crossing each physical cube edge;
8. the same sequence at 12^3, 16^3, and 20^3, bloom disabled.

The correct comparison question is:

> At normal viewing distance, does this read as a continuous volume of ink in moving liquid, and
> is the extra structure of the next grid size actually visible on the panels?

### Initial device budgets

Treat these as targets to verify, not predicted measurements:

- 16^3, two dye channels;
- at most eight vortons;
- 20 field updates/s and 30 display frames/s;
- no per-frame heap allocation;
- field state and all render scratch in internal SRAM;
- at least 20% internal-SRAM headroom after radio and platform initialisation;
- no missed panel deadline when the field is fully occupied and RLE gives no benefit.

## 10. What not to build first

- **Another particle fluid solver:** SPH, FLIP, MPM, or a differently tuned PBF still pays to
  simulate the carrier liquid rather than the visible dye.
- **A full 3D pressure projection:** it adds velocity, divergence, pressure buffers, and repeated
  grid sweeps. The LED output may not reveal the improvement.
- **Lattice Boltzmann D3Q19:** local access is attractive, but 19 populations per cell are the
  wrong memory trade for a small MCU.
- **A surface-only height field:** cheap for slosh, unable to form the volumetric structures in
  the reference.
- **Independent face simulations:** cheap but they cannot preserve a feature around a corner.
- **Per-frame random turbulence:** visually noisy and temporally incoherent.
- **High grid resolution before projection is tested:** the panels and optics may hide everything
  above 16^3.

## 11. Fallbacks and upgrades

If 16^3 advection is still too expensive, fall back in this order:

1. 12^3 field, still rendered with bilinear upscale;
2. one dye concentration channel plus a global palette transition;
3. 12-15 Hz field updates with 30 Hz render interpolation;
4. four vortons instead of eight;
5. an active-region update;
6. a 32-64 element metaball/tendril system with short ribbon histories.

The last option is no longer a fluid simulation, but it can retain the reference's silhouette and
temporal motion more convincingly than a handful of large PBF particles.

If the basic field is cheap and the image still lacks curl, upgrade in this order:

1. longer-lived and paired vortons;
2. one additional low-frequency curl mode;
3. bounded post-advection sharpening;
4. a coarse stored velocity field at 8^3, upsampled for the 16^3 dye field. **Measured at 1.33x,
   not the 8x the arithmetic suggests**: evaluating velocity at 512 nodes instead of 4096 removes
   seven eighths of a term worth half the step, but trilinear interpolation of three velocity
   components costs back 77% of what it saves. Still worth taking -- it is a 25% saving on the
   advection step for 1.5 KB -- but it is tuning, not a structural fix, and nobody should re-derive
   the 8x and be disappointed;
5. a small number of pressure iterations on that 8^3 velocity field.

Do not jump directly to a 16^3 three-component ping-ponged velocity field. At 8-bit components it
adds 24 KiB before pressure/divergence scratch, and its visual value should be demonstrated first.

## 12. Expected result

The proposed model changes the cost from irregular neighbour work that grows badly with particle
count and fill depth to bounded, contiguous work over a few thousand byte-sized cells. A full cube
and a nearly empty one have the same worst-case grid cost; a sparse plume is cheaper with active
bounds. The two-dye 16^3 state is only 16 KiB, and rendering all six low-resolution projections
touches 24,576 voxel samples per frame.

That does not prove a particular frame rate on an unmeasured MCU. It does remove the architectural
reason the current beaker needs far more than an ESP32-S3. It also targets the supplied visual more
directly: the state being simulated is the thing the viewer sees -- coloured concentration and its
coherent motion -- instead of thousands of invisible carrier-liquid interactions.

---

## 13. What this actually costs, measured

Everything above was written as a proposal. This section is measurement, and it is what the
proposal now rests on.

**Method.** The advection loop of section 2 and the projections of section 3.1 were implemented as
a standalone fixed-point kernel and timed on the host, then converted with two factors this
repository has measured on hardware:

| calibration workload | host | device | factor |
|---|---|---|---|
| solver, float and gather-heavy | 0.739 ms | 74.65 ms | **101x** |
| splat + resolve, integer | 0.093 ms | 22.96 ms | **247x** |

Integer code scales **2.4x worse** to the S3 than the float solver does, because the host
vectorises it and Xtensa cannot. That is the opposite of the intuition that an integer kernel must
port well, and it is why every figure below is given as a range between the two factors rather than
resting on one.

### 13.1 The field

| | S3, projected |
|---|---|
| advect 16^3, velocity per cell | **5.9 - 14.4 ms** |
| advect 16^3, 8^3 velocity lattice | 4.4 - 10.8 ms |
| six 16x16 projections | 1.8 - 4.3 ms |
| two faces only, a display node | 0.6 - 1.4 ms |

At the 20 Hz field / 30 Hz display of section 2.3, that is **12-29% of one core** on the master and
**2-4%** on a display node.

### 13.2 Against PBF, for the same picture

The comparison that matters is not 512 particles -- it is the *full* beaker the reference image
shows. At `d = 2.5` that is 1,905 particles, and the solver's own measured `n^1.77` gives:

```
PBF, 1905 particles      732 ms/step  ->  1465 ms/frame  ->  0.68 fps
ink field, full volume   5.9 - 14.4 ms/step                  51x - 125x
```

The multiple is not the point. **The shape is**: PBF's cost grows with how full the vessel is, and
the field's does not. A full 16^3 field and an empty one cost the same, and that is the property
`MCU-REQUIREMENTS.md`'s "beaker needs 9.2x an S3" turns out to depend on.

### 13.3 What this means for the MCU question

An S3 can run the whole simulation. Whether one S3 can run the whole *cube* depends on panel
output, not on physics:

| one S3 doing everything | resolve + blit | total |
|---|---|---|
| six 32x32 faces | 3.4 + 6.7 ms | **47-72%** of a core -- fits |
| six 64x64 faces | 13.5 + 26.9 ms | **138-163%** -- does not fit |

So the three display boards remain necessary at 64x64, but for **panel bandwidth, not for
physics**. After this change the master is nearly idle and the cost concentrates in the blit --
~9.0 ms per display node per frame, untouched by anything in this document. That, and not the
choice of processor, is the next thing worth optimising.

A faster part helps less than the existing tables suggest. Compiling the kernel for RV32IMAFC and
Xtensa LX7 and applying `RESOURCES.md`'s own formula:

| workload | RV32/Xtensa instrs | P4 speedup |
|---|---|---|
| ink advection | 0.88 | **2.12x** |
| ink projection | 1.13 | **1.65x** |

Most of the P4's advantage over Xtensa was hardware float divide and square root -- `REQ-MCU-1` is
written the way it is for exactly that reason. This kernel uses neither by design, so the advantage
narrows to clock speed, on a workload already at 29% of an S3.

### 13.4 What none of this establishes

**Whether it looks right.** Every figure here is cost. The plume's coherence, whether tendrils
survive semi-Lagrangian diffusion, and whether a 16x16 projection upscaled 4x to a 64x64 panel
reads as ink -- none of that is measured, and none of it follows from the cost being low. Phase A
exists to answer it, and it remains the real risk in this proposal. The cost case was never the
doubtful part.

Two narrower caveats: the kernel timed here is the two hot loops, without active-bounds tracking,
injection, a surface or wire encoding; and the `n^1.77` extrapolation runs 3.7x past its measured
range, so 0.68 fps is an order of magnitude rather than a figure.
