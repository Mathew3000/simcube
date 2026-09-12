# How partsim works

The mathematics and the software structure, for someone who has to change either.

This is a description of what the code does, not of what it should do. Where a constant was
chosen by measurement the measurement is given, because those are the numbers that go stale
first. Where something is an approximation it says so.

Companion documents: [`DECISIONS.md`](DECISIONS.md) for why each choice was made,
[`RESOURCES.md`](RESOURCES.md) for the per-role memory and CPU budget and the
hardware measurements, [`CUBE-PCB.md`](CUBE-PCB.md) for the board.

---

## 1. What is being simulated

A closed cube of fluid, 32 world units on a side, seen from the inside by six LED panels that form
its walls. Tilting the cube pools the liquid; shaking it splashes. Three materials: water, sand,
and fire — the first two as particles, the third as a field, for reasons in §5.

The same C++17 core compiles to three targets:

| target | what it is | what it proves |
|---|---|---|
| host | `ctest`, benchmarks, PPM dumps | the physics, at desktop speed |
| WASM | three.js cube in a browser | what the hardware will look like |
| ESP32-S3 | the firmware | that it fits and runs |

All three run **bit-identical** physics. That is a design constraint, not a nicety — see §9.

### The world is 32 units, always

`kWorldSize = 32.0` is an invariant. Panel resolution is a *display* choice and the pitch is
derived from it:

```
pitch = kWorldSize / panelRes        32 texels -> 1.0,   64 texels -> 0.5
```

Two independent parameters would let someone request 64 texels at pitch 1.0 — a 64-unit world,
which was measured and rejected (its heat field alone is 155 KB). Deriving the pitch makes that
unrequestable. `tests/test_resolution.cpp` asserts the state hash is bit-identical at both
resolutions: the same fluid, sampled more finely.

---

## 2. The fluid solver — Position-Based Fluids

`core/src/Solver.cpp`, `core/include/partsim/Solver.h`.

Macklin & Müller's PBF. The choice over SPH is about stability at a large timestep: PBF solves a
density *constraint* on positions rather than integrating a pressure force, so it does not go
unstable when the timestep is too large for the stiffness — it just converges less far.

### One step

```
1. predict     v ← v + g·dt,  clamped to the CFL cap
               p* ← clamp_to_box(x + v·dt)
2. grid        counting sort on p*, then build neighbour lists          (§3)
3. iterate     × kSolverIterations:
                 a. density ρᵢ and Lagrange multiplier λᵢ    (one fused walk)
                 b. positional correction Δpᵢ, applied in place
                 c. granular friction, for materials with μ > 0
4. velocity    v ← (p* − x)·(1/dt)·damping,  with a sleep deadband
               x ← p*
5. viscosity   XSPH
```

### Kernels

The standard PBF pairing, with `h = kSmoothRadius = 2·d` where `d` is the rest spacing:

$$W_{\text{poly6}}(r) = \frac{315}{64\pi h^9}\,(h^2 - r^2)^3, \qquad
\nabla W_{\text{spiky}}(\mathbf{r}) = -\frac{45}{\pi h^6}\,(h-r)^2\,\frac{\mathbf{r}}{r}$$

Poly6 for density, Spiky for gradients — **not** interchangeable. Poly6's gradient vanishes as
`r → 0`, so two coincident particles would feel no restoring force and clump permanently. Spiky's
gradient is maximal there.

Both are evaluated on squared distance where possible (`poly6(r2)`) so the square root is only
taken when the gradient actually needs it.

### Mass, and why rest density is exactly 1

`Solver::init()` sums the poly6 kernel over a rest lattice of spacing `d` out to `kLatticeReach`
cells, and sets

$$m = \left(\sum_{i,j,k} W_{\text{poly6}}\!\left(\left|(i,j,k)\cdot d\right|\right)\right)^{-1}$$

so a particle sitting in a perfect rest lattice measures density exactly 1.0. This is not
cosmetic: it makes `kCfmEpsilon`, `kSCorrK` and every λ in the file **O(1) and independent of the
kernel's normalisation constant**. Change `h` and the constants still mean what they meant.

### Density constraint and λ

For each particle, the constraint is $C_i = \rho_i/\rho_{0,i} - 1$, with $\rho_{0,i}$ the
material's `restDensityScale` (water 1.0, sand 2.0 — which is *why* sand sinks, rather than a
special case in the gravity path).

$$\lambda_i = \frac{-C_i}{\sum_j \left|\nabla_{p_j} C_i\right|^2 + \varepsilon}$$

Two details in the code that matter:

- **Only compression is resolved.** If `C ≤ 0` the particle gets `λ = 0` and is skipped. Pulling a
  rarefied free surface back together is exactly what produces the clumping and tensile
  instability PBF is known for.
- **Density and the gradient sum are computed in one fused walk.** Separate passes would double
  the number of full neighbour scans, and those scans dominate the whole solver (§3).

`ε = kCfmEpsilon = 0.05` is constraint-force mixing — relaxation in the denominator. Sized against
the actual sum-of-squared-gradients, which is ~0.7 for a saturated neighbourhood here. Measured:
0.02 is stiffer and usable, 0.01 is unstable.

### Walls as missing density

A particle near a wall is missing every neighbour the wall displaces. Uncompensated, that deficit
reads as tension and the fluid **visibly shrinks away from the glass** — the worst possible
artefact when the glass is what you are looking at. Worse, under-compensating lets the fluid
over-pack against the floor while the measured density still reads 1.0.

Poly6 integrates in closed form. With $W = C(h^2-r^2)^3$, the marginal over a plane at offset $x$
is $M(x) = \tfrac{\pi C}{4}(h^2-x^2)^4$, so the fraction of the kernel's mass beyond a wall at
distance $d$ is

$$f(d) = \frac{\pi C}{4}\int_d^h (h^2 - x^2)^4\,dx$$

and the antiderivative of the expanded quartic is exact. Sanity checks, both asserted in
`tests/test_solver.cpp`: $f(0) = 0.5$ and $f(h) = 0$.

Baked into a 17-entry LUT at init. Applied per axis and summed, so a corner double-counts its
overlap slightly — accepted, and erring *dense* in corners is the safe direction.

### Artificial pressure

Macklin's `s_corr`, added to λ inside the correction:

$$s_{\text{corr}} = -k\left(\frac{W_{\text{poly6}}(r)}{W_{\text{poly6}}(\Delta q)}\right)^{n},
\qquad \Delta q = 0.2h,\; k = 5\times10^{-3},\; n = 4$$

Without it particles cluster into strings, and the effect is **worse at low iteration counts**,
which is exactly the regime this project runs in.

### The correction, and why it is Gauss-Seidel

$$\Delta \mathbf{p}_i = \frac{1}{\rho_0}\sum_j (\lambda_i + \lambda_j + s_{\text{corr}})\,
\nabla W_{\text{spiky}}(\mathbf{p}_i - \mathbf{p}_j)$$

Applied **in place** — later particles in the same pass see earlier corrections. Textbook PBF is
Jacobi, which needs a 3N delta buffer the ESP32 cannot spare. Gauss-Seidel is cheaper, converges
faster, and stays deterministic *because the particle order is a pure function of position* (§3).

This is also why the solver cannot be trivially split across the second core: both correction
passes write `setPred(i)` inside the neighbour loop, so splitting them changes the answer and
breaks the cross-target determinism claim.

Each iteration's correction is clamped to `kMaxDeltaP = 0.5·d`. This is the single most effective
guard against a blow-up: an over-large delta pushes a particle *through* its neighbours, which
raises the next density further still.

### Granular friction

What makes sand sand. A position-based friction projection, after Macklin's unified particle
physics: damp the **tangential** component of each contact pair's relative displacement over the
step. Both the displacement and the predicted position are available at that point, so it needs no
extra state.

Coulomb-bounded, and the bound is what makes it stable:

```
limit = μ · overlap
scale = (|t| ≤ limit) ? 1 : limit/|t|      static below the limit, kinetic at it
Δp   -= t · 0.5 · scale                    half, since j runs its own pass
```

Contacts are projected **sequentially**, each applied to `pred` before the next is read.
Accumulating bounded per-pair corrections and adding them at the end over-corrects by roughly the
neighbour count (~25×) and blows up; *averaging* them mostly cancels, because neighbours slide in
opposing directions, and the pile then spreads flat. Sequential projection is both stable and
effective.

Water has `friction = 0` and skips the pass entirely, so the scenes that set the frame budget pay
nothing for sand existing.

> **This pass is why 30 Hz physics was rejected.** The Coulomb bound is a *position* quantity
> while the sliding it resists grows with `dt`. At 60 Hz a sand heap holds at 5.65 units while
> water spreads flat at 0.00 — the defining difference between the materials. At 30 Hz the heap
> collapses to 0.00. Scaling the bound by the timestep ratio does not recover it (0.03).

### Velocity update and sleep

$$\mathbf{v} = \frac{\mathbf{p}^* - \mathbf{x}}{dt}\cdot 0.98$$

The 0.98 bleeds off the positional residual a finite iteration count leaves behind. Below
`kSleepSpeed` (or a material's higher `staticVelocity`) the velocity is zeroed outright — without
a generous deadband a sand pile creeps indefinitely and slowly flattens, which reads as the sand
*melting*.

### XSPH viscosity

$$\mathbf{v}_i \mathrel{+}= c\,m\sum_j (\mathbf{v}_j - \mathbf{v}_i)\,W_{\text{poly6}}(|\mathbf{p}_j - \mathbf{p}_i|^2)$$

In place, for the same memory reason as the correction pass. At `c = 0.02` the resulting slight
asymmetry in diffusion is invisible.

### CFL

Predicted velocity is capped at $0.4h/dt$. A particle that crosses more than a fraction of the
kernel radius in one step tunnels past the neighbours that were supposed to stop it.

`kGravityMag = 55` is tuned *against this cap*, not for physical realism: a full-height fall gives
$\sqrt{2g\cdot 32} = 59$ units/s against a cap of $0.4 \times 6.0 \times 60 = 72$.

> Note the coupling: the cap contains both `h` and `dt`. `h` is `2·kRestSpacing`, so **coarsening
> the particles without re-checking the CFL margin will start the fluid tunnelling.** It happens
> to be safe at the current 60 Hz / `d = 3.0`; it is not automatically safe.

---

## 3. Neighbour search

`core/include/partsim/SpatialHash.h`, `core/src/SpatialHash.cpp`.

This is the dominant cost of the whole simulation, so its structure is load-bearing.

### A uniform grid, not a hash

The container is a known AABB, so the bucket index is a direct `flatten(coordOf(p))` — no modulo,
no collisions, no probing. Cell size is `kCellSize = h`; a smaller cell would make the 27-cell
gather silently miss neighbours, so `SimVolume::build` rejects `cellSize < kSmoothRadius`
outright.

### Counting sort, not linked lists

`head[cell] + next[particle]` is two arrays and no sort, but the gather then chases pointers to
random addresses — roughly a cache miss per neighbour. Instead: counting sort, then **permute
every particle array into cell order**, so the gather streams almost linearly.

The permutation is a real side effect — particle indices are not stable across a step.

### Determinism comes from the iteration order

`forEachNeighbour` walks cells in ascending `flatten()` order and particles ascending within a
cell. Since the sort key is position and the permutation is deterministic, **the iteration order
is a pure function of the particle positions** — which is what lets a Gauss-Seidel solver be
bit-identical across three compilers and two instruction sets.

### The neighbour cache

The 27-cell gather scans a $3h$ box looking for an $h$-radius sphere, so most of what it touches
is out of range. Measured: **19.0 useful neighbours per 70.8 candidates, 27%**. The solver pays
that scan five times per step (2 iterations × {density, correction}, plus XSPH).

So the lists are built once, after the grid, and reused. Stored as a fixed stride of
`kMaxNeighbours = 64` `uint16` indices plus a `uint8` count.

**The naive version of this is a trap.** Freezing the neighbourhood at the start of the step does
not give a wrong answer — it gives a *slowly settling* one, because the correction pass moves
positions as it goes and the frozen list stops reflecting them. Measured: mean|v| 0.47 after 1500
steps against 0.053 uncached, and 4000 steps to rest instead of 2500. On a cube that is visible
shimmer.

The fix is a **margin**: cache to `1.15·h`, so the list also holds particles the correction pass
can pull inside. That restores 0.053 exactly. 1.3 is no better and costs 19% more.

This is not a proof. A single iteration's correction is clamped to `0.5·d`, so two particles could
in principle separate by `1.0·h` over two iterations and need a margin of 2.0. 1.15 covers what
the fluid actually does; if it ever does not, the failure is a marginally stale neighbourhood, and
the golden hashes on three targets would surface it as a divergence.

Sizing is measured, not guessed: worst case 50 neighbours across every scene, cap 64. Overflow
**truncates deterministically** — dropping an over-dense particle's furthest neighbours is a far
better failure than sizing the pool for a worst case that never occurs.
`SpatialHash::truncated()` reports it, and it is zero as configured.

Cost on hardware: **1.19× at 128 particles rising to 1.28× at 512**, for 64.5 KB.

Two passes deliberately keep their own live gather:

- **friction** — centred on `pos(i)` not `pred(i)`, with a contact radius of `d` not `h`. A
  different neighbourhood, and a smaller one.
- **XSPH** — by then `pos` holds the *corrected* predicted position, so the cached list is a step
  out of date in a way it never is inside the iteration loop.

---

## 4. Particle size is the cost lever

Filling a volume needs particles proportional to $1/d^3$. This is the strongest lever in the
project and everything else derives from it.

| `kRestSpacing` | particles for the same waterline | relative solver cost |
|---|---|---|
| 1.5 | 2427 | — |
| 2.25 | 719 | 0.12× |
| **3.0** | **303** | **0.05×** |

Three things must follow `d` or coarsening reads as a bug rather than a choice:

1. **`kSplatRadiusWorld`** — the blob has to be wider than the gap between particles or the fluid
   reads as separate dots. At `5/3·d` the settled crystal lattice is plainly visible looking down
   the bottom face; `2·d` hides it with the waterline still crisp; `7/3·d` hides it while visibly
   softening the waterline.
2. **`kSplatExposure`** — *derived*, not tuned. See §6.
3. **Scene and test fixture counts** — a count like `3000` or `settle(1500)` states a **fill
   level**, not a number. `particlesForFill()` rescales by $1/d^3$. Left alone, a scene silently
   becomes a function of the pool size via the capacity clamp, and a test fixture starts measuring
   an overfull box that cannot settle by construction.

The grid and heat-field cell counts also shrink with `d`, because `kCellSize = 2d` and the heat
cell is `d`.

---

## 5. Heat and fire

`core/include/partsim/FieldGrid.h`, `core/src/FieldGrid.cpp`.

**Fire is not particles, and that is the whole point.** It has no surface and no
incompressibility, so PBF buys nothing for it. What reads as flame is a buoyant scalar advecting
upward and cooling: one `uint8` per cell at half the sim resolution, a few KB and a single
semi-Lagrangian pass, against thousands of cycles per particle.

Water gets particles because its surface *is* the picture. Fire does not because it has none.

Per step, semi-Lagrangian advection along an analytic velocity:

```
velocity = buoyancy (opposite gravity) + container acceleration + a little swirl
value    = sample(cur, backtraced position) · cool
cool     = 1 − (1 − 0.03)·dt
```

Backtracing is subdivided, because buoyancy is fast relative to the cell size. Emitters are
injected **after** advection so they always show, even at high buoyancy, with a little flicker at
the base. The grid is ping-ponged, which is why it is the largest pool after the particles.

Using the same object-space gravity as the solver means flames lean when the cube is tilted and
get pressed around when it is shaken, with no extra plumbing.

`empty()` (peak == 0) lets `splatField` exit immediately when nothing is burning, which is most of
the render cost of a cold volume.

---

## 6. Rendering

`core/include/partsim/Geometry.h`, `core/src/Renderer.cpp`.

### Panels

A `Panel` is a baked frame: `origin` (the **corner** of texel 0,0), `u` and `v` (world step per
texel, orthogonal, both of length `pitch`), and inward normal `n`. World→panel is exact and
division-free because the inverse squared lengths are precomputed:

```
s = (P − origin)·u · invU2      texel x, fractional
t = (P − origin)·v · invV2      texel y
dist = (P − origin)·n           > 0 means inside the volume
```

**Orientation convention: panel +x is right and +y is up as seen from _outside_ the object.**
Choosing outside rather than inside means the emitted byte buffer is simultaneously correct for
LED scan order and for a three.js texture — no UV flip anywhere, the same bytes drive both.

### Depth-weighted splatting

Each particle is tested against every panel this node drives (at most 8, so a dot product each —
any acceleration structure would cost more than it saves, while also breaking the property that a
particle in a corner correctly lights three faces).

$$\text{attenuation}(d) = \left(1 - \frac{d}{D}\right)^2, \qquad D = \texttt{kSplatInfluence} = 8$$

Squared rather than linear because it reads more like light scattering through a translucent
medium. Heat uses a gentler and much longer falloff — `kHeatInfluence = 24`, since a plume in the
middle of a 32-unit cube is 16 units from every side face and at a reach of 8 only the floor could
see a campfire at all.

The radial kernel is a LUT over squared distance in texels, forced to zero at the rim so the
footprint is genuinely bounded. The **footprint is derived** as `kSplatRadiusWorld / pitch`, which
is what makes a particle a fixed *physical* size rather than a fixed number of texels.

Accumulation is `uint16` per channel, `(atten · kernel) >> 6`. The shift is 6 and not 8
deliberately: at `>> 8` a single particle's contribution maxes at 255 and the dim tail of the
falloff rounds to zero, truncating the outer glow.

### Three channels, resolved once

Accumulate per-**material intensity**; apply the palette once in `resolve()`. That is what makes
palettes pure data and scene crossfades a two-lookup lerp rather than a re-render.

| channel | content |
|---|---|
| 0 | water intensity |
| 1 | sand intensity |
| 2 | heat intensity |

### Exposure is derived, not tuned

A texel's accumulation is the sum over the particles inside the projected splat *column*, which
holds $\pi R^2 \cdot D / d^3$ particles. So accumulation goes as $R^2/d^3$, and

$$\texttt{kSplatExposure} = 7200 \cdot
\frac{R^2/d^3}{R_{\text{ref}}^2/d_{\text{ref}}^3}$$

written as a ratio against the reference configuration so it is *exactly* 7200 there. Held fixed
at 7200 instead, coarsening from `d = 1.5` to `3.0` measures mean luminance 59.7 → 13.4 while the
lit fraction stays at 40%: the same waterline, rendered too dark to read. Widen the blob and it
clips to white. Both are the same missing term.

---

## 7. Motion — object-space gravity

`core/include/partsim/MotionSource.h`, `core/src/MotionSource.cpp`.

The unifying abstraction: **the solver only ever sees a gravity vector in object space.** The
browser rotates the object and computes it directly; the device has an IMU reporting in the device
frame. Neither the solver nor the field knows which.

```
gravity_object = rotate_by_conjugate(orientation, (0, −g, 0))
effective      = gravity_object − container_acceleration
```

A shake is **one vector subtraction**, not a separate force path — container acceleration is
indistinguishable from gravity in the opposite direction, so it costs nothing to support.

### From the IMU

- `AxisMap` — a signed permutation, device axis → object axis. The *single* place the physical
  mounting is described. Every "why is gravity sideways" bug on a device like this comes from that
  knowledge being smeared across the firmware. A typo'd table that repeats an axis silently
  collapses one dimension of motion and looks like a flaky sensor, so `valid()` checks it at init.
- A complementary filter: gyro integration corrected toward the accelerometer at `alphaMax`. That
  is small **on purpose** — the accelerometer is the only absolute tilt reference, but during
  handling it is dominated by linear acceleration, so leaning on it hard enough to track a tilt in
  one frame also means every shake yanks "down" around.
- Accelerometer readings further than `trustWindowG` from 1 g are ignored entirely.
- A high-pass at `shakeCornerHz` separates shake from residual tilt error; the shake term becomes
  container acceleration.
- Gyro bias is tracked only while genuinely still (`stillGyro` **and** `stillAccelG` both).

---

## 8. Fixed timestep and substeps

```
accumulator += wallDt                     (clamped to 0.25 s)
while accumulator ≥ kFixedDt and n < kMaxSubsteps:
    fixedStep(kFixedDt); accumulator −= kFixedDt; n++
if n == kMaxSubsteps: accumulator = 0     drop the backlog
```

The backlog is **dropped rather than accrued**: catching up would make a slow frame trigger even
more work on the next one. The consequence is that `advance()` makes the step count a function of
frame timing — which is why a replication master must use `stepFixed()` directly, since its step
index is the authoritative clock (§10).

`kFixedDt = 1/60` and the firmware displays at 30 Hz, so it runs **two physics steps per displayed
frame**.

Rendering may extrapolate along particle velocity by `timeOffset` to cover unconsumed accumulator
time. Only the *splat* moves — the simulation state is untouched, so it can never feed back.

---

## 9. Determinism

Three targets must produce **bit-identical** particle state. Not approximately: the golden test
compares a 32-bit hash.

What it costs:

- **Fixed timestep.** No variable `dt` anywhere in the physics.
- **`-ffp-contract=off` on every target.** Otherwise Xtensa fuses `a*b+c` into a single MADD with
  different rounding to WASM's separate multiply and add, and trajectories diverge within a few
  hundred steps.
- **No libm transcendentals in `core/`.** `Math.h` carries a polynomial `fsin`/`fcos`/`fexp`;
  `scripts/check_no_libm.sh` is a ctest entry that enforces it.
- **`xoshiro128**`**, seeded explicitly, never the platform RNG.
- **Deterministic iteration order** everywhere, including the neighbour gather (§3).
- `-Wdouble-promotion` as an error: a stray `0.5` or `sqrt()` promotes to double, which the S3
  emulates in software and which silently halves the framerate.

What it buys: the browser is a *prediction* of the hardware rather than an illustration of it, and
a divergence is a real signal instead of noise. Four ctest entries check it — host, WASM, emulated
Xtensa under QEMU, and the device-capacity profile.

`stateHash()` covers positions and velocities. A separate pixel hash covers the render, because
resolution is display-only and the state hash must survive a resolution change untouched.

---

## 10. Multi-node

Six 64×64 panels need 288 KB of DMA buffer against 320 KB of total DRAM, so one S3 cannot drive
them. The work splits into one **simulation master** plus three **display nodes** of two faces
each.

**One simulation, many drawers.** Divergence is impossible by construction; the failure mode is
**staleness** — a dropped frame freezes one face while the others move. That is what the browser's
fault injection exists to make visible.

### The seam

`RenderState.h` — `ParticleView` / `HeatView` are read-only views over raw arrays, and
`RenderParticles` / `HeatBuffer` are the draw-only containers. A display node carries these instead
of a `Simulation`. That is not an optimisation: with the full `Simulation` a node measures 244.6 KB
against a 230 KB ceiling, and all of the overshoot is state it never uses (predicted positions,
λ, the second field buffer, the neighbour grid, the sort scratch).

`heatGridDim()` / `heatCellSize()` are shared by `FieldGrid` and `HeatBuffer` so the two cannot
disagree about the grid they are describing.

### The wire format

`core/src/SimFrame.cpp` — in `core/` precisely so all three targets share one implementation.

| field | size |
|---|---|
| header | 20 B — magic, version, flags, step, counts, **geometry hash**, palette A/B, blend, Fletcher-16 |
| per particle | 10 B — 3×`uint16` position, 3×`int8` velocity, 1 B material |
| heat field | PackBits RLE — zero bytes for a water-only scene |

20,985 B/frame, ~630 KB/s at 30 fps. Particles rather than pixels: six 64x64 faces of finished
RGB is 73,728 B/frame *and* would put all six faces' splat cost back on the master.

The **geometry hash** rejects mismatched builds, and `decodeFrame` validates everything before
touching any state. Broadcast works by asserting all three chip selects at once — every node needs
every particle, since a particle can light any face.

Display nodes render from **quantised** positions, and the browser preview quantises too, or it
stops predicting what the hardware shows.

### Render sets

Every node holds the full six-panel table — `Geometry::bounds()` must yield an identical container
everywhere — while allocating accumulation buffers only for the faces it physically drives.
`Renderer::init(g, panels, count)` takes a render set and indexes `accum_` by **slot**. At 64x64 an
accumulator is 64x64x3 channels x 2 B = 24,576 B, so the difference between 6 slots and 2 is 96 KB
on a node with 230 KB to spend.

---

## 11. Source map

```
core/                        portable, no platform headers, no libm transcendentals
  Config.h                   every capacity and tuning constant, with its justification
  Math.h  Types.h  Rng.h     vectors, quaternions, polynomial transcendentals, xoshiro128**
  Particles.h                SoA pools; O(1) swap-with-last removal
  Geometry.{h,cpp}           panels, cube()/slab(), world<->panel projection
  SimVolume.{h,cpp}          container AABB + uniform grid dimensions
  SpatialHash.{h,cpp}        counting sort, 27-cell gather, neighbour cache
  Solver.{h,cpp}             PBF: kernels, wall density, lambda, correction, friction, XSPH
  FieldGrid.{h,cpp}          heat/smoke, semi-Lagrangian
  Renderer.{h,cpp}           splat, accumulate, resolve through palettes
  Palette.h PaletteData.cpp  colour ramps as tables
  Scene.h ScenePresets.cpp   scene presets as tables
  MotionSource.{h,cpp}       IMU fusion, axis mapping, shake extraction
  Simulation.{h,cpp}         the façade: owns everything, fixed-step loop, transitions
  RenderState.{h,cpp}        the draw/run seam for display nodes
  SimFrame.{h,cpp}           wire format
  ChainMap.{h,cpp}           face -> HUB75 chain position, with rotation and mirroring

platform/app/                App.{h,cpp} -- the firmware minus the hardware and the scheduler
                             Console, Clock, Display, MotionSensor, FrameLink, Role
platform/host/               golden.cpp, bench.cpp, ppm_dump.cpp, memreport.cpp, dutyreport.cpp
                             console_main.cpp -- platform/app on a second platform
platform/wasm/               bindings.cpp + web/ (three.js cube, multi-node preview)
platform/esp32/              main.cpp (bring-up + scheduling), Pins.h, RoleStraps.{h,cpp},
                             PanelDriver, Lsm6dsox, SpiFrameLink, platformio.ini
tests/                       153 cases; 7 ctest entries by default, 8 with the opt-in QEMU check
scripts/                     build, budget, determinism and QEMU checks
```

`Simulation` is a façade over the parts, not a god object — each piece above is independently
constructed and independently tested. It owns the fixed-step loop, scene transitions (population
drains and refills gradually, at a rate that scales with `d` so the *duration* is what stays
fixed), and the palette crossfade.

---

## 12. Where the time actually goes

Measured on an ESP32-S3 devkit at 240 MHz, current configuration. See `RESOURCES.md` §5.1 for the
QEMU conversion factors and the full derivation.

| particles | sim/step | splat | resolve | blit | frame | fps |
|---|---|---|---|---|---|---|
| 128 | 8.06 | 5.59 | 2.60 | 5.96 | 27.67 | 36.1 |
| 256 | 26.14 | 11.50 | 2.89 | 6.25 | 70.02 | 14.3 |
| 384 | 49.05 | 16.20 | 3.13 | 6.49 | 120.78 | 8.3 |
| 512 | 74.65 | 19.59 | 3.37 | 6.73 | 175.63 | 5.7 |

Solver cost scales as $n^{1.57}$ — superlinear because the gather widens as density rises.

**`blit` contains `resolve`.** `present()` resolves each face into the staging buffer and then
pushes it, while the `resolve` column times that work again on its own. A frame is
`sim × substeps + splat + blit`, which is what the `frame` column computes; adding `resolve` to it
counts resolve twice. Earlier revisions of this section did, and the floor below is ~3 ms lower
than the figure it used to quote for that reason.

**The bottleneck has moved.** Splat + blit is a **22.69 ms floor at 384 particles that no processor
removes**, capping the frame at 44.1 fps however fast the solver gets. Widening the blob to hide the
coarse lattice (§4) is what put it there; it bought the look and it moved the bottleneck. An
ESP32-P4 is ~2.3× on the solver (`CUBE-PCB.md` §6.2, from a measured instruction-count ratio)
and therefore buys a few fps.

The floor was 28.67 ms until the two software levers named here were built — the circular splat
bound and the row-walking blit, `DECISIONS.md` P3. **Splat is now three quarters of what remains**,
and the cheap texels have already gone: what is left is the texels that genuinely contribute (F5).
The levers on that are the blob radius, which is a look decision (§4, F3), and split-kernel chroma
(P1) — not tighter loop bounds.
