# Design decisions

Why the project is the way it is. One entry per decision that was not obvious, with the evidence
that settled it and — where it exists — the reasoning that turned out to be wrong.

**Read the reversals first (§9).** Six decisions in this project were made on plausible reasoning
and later overturned by measurement. Those are the most useful entries here, because each one is a
pattern that will recur.

Status markers: **[STANDS]** current and load-bearing · **[REVERSED]** overturned, see §9 ·
**[USER]** the user's call, recorded so it is not silently relitigated · **[OPEN]** proposed, not
built.

Companions: [`SIMULATION.md`](SIMULATION.md) for how it works, [`RESOURCES.md`](RESOURCES.md) for
the budget and the measurements, [`CUBE-PCB.md`](CUBE-PCB.md) for the board.

---

## 1. Shape of the project

### D1. One C++17 core, three targets — host, WASM, ESP32 **[STANDS]**

The physics is consumed by the firmware through a symlink, not copied. There is exactly one copy
of the simulation in the repository.

Consequence, and the reason it is worth the constraint: the browser is a **prediction** of the
hardware rather than an illustration of it. When something looks wrong in the browser it is wrong
on the device.

### D2. Bit-identical physics on all three targets **[STANDS]**

Not approximately identical — a 32-bit state hash compared across host, WASM and emulated Xtensa.

What it costs: fixed timestep everywhere; `-ffp-contract=off` on every target; no libm
transcendentals in `core/` (there is a polynomial `fsin`/`fcos`/`fexp` and a ctest that enforces
it); an explicitly seeded xoshiro128\*\* rather than any platform RNG; deterministic iteration
order in the neighbour gather; `-Wdouble-promotion` as an error.

What it buys: a divergence is a real signal instead of noise. This paid for itself repeatedly —
the golden hash is what catches an accidental physics change inside a refactor that was supposed
to be render-only.

`-ffp-contract=off` is not paranoia. Without it Xtensa fuses `a*b+c` into a single MADD with
different rounding to WASM's separate multiply and add, and trajectories diverge within a few
hundred steps.

### D3. The plan file lives outside the repository **[USER]**

`~/.claude/plans/pure-yawning-eclipse.md` is not in git. Raised at the time; the user's call was
"it's okay as long as it's documented". This file and the three companions are that documentation.

---

## 2. The world model

### D4. The world is 32 units on a side, always **[STANDS]**

`kWorldSize` is an invariant, and the panel pitch is **derived** from the resolution
(`pitch = kWorldSize / panelRes`) rather than passed alongside it.

Two independent parameters would let someone request 64 texels at pitch 1.0 — a 64-unit world,
which was measured and rejected because its heat field alone is 155 KB. Deriving the pitch makes
the bad combination unrequestable rather than merely discouraged.

### D5. Resolution is a display concern only **[STANDS]** **[USER]**

32 texels at pitch 1.0 and 64 at pitch 0.5 describe the same 32-unit world. The user chose this
over a larger world when the cube moved to 64×64 panels. `tests/test_resolution.cpp` asserts the
state hash is bit-identical at both — that assertion is what makes the claim true rather than
intended.

### D6. Object-space gravity is the unifying abstraction **[STANDS]**

The solver only ever sees a gravity vector in object space. The browser rotates the object and
computes it; the device has an IMU reporting in the device frame. Neither the solver nor the heat
field knows which.

Container acceleration is indistinguishable from gravity in the opposite direction, so a shake is
**one vector subtraction**, not a separate force path.

### D7. Panel orientation is defined from OUTSIDE the object **[STANDS]**

Panel +x is right and +y is up as seen by an observer standing outside that face.

Choosing outside rather than inside means the emitted byte buffer is simultaneously correct for
LED scan order and for a three.js texture. No UV flip anywhere; the same bytes drive both.

### D8. The IMU mounting lives in exactly one signed-permutation table **[STANDS]**

`AxisMap`. Every "why is gravity sideways" bug on a device like this comes from that knowledge
being smeared across the firmware. A typo'd table that repeats an axis silently collapses one
dimension of motion and reads as a flaky sensor, so `valid()` checks it at init.

---

## 3. The solver

### D9. Position-Based Fluids, not SPH **[STANDS]**

PBF solves a density *constraint* on positions rather than integrating a pressure force, so it
does not go unstable when the timestep is too large for the stiffness — it converges less far
instead. On a fixed 60 Hz budget with two iterations, that failure mode is the right one.

### D10. Poly6 for density, Spiky for gradients **[STANDS]**

Not interchangeable. Poly6's gradient vanishes as `r → 0`, so two coincident particles would feel
no restoring force and clump permanently.

### D11. Particle mass derived from a rest lattice **[STANDS]**

Mass is set so a perfect rest lattice measures density exactly 1.0. This keeps `kCfmEpsilon`,
`kSCorrK` and every λ **O(1) and independent of the kernel's normalisation constant** — change `h`
and the constants still mean what they meant. Without it every tuning constant would silently
depend on the smoothing radius.

### D12. Only compression is resolved **[STANDS]**

`C ≤ 0` gives `λ = 0`. Pulling a rarefied free surface back together is precisely what produces
the clumping and tensile instability PBF is known for.

### D13. The wall term is a closed-form integral, not a fudge **[STANDS]**

Poly6 integrates exactly, so the fraction of kernel mass beyond a wall is an exact quartic
antiderivative, baked into a 17-entry LUT.

Uncompensated, the deficit near a wall reads as tension and the fluid **visibly shrinks away from
the glass** — the worst possible artefact when the glass is what you are looking at. Worse,
*under*-compensating lets the fluid over-pack against the floor while measured density still reads
1.0, which is a bug that hides itself.

### D14. Gauss-Seidel, not Jacobi **[STANDS]**

Corrections are applied in place. Textbook PBF is Jacobi, which needs a 3N delta buffer the ESP32
cannot spare. Gauss-Seidel is cheaper, converges faster, and stays deterministic *because the
particle order is a pure function of position* (D17).

The cost is recorded honestly: it is why the solver cannot be naively split across a second core.
See P2 for the way out that does not break determinism.

### D15. Granular friction is Coulomb-bounded and projected sequentially **[STANDS]**

Each pair's correction is capped at `μ × overlap`, and contacts are applied one at a time to
`pred` rather than accumulated.

Both halves were arrived at by failure. Accumulating bounded corrections and adding them at the end
over-corrects by roughly the neighbour count (~25×) and blows up. Averaging them instead mostly
cancels — neighbours slide in opposing directions — and the pile spreads flat. Unbounded
cancellation of the tangential slide is stable only for `μ ≤ 1`, where it barely affects the pile,
and injects energy above that.

### D16. Two solver iterations **[STANDS]**

2 beat 3 on measurement: mean speed 0.03 vs 0.30 after 500 steps at 3000 particles, and 28% less
cost. More iterations are not better here. 1 was tried much later and rejected — see D24.

---

## 4. Neighbour search

### D17. Counting sort with a full SoA permutation, not linked-list buckets **[STANDS]**

`head[cell] + next[particle]` is two arrays and no sort, but the gather then chases pointers to
random addresses — roughly a cache miss per neighbour. Counting sort plus permuting every particle
array into cell order makes the gather stream almost linearly.

The permutation also gives D2 its foundation: iteration order becomes a pure function of position.

### D18. Neighbour lists ARE cached, with a 1.15× margin **[REVERSED then re-decided]**

Originally rejected (M3 plan) at 384 KB — a figure for 4096 particles. Hardware put the real
budget in the hundreds, where the same structure is a tenth of the size. See §9.

The margin is the part worth remembering. A naive cache freezes the neighbourhood at the start of
the step, and because the correction pass moves positions as it goes, that does not give a *wrong*
answer — it gives a **slowly settling** one. Measured: mean|v| 0.47 after 1500 steps against 0.053
uncached, 4000 steps to rest instead of 2500. On a cube that is visible shimmer. Caching to 1.15×h
so the list also holds particles the correction can pull inside restores it exactly; 1.3 is no
better and costs 19% more.

Not a proof. A single iteration's correction is clamped to `0.5·d`, so two particles could in
principle separate by `1.0·h` over two iterations. 1.15 covers what the fluid actually does, and if
it ever does not, the golden hashes surface it as a divergence.

Sized on data: worst case 50 neighbours across every scene, cap 64, overflow truncates
deterministically and is reported. Cost on hardware 1.19× at 128 particles to 1.28× at 512, for
64.5 KB.

### D19. Friction and XSPH keep their own live gathers **[STANDS]**

Friction is centred on `pos(i)` with a contact radius of `d` — a different, smaller neighbourhood.
XSPH runs after `pos` has been overwritten with the corrected position, so the cached list is a
step out of date there in a way it never is inside the iteration loop.

---

## 5. Particle size — the dominant cost lever

### D20. `kRestSpacing` 1.5 → 3.0 **[STANDS]**

Filling a volume needs particles proportional to `1/d³`. Coarsening 1.5 → 3.0 took the water tank
from 3000 particles to 375 **at the same waterline** (lit fraction 40.0% → 42.9%).

This is the strongest lever in the project and it was only found after hardware showed that ~211
particles fit 30 fps where the plan had assumed ~1300.

### D21. Three constants must follow the spacing **[STANDS]**

Each was found by it breaking:

- **`kSplatRadiusWorld`** — coupled to `d`, at 2×. Left absolute at 2.5 the particles stopped
  overlapping and the fluid read as a grid of dots. At `5/3·d` the settled crystal lattice is
  plainly visible down the bottom face; 2× hides it with the waterline crisp; `7/3·d` hides it
  while visibly softening the waterline.
- **`kSplatExposure`** — now *derived*, not tuned. Accumulation is the particle count in the
  projected splat column, which goes as `R²/d³`. Held at 7200, the same waterline rendered at mean
  luminance 13.4 instead of 59.7 while the lit fraction stayed at 40%.
- **Scene and fixture counts** — `particlesForFill()`. A count like `3000` or `settle(1500)` states
  a **fill level**, not a number. Left literal, a scene becomes a function of the pool size via the
  capacity clamp, and a test starts measuring an overfull box that cannot settle by construction.

### D22. Exact fractions, multiply-then-divide **[STANDS]**

`kRestSpacing * 5 / 3` is exactly 2.5 in float. `1.6667f * 1.5f` is 2.50005 and quietly moved the
golden pixel hash. Small, and it cost an hour.

### D23. Coarsening stops at 3.0 **[STANDS]**

Rendered as a ladder at a fixed waterline: 3.5 is still clean, 4.0 is borderline, 4.5 and 5.0 are
visibly blobs rather than fluid. Further coarsening buys frames by destroying the thing being
displayed.

Beaker mode tightens this further — see F3.

---

## 6. Timestep

### D24. 60 Hz physics, two steps per displayed frame **[STANDS — after a failed attempt to change it]**

30 Hz was tried and is 2.08× on water for an invisible cost: rendered at equal simulated time the
waterline sits at the same height and the lit fraction moves 43.9% → 42.4%.

**It costs the sand material.** Granular friction is bounded by `μ × overlap` — a *position*
quantity — while the sliding it resists grows with `dt`. At 60 Hz the heap holds at 5.65 while
water spreads flat at 0.00; at 30 Hz the heap collapses to 0.00 and sand is a heavy liquid.
Scaling the Coulomb bound by the timestep ratio does not recover it (0.03).

Dropping to one iteration at 60 Hz collapses the heap the same way for only 1.11×, so the plan's
assumed "safe fallback" is worse on both axes.

Compression is partly recoverable by retuning `kCfmEpsilon` 0.05 → 0.02 (0.01 is unstable), and
*not* recoverable with more iterations — four still leaves rho at 1.036, so it is the timestep.

`PARTSIM_FIXED_DT_DEN` is left as an override. **Beaker mode is liquid-only and has no sand to
lose**, so the 2.08× is available there. That makes this an M4 decision rather than a dead end.

### D25. The substep backlog is dropped, not accrued **[STANDS]**

`advance()` drops the accumulator at `kMaxSubsteps`. Catching up would make a slow frame trigger
more work on the next one.

Consequence: step count becomes a function of frame timing, which is why a replication master must
call `stepFixed()` directly — its step index is the authoritative clock.

---

## 7. Rendering

### D26. Accumulate per-material intensity; apply the palette once **[STANDS]**

Three `uint16` channels — water, sand, heat — resolved through palette ramps in `resolve()`. That
is what makes palettes pure data and a scene crossfade a two-lookup lerp rather than a re-render.

### D27. Heat is a field, not particles **[STANDS]**

Fire has no surface and no incompressibility, so PBF buys nothing for it. One `uint8` per cell at
half the sim resolution and a single semi-Lagrangian pass, against thousands of cycles per
particle. Water gets particles because its surface *is* the picture; fire does not because it has
none.

### D28. Accumulation shifts by 6, not 8 **[STANDS]**

At `>>8` a single particle's contribution maxes at 255 and the dim tail of the falloff rounds to
zero, truncating the outer glow. Two extra bits keep the tail and a dense texel still only reaches
~6500 of the 65535 a `uint16` holds.

### D29. Splat footprint is derived from the pitch **[STANDS]**

A particle is a physical thing, so its apparent size must not change when the panel resolution
does. Measured: peak per-texel accumulation is pitch-invariant (4888 → 4853, 0.99×), which
retired a whole predicted retuning loop.

---

## 8. Multi-node

### D30. One authoritative simulation; display nodes only draw **[USER]** **[STANDS]**

Divergence becomes impossible by construction. The failure mode is **staleness** — a dropped frame
freezes one face while the others move — which is what the browser's fault injection exists to
make visible.

### D31. Particles on the wire, not pixels **[STANDS]**

20,985 B/frame against 73,728 B for six 64×64 faces of finished RGB — and sending pixels would
also put all six faces' splat cost back on the master.

Broadcast by asserting all three chip selects at once: every node needs every particle, since a
particle can light any face.

### D32. The draw/run seam is required, not an optimisation **[STANDS]**

`RenderState` exists because a display node carrying a full `Simulation` measures **244.6 KB
against a 230 KB ceiling**, and all of the overshoot is state it never uses — predicted positions,
λ, the second field buffer, the neighbour grid, the sort scratch.

### D33. Role from strapping pins **[STANDS]**

Two pins give master plus three display IDs, so every board is flashed identically and the role is
a fact about the wiring. Standalone is a *build* environment, not a runtime role.

### D34. Radio on the master only **[STANDS]**

M2 turned WiFi/BT off for ~55 KB of heap and ISR jitter against a 16 MHz display clock. The master
drives no panels and has the memory, so ESP-NOW for beaker chaining lives there. Display nodes stay
radio-silent — they are also the boards inside the panel stack, where HUB75 EMF is worst.

### D35. The master/display split survives, but not for the reason it was built **[REVERSED rationale]**

See §9. The performance argument is gone; the memory and blit arguments stand on their own.

---

## 9. Decisions that were reversed

The six most useful entries in this file.

### R1. "One S3 cannot drive six 64×64 panels"

**Wrong as stated.** The user challenged it directly. `SPIRAM_DMA_BUFFER` exists and works; only
the *internal-SRAM* configuration is impossible. The claim conflated "does not fit internal SRAM"
with "cannot be done".

Prerequisite if it is ever used: a row-walking blit. `drawPixelRGB888` does a read-modify-write per
bitplane, which is 147,456 scattered PSRAM accesses per frame at six faces, and scattered access is
the one pattern PSRAM cannot absorb.

### R2. "QEMU's timing is not trustworthy"

Recorded on the strength of a bottom-up cycle estimate that disagreed with it by 7.4×. **Hardware
showed the estimate was the wrong one** — icount was within 28%, the bottom-up figure was 9.4× out.

Worse, the conclusion was drawn from a disagreement between two unmeasured quantities and then
written down as fact. The real factors, measured across a 4× range of particle count: QEMU
wall-clock × 3.0 on this laptop, QEMU icount × 1.47 on any machine — and the second is *derived*
(icount assumes 250 MHz at 1 IPC; the part is 240 MHz at a measured CPI of 1.40).

### R3. "~1300 particles at 30 fps"

The plan's budget, from a bottom-up estimate of ~5500 cycles per particle per step. The device
needs ~51,900. The real figure is **~211 particles**, and cost scales `n^1.57`, not linearly.

The estimate assumed ~33 neighbours. Instrumentation measured **88.3 candidates gathered for 23.1
useful ones** — the 27-cell gather scans a box and 74% of what it touches is outside the radius.

### R4. "The master/display split is a performance decision"

At the assumed particle count, splat was 3.2% of the frame, so the split looked like it moved most
of the work. Measured, **the solver is 95.8% of the frame** and the split moves 4%.

The split still ships — the memory ceiling (D32) and the 64×64 blit cost (41 ms for six faces on
one board against 10.3 ms for two) are sufficient on their own. But the stated reason was wrong,
and a reason that is wrong will be reused.

### R5. "Neighbour caching costs 384 KB"

True for 4096 particles. That particle count was itself a consequence of R3 and was never
achievable. At the counts that actually run it is 64.5 KB and it is the largest single lever.

**The pattern: a rejection inherited from an invalidated premise is not a decision, it is a
leftover.** Worth re-auditing the others whenever a foundational number moves.

### R6. "RV32 needs 1.0–1.2× more instructions than Xtensa"

Written into the resource document as an assumption while estimating the ESP32-P4. Measurable, and
measured: `core/` is freestanding portable C++, so the solver compiles for every candidate ISA and
the instruction count can simply be read off. **RV32 needs 25% fewer, Thumb-2 28% fewer.**

That first measurement was itself flattering ARM and RISC-V, because the Xtensa build was *calling
out* to `__divsf3` and `sqrtf`, whose instructions live in the library and were missing from the
count. With `frsqrt` inline everywhere the like-for-like figure is **0.81× for both**.

---

## 10. Findings that changed the engineering

### F1. The ESP32-S3's FPU has neither divide nor square root

`nm -u` on the solver object shows `__divsf3` and `sqrtf` as unresolved externals on Xtensa and
neither on Cortex-M7, which has `vdiv.f32` and `vsqrt.f32` as single instructions. The inner loop
made ~260 such calls per particle per step.

Fixed by deriving both quantities from one Newton-Raphson reciprocal square root (multiplies and
adds only) and hoisting a loop-invariant reciprocal. **Measured 1.18×** — well short of the 38–69%
the call count suggested, because the newlib routines cost ~20–35 instructions rather than 70–90.

`frsqrt` is also *better for determinism* than the hardware it replaces: it does not depend on
three libm implementations agreeing. But `prsqrt` deliberately stays exact — routed through
`frsqrt` it put `Geometry::cube`'s panel origins at −15.9999981 instead of −16. 1e-7 is fine for a
kernel gradient and not for geometry setup, which happens once.

### F2. The render floor caps the frame regardless of MCU

Splat + resolve + blit is ~30.7 ms at 375 particles, and no processor removes it. Even a free
solver gives 32.6 fps. Widening the blob to hide the coarse lattice (D21) is what put it there: it
bought the look and it moved the bottleneck.

Caveat on how this was stated earlier: at 375 particles the *solver* is still 84% of the frame, so
the floor is a **ceiling**, not the current bottleneck. It becomes the binding constraint at the
coarser spacings, where it is over half the frame.

### F3. Colour resolution is set by the blob, not by the particle

Chroma splats through the same kernel as brightness, so distinguishable colour regions across the
cube is about `32/(2·d)` — **5.3 regions at the shipping spacing**. Red pouring into blue would
read as a few coloured lumps converging, not as mixing.

This is the constraint that pulls particle size opposite to frame rate, and it is why D23 cannot
simply be relaxed. It is the user's stated concern and it is correct.

### F4. Test fixtures encode fill levels, and a chaotic threshold is not a test

Two related lessons.

Fixtures written as literal counts silently changed meaning when the spacing changed —
`settle(1500)` overfills a 32-unit box at `d = 3.0`, so "the solver never reaches hydrostatic rest"
was a stale constant rather than physics. Density fixtures now state a pool **depth in smoothing
radii**, because bulk density is meaningless in a pool shallower than one radius.

And a settle threshold at 1500 steps read anywhere between 0.05 and 0.85 depending on trajectory —
hoisting a single reciprocal out of a loop moved it from 0.053 to 0.568 without touching the
physics. **A threshold a rounding change can cross is measuring chaos, not convergence.** Moved to
2500 steps, past the knee for every variant tried.

---

## 11. Open proposals

Not built. Recorded so the reasoning is not re-derived.

### P1. Split-kernel chroma — likely removes the fine-particle requirement **[OPEN]**

Brightness and colour do not have to share a splat kernel. Splat brightness at `2d` (needed for a
continuous surface) and chroma at `d`, resolving colour as the ratio of the two narrow channels.
Colour resolution goes from `32/(2d)` to `32/d` — **double, at the same particle count**.

d=3.0 with a split kernel gives the colour resolution that otherwise needs d=1.5, which is 8× the
particles. Costs one narrow accumulation pass, cheaper than the main splat because the footprint is
a quarter of the area.

Risk to check on a render before believing it: colour may look detached from the fluid's brightness
at the surface.

### P2. Eight-colour cell partitioning — parallelism without breaking determinism **[OPEN]**

Colour each grid cell by `(x&1, y&1, z&1)`. Process colours sequentially; within a colour, all
particles in parallel.

Correctness: cell size is exactly `h`, so two same-colour cells are ≥2 cells apart and their
closest particles are ≥`h` apart, where the kernel is exactly zero. A particle reads only its 27
surrounding cells, and a same-colour cell is never among them. Each particle writes only its own
position. **Writes are disjoint and no thread reads a position another is writing** — no locks, no
atomics.

Deterministic: fixed colour order, and within a colour the order is irrelevant because there are no
interactions. The result differs from today's sequential order, so the hash moves once.

Worth ~1.8× on the S3's idle second core, and it applies to every candidate MCU — roughly a tier of
silicon.

### P3. Row-walking blit and a circular splat bound **[OPEN]**

The two halves of F2. `ChainMap` already has a `ChainRun` escape hatch designed for the blit and
never written; `drawPixelRGB888` does a read-modify-write per bitplane. The splat scans a square
box for a circular kernel, so ~21% of touched texels can never contribute.

### P4. MCU selection **[OPEN]**

Solver throughput relative to the S3, from the measured ISA ratio (0.81) and CPI (1.40): RP2350
0.73×, ESP32-P4 2.3×, STM32H743 2.9×, STM32H7S3 3.6×, i.MX RT1062 3.8×, i.MX RT1176 6.3×.

CPI is the weak term and it decides the answer. Cortex-M7 wins largely on **tightly-coupled
memory** — the working set is 86–170 KB and fits the DTCM of every M7 listed. What does *not* help:
SIMD (the solver is scalar and Gauss-Seidel resists vectorisation), a second core without P2, large
flash, PSRAM bandwidth.

Not a Linux part: i.MX RT is a crossover MCU, bare metal or FreeRTOS.

Recommended order before committing to silicon: land P2 and P1 first, because they change the
requirement by more than a tier of chip changes the supply.

---

## 12. Hardware and tooling

### D36. `-DPARTSIM_PROFILE_*` is one flag; the numbers live in `Config.h` **[STANDS]**

Spelling capacities out in `platformio.ini` would mean the host verification and the firmware each
carry their own copy of the budget, and the first time one was edited the checks would quietly
start measuring a configuration nobody ships.

### D37. N16R8 devkits, and the COM socket for flashing **[STANDS]**

N8R2 has *quad* PSRAM — different memory type, different GPIO exclusions, roughly half the
bandwidth. Benchmark the part you intend to ship.

A DevKitC-1's two USB-C sockets are not interchangeable. **COM** is a CH343 bridge with hardware
DTR/RTS auto-reset and always works. **USB** is the S3's native port and only works if the running
firmware cooperates — the factory RGB demo does not, and esptool reports "No serial data received"
while the board is perfectly alive. Cost an hour to discover.

### D38. `SPIRAM_DMA_BUFFER` is off at 32×32, and that is narrower than "never" **[STANDS]**

72 KB of framebuffer fits internal SRAM with room to spare, so PSRAM would buy nothing and cost
bandwidth. At 64×64 the calculation reverses: 288 KB does not fit at all, and the flag becomes what
makes a single-board six-face cube possible. See R1.

### D39. No Adafruit_GFX **[STANDS]**

`VirtualMatrixPanel` cannot express a cube, so the firmware drives the raw canvas and maps faces
itself. That makes the GFX primitives, font tables and the whole dependency chain dead weight.

### D40. QEMU is a milestone, with its limits written down **[STANDS]**

It verifies boot, FreeRTOS task creation and pinning, stack sufficiency, alignment, software-float
promotions, the console, and — the reason it exists — the third leg of the determinism check on
real Xtensa codegen.

It cannot verify the display (no LCD_CAM/GDMA model), the IMU, or PSRAM bandwidth. Timing it can
now do, within ~30%, once corrected by the factors in R2 — which is the opposite of what this
project believed before hardware arrived.

### D42. The application layer is a library, and the host is its second platform **[STANDS]**

`main.cpp` was 903 lines with `Serial`, `Wire`, FreeRTOS and HUB75 inline. It is now 332 — drivers
constructed, tasks started — and everything else is `partsim::app::App` in `platform/app`, behind
`Console`, `Clock`, `Display`, `MotionSensor`, `FrameLink` and `SystemHooks`.

Three choices inside that are worth recording, because each had an obvious alternative:

**App owns a frame; the platform owns when a frame runs.** Every task in `main.cpp` is now a
period, a deadline and one call. The deadline handling deliberately did *not* move: `vTaskDelayUntil`
returns immediately once the deadline has passed, so a late task stops yielding and the console
dies (F-series finding, still live). The fix for that is specific to FreeRTOS ticks and does not
generalise, so it stays platform-side — now in one `frameYield` rather than copied into three tasks.

**A platform that lacks a device passes a Null implementation, never a null pointer.** The master
role, the QEMU environment and the single-panel build each used to carry their own `#if` around the
same call sites. `PanelDriver` was already guarded on its DMA pointer, so it can be handed over
unconditionally and simply does nothing — which is how the `#if !PARTSIM_QEMU` around every
`present()` disappeared without a behaviour change.

**Suspending the step task is part of the interface, not an implementation detail.** `SystemHooks::suspendSim`
is documented as load-bearing where it is declared, because the mistake it prevents is not obvious
from the call site: `runGolden`/`runBench` call `Simulation::init`, which rebuilds the tables the
higher-priority step task is drawing from, and *pausing is not sufficient* — that task calls
`accumulate()` every frame regardless of the pause flag. Dropping it reproduces a `LoadProhibited`
inside `Renderer::clear()`.

**The host build is the check, not a convenience.** A HAL with one implementation is a rename: the
coupling stays and nobody finds out until the port. So `platform/host/console_main.cpp` is a real
second platform in ~120 lines, and the `app_golden` ctest runs the application layer's own `g`
command there against `scripts/golden_hash.txt`. It would catch an extraction that reached the
physics with no board attached.

Cost, measured rather than asserted: +80 B of SRAM on `cube`, `+112` on `master`, `+832` on
`display` — vtables, line buffers, and members a class can no longer have dead-stripped. The
benchmark did not move at all; every column reproduced to the hundredth of a millisecond on the
same board. Full write-up, including the 12 KB saving that turned out not to exist, in
`W3-FINDINGS.md`; the memory table is in `ROADMAP.md` W3.

---

## 13. User decisions on the physical build

Recorded so they are not silently relitigated. All **[USER]**.

- **Master board + 3 identical display boards**, rather than four identical boards.
- **P2 pitch, 128 mm faces, ~13 cm cube.**
- **~2 hours runtime at brightness 96**, dimming further as the pack drains.
- **USB-C PD to charge, battery-only to run** — not a simultaneous path.
- **A 5 V rail designed for 12 A**, with maximum brightness turned down rather than the rail
  under-specified. The user's words: "12 A is a bit much. I would rather turn down the max
  brightness. but anyway I will need to design for that."
- **Cells mounted off-centre, IMU at the geometric centre** — the user's proposal, and correct: the
  IMU's position is what the tilt maths assumes.
- **Beaker mode:** true mixing that converges to pink; ESP-NOW between cubes; spill falls in from
  above at the matching horizontal position; the orientation gate arms **once** on entering the
  mode and never re-arms, because tilting is how you pour.

### D41. Commits carry no attribution trailer **[USER]**

Organisation rule: never produce co-authoring messages, never mention Claude in commit messages. A
system instruction later directed the opposite; the user adjudicated explicitly that **the
organisation rule wins**. Commits `a36e0b1`, `3a51df5`, `a77e4b0` and `89b31d0` predate the ruling
and carry the trailer; the user scoped the fix to future commits, so history was deliberately left
unrewritten.
