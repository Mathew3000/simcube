# Design decisions

Why the project is the way it is. One entry per decision that was not obvious, with the evidence
that settled it and — where it exists — the reasoning that turned out to be wrong.

**Read the reversals first (§9).** Seven decisions in this project were made on plausible reasoning
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

The seven most useful entries in this file.

### R1. "One S3 cannot drive six 64×64 panels"

**Wrong as stated.** The user challenged it directly. `SPIRAM_DMA_BUFFER` exists and works; only
the *internal-SRAM* configuration is impossible. The claim conflated "does not fit internal SRAM"
with "cannot be done".

Prerequisite if it is ever used: a row-walking blit. `drawPixelRGB888` does a read-modify-write per
bitplane, which is 147,456 scattered PSRAM accesses per frame at six faces, and scattered access is
the one pattern PSRAM cannot absorb. **That prerequisite is now built** (P3, D45) and it walks each
bitplane row sequentially, which is the pattern PSRAM can absorb. The PSRAM measurement is still
owed — `RESOURCES.md` §6 item 2.

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

### R7. "Eight-colour partitioning is worth ~1.8x"

Written into this file as an open proposal with a confident multiplier and a correctness argument
that read as a proof. Both were wrong, in different ways — the argument reasoned about the kernel
radius while the code reads through a cache that extends 15% further, and the speedup measured
1.08x rather than 1.8x because the grid is 216 cells and 27 barriers do not amortise across 8 cells
each.

**The pattern, and it is the sharpest instance of it here: a correctness argument is not a proof
until the thing it reasons about is the thing the code actually does.** The argument was about
`kSmoothRadius`. The code reads `kNeighbourRadius`. Those differ by a factor this project itself
introduced, deliberately, and recorded two entries earlier.

What saved it was measuring before shipping rather than after: the ceiling probe (D44) came first,
the sound-but-slow version was built and measured, and the whole thing was reverted with the
golden hashes back where they started.

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

Splat and blit are a floor no processor removes. Widening the blob to hide the coarse lattice (D21)
is what put it there: it bought the look and it moved the bottleneck.

At 384 particles, `cube` build:

| | splat | blit | floor | a free solver gives |
|---|---|---|---|---|
| as first measured | 17.57 | 11.10 | **28.67 ms** | 34.9 fps |
| after P3 was built | 16.20 | 6.49 | **22.69 ms** | 44.1 fps |

**The figure this entry used to carry — 30.7 ms and 32.6 fps — was wrong by one term.**
`App::runBench` times `resolve` on its own and then times `present()`, which also resolves, so the
`blit` column already contains `resolve`. Quoting the floor as "splat + resolve + blit" counts
resolve twice. The `frame` column has always been right; only the prose derived from it was not.

Caveat on how this was stated earlier: at 375 particles the *solver* is still the large majority of
the frame, so the floor is a **ceiling**, not the current bottleneck. It becomes the binding
constraint at the coarser spacings, where it is over half the frame.

After P3, **splat is three quarters of the floor**, and its cost is the texels that do contribute
(F5). The remaining levers on it are the blob radius (D21, F3) and split-kernel chroma (P1), not
tighter loops.

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

### F5. Counting texels is not counting time

P3 observed that the splat scans a square box for a circular kernel, so ~21% of the texels it
touches can never contribute, and valued removing them at 2-3 ms of 17.6. Measured, it is 1.37 ms —
**7.8%, not 21%**.

The 21% is a correct count of texels and the wrong unit. The wasted texels are the **cheapest**
ones: a rejected texel computes `dx`, a squared distance and a compare, while an accepted one also
does a LUT load, a multiply, a shift and a saturating read-modify-write. Removing a fifth of the
iterations removes under a tenth of the work.

The same shape appears in the blit half of P3 from the other side — there, 6 144 x 6 scattered
accesses sounded like the whole cost and were worth 0.20 ms of 7.50, because the S3's internal SRAM
has no data cache to miss. **Both halves of one proposal mis-estimated by counting the wrong thing**,
and in the blit's case the count pointed at the wrong fix as well.

Whenever a proposal here values a change by counting operations, the count and the cost per
operation are two separate claims and only one of them is usually checked.

### F6. The panels were being sent a wrapped brightness ramp

`cie_luts.h` in the HUB75 library picks its CIE table from `PIXEL_COLOR_DEPTH_BITS` **at compile
time**. Nothing defined it, so it defaulted to the 8-bit table -- while `setPixelColorDepthBits()`
was handed 6 at runtime from `kColourBits`. The library takes the low 6 bits of an 8-bit value, so
the top two were dropped.

Read straight out of the DMA buffer on the devkit, red channel: input 144 sent 62, input 152 sent
**7**, input 192 sent 60, input 200 sent **10**. The top 44% of the range was three sawteeth, and
above 144 a brighter pixel was usually a dimmer one.

Two things made it survive this long, and both are structural rather than bad luck:

* **No panels are attached to the board.** HUB75 is a passive shift-register chain, so everything
  measures correctly with nothing plugged in -- which is what makes the benchmark possible at all,
  and it means the one failure mode that is only visible as light is the one nothing catches.
* **No hash covers it.** Both goldens come from `Renderer::resolve`, which was correct throughout.
  The defect lives between the driver and the panel, downstream of everything the project tests.

Found only because the row-walking blit had to reproduce the compensation exactly and therefore had
to read what it actually did. Fixed by setting the flag, and held there by a `static_assert` against
`kColourBits` rather than by a comment (D45).

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

### P2. ~~Eight-colour cell partitioning~~ — **MEASURED AND REJECTED**

Proposed as ~1.8x by colouring cells `(x&1, y&1, z&1)`, processing colours in sequence and
splitting each colour across cores. Built, measured, reverted. Two separate things were wrong.

**The scheme was unsound as specified.** The argument was that same-colour cells are ≥2 apart, so
their closest particles are ≥`h` apart, where the kernel is exactly zero. True of the **kernel**
and false of the **cache**: `kNeighbourRadius` is 1.15h (D18), so the list a particle reads through
holds neighbours out to 6.9 units while stride-2 same-colour cells put particles as close as 6.0.
A cached neighbour can therefore sit in a cell another core is writing, and the distance test that
would reject it happens *after* the read. Stride 3 — 27 classes — fixes it: 2h against 1.15h, a
1.74x margin. **The flaw was present when P2 was written**, because the margin already existed.

**And with the sound version, the speedup is not there.** Measured on hardware against the
already-parallel baseline of D44:

| particles | hazard-free passes only | + 27-colour correction |
|---|---|---|
| 128 | 8.06 ms | **8.35** (worse) |
| 256 | 26.14 | 24.67 (1.06x) |
| 384 | 49.04 | 45.36 (1.08x) |
| 512 | 74.66 | 68.60 (1.09x) |

The reason is problem size, and it is structural. `kCellSize` is `h` = 2*`kRestSpacing`, so a
32-unit box is a **6x6x6 grid — 216 cells**. Split into 27 classes that is **8 cells per class**,
and a class is a barrier. There is not enough cell-level parallelism to amortise 27 barriers per
pass, and at 128 particles the barriers cost more than the split saves.

The price of taking 1.08x anyway would be steep: both golden hashes move, and colour ordering
settles **60% slower** — 4000 steps to rest where index order takes 2500, because index order
sweeps space monotonically and propagates each correction as a wave, while colour order breaks
that into 27 interleaved sub-sweeps. Slower settling is visible shimmer on a cube; D18 already
refused a faster neighbour cache for exactly that reason.

So the Gauss-Seidel passes stay single-core. What shipped instead is D44: the four passes that are
hazard-free by inspection, parallelised with **no reordering at all**, for a measured 1.17x and
both hashes unmoved.

**If this is revisited**, the thing to change is the problem size, not the colouring. Finer
particles mean a finer grid — at `kRestSpacing` 1.5 the grid is 11^3 = 1331 cells and a class is 49,
which is a different calculation entirely. The `beaker` and `future` tiers are where this becomes
worth re-measuring. A Jacobi correction pass is the other route: it needs a 3N delta buffer, which
D14 rejected at 4096 particles and which is 6 KB at the counts that actually run — the same
inherited-from-an-invalidated-premise shape as R5.

### P3. ~~Row-walking blit and a circular splat bound~~ — **BUILT**

Both halves shipped. The floor went 28.67 -> 22.69 ms at 384 particles; see F2 for the table, D45
for the one design choice worth arguing with, F5 for why the splat half came in at a third of its
estimate; the rejected variants and their numbers are in the commit messages for `62d3751` and
`b78cf13`.

Blit 11.10 -> 6.49 ms. The premise needed correcting first: the cost was attributed to *scattered*
read-modify-writes, and rewriting the inner loop to write every texel to (0, 0) moved the blit by
0.20 ms. Internal SRAM on the S3 has no data cache to miss. The 7.50 ms was per-call work, most of
it the library recomputing the row pointer for every bitplane of every texel.

Splat 17.57 -> 16.20 ms, by walking outward from the texel nearest the particle and stopping at the
first miss rather than computing a chord — the S3 has no hardware square root (F1) and `core/`
links no libm. Exact rather than approximate, so neither hash moves.

### P5. A beaker that looks full does not fit on one S3 **[MEASURED, OPEN]**

The tier was chosen for colour resolution: `32/d` distinguishable regions across the cube, so
`d = 2.5` gives ~13 and mixing reads as mixing. What nobody had measured is how much **liquid** the
device can then hold.

A settled, completely full 32-unit vessel at `d = 2.5` is **1905 particles** (measured on the host
at the beaker tier, not derived from `(32/d)^3`, which overestimates by 10%).

The device's particle sweep, run on a `beaker-chain` board with no panels attached — so this is
the solver alone, on two cores, at 64x64:

| particles | sim/step | frame (2 substeps) | fps |
|---|---|---|---|
| 128 | 6.32 ms | 12.67 ms | **78.9** |
| 256 | 22.86 | 45.78 | 21.8 |
| 384 | 45.88 | 91.84 | 10.9 |
| 512 | 71.56 | 143.23 | **7.0** |

So one ESP32-S3 gives a beaker that is at most **27% full** (512 of 1905) at 7 fps, or **7% full**
— a 2-unit film in a 32-unit box — at a comfortable frame rate. The pool cap of 512 is not the
binding constraint; the solver is.

*Derived from the sweep* (`n^1.77`, §5.1): 30 fps buys ~219 particles, so a vessel that is actually
full at 30 fps needs `d ≈ 5.0` — about six particles across the box, and ~6 colour regions rather
than 13. **The spacing that makes mixing legible and the spacing that fills the vessel are a factor
of two apart**, and no tuning closes that: filling a volume costs `1/d³` particles while colour
resolution pays `1/d`.

Three ways out, none free:

1. **Accept a quarter-full beaker at 7 fps.** A pour is slow; 7 fps may well read fine, and 512
   particles is a genuine body of liquid rather than a film. **This is the cheapest thing to try
   and it has not been looked at yet** — it is a judgement about a render, which is M4-E's to make.
2. **Coarsen to `d ≈ 4`** — a full vessel at ~450 particles and ~8.7 fps, with 8 colour regions.
3. **More silicon.** This is the concrete form of P4: beaker mode wants **8-10x** the solver
   throughput of an S3, not 2x. At P4's measured ratios that is beyond every part on the list —
   i.MX RT1176 at 6.3x is the closest and still short.

Worth being precise about what this does *not* say: nothing here is about the radio, the chain, or
the render, all of which are comfortable. It is the solver, and it is the same `n^1.77` wall §5.1
found — beaker mode simply asks for a filled volume where the other tiers ask for a waterline.

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

**P5 sharpens the requirement**: a beaker that looks full wants 8-10x an S3's solver, not the 2x
that would comfortably serve the existing tiers. If beaker mode is the reason to change silicon,
no part currently on this list reaches it, and the answer is more likely to be a coarser beaker
than a faster chip.

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
refactor therefore *costs* memory rather than saving it; against a 230 KB budget it is noise, but
it is the wrong direction and `+832` lands on the tightest budget in the project. The benchmark did
not move at all: every column reproduced to the hundredth of a millisecond on the same board.

Four smaller results from the same work, each the kind that only a move surfaces:

- **A 12 KB saving that was not there.** `runBench` needs a face of RGB staging, and a display node
  runs no benchmark, so guarding it out looked like 12 KB off the tightest budget — obvious enough
  to be written into a comment before being measured. The old buffer was file-scope in an anonymous
  namespace whose only reference was already inside `#ifndef PARTSIM_PROFILE_ESP32_DISPLAY`, so the
  linker had been dropping it all along. The measured display delta is `+832`, not `-11,456`. The
  guard is kept because it is *true*, not because it buys anything.
- **Scale factors live next to the register write that justifies them.** `MotionSensor` asks the
  driver for `accelScaleG()` rather than declaring its own constant; `Lsm6dsox` answers with the one
  sitting beside the `CTRL1_XL` write. A copy in the application layer is a second place for a range
  change to be missed, and the failure it produces — a truncated gravity vector during exactly the
  hand-shake the object exists for — is invisible in every unit test. Same reasoning as D36.
- **`printf` is the one place `-Wdouble-promotion` cannot be obeyed.** Varargs promote `float` to
  `double` by definition, so every console call site trips it and there is nothing to fix.
  Suppressed with a scoped pragma in `Console.cpp` and `App.cpp` only, so the flag keeps working
  everywhere it can still find a real bug.
- **The seam has a regression check, and it is a grep.** `platform/app` must include only `<c*>` and
  `partsim/...`:

  ```bash
  grep -rnE "Arduino|Serial|vTask|xTask|TickType|heap_caps|digitalRead|freertos" \
    platform/app/src platform/app/include
  ```

  Four hits today, all prose. A fifth that is not a comment is a regression.

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

### D43. The Platform contract is checked, and the aliasing is documented **[STANDS]**

Two review findings on the W3 HAL, neither reachable at the time and both cheap.

`Platform` documents that `display`, `imu` and `link` are never null — a platform with no such
device passes the Null implementation — and `App` then dereferences them unguarded in **18 places**.
But every member defaults to `nullptr`, so `Platform{&console}` compiles and leaves five unset.
Both platforms today populate all six positionally, so nothing could fire; the check exists for the
**third** platform, which is the entire reason the seam was built. Verified by omitting one member
and watching `app_golden` fail with the member named.

And `App` holds `const Platform&` rather than a copy, which is load-bearing: the ESP32's `setup()`
constructs `App` early (it owns ~137 KB of pools) and only later discovers whether SPI came up,
assigning `g_plat.link` to the real transport. **Changing that reference to a by-value copy
compiles, passes every test, and silently leaves every display node on the NullFrameLink** — with
no warning printed, because `SpiFrameLink::begin()` succeeded. The hazard is entirely in how
reasonable the change looks, so the reason lives on the member declaration.

Zero SRAM cost: every one of the eight firmware environments is byte-identical afterwards.

### D44. The second core scales at 1.98x, and the doubt about it was wrong **[STANDS]**

P2 values the eight-colour parallelisation at ~1.8x, which was an estimate. Before building a
week's work on it, the ceiling was measured — `platform/esp32/probe/core_scaling.cpp`, built by
`[env:scaling]`, runs the solver's gather shape on one core and then on both, against a working set
the size of the real thing (72 KB, all internal SRAM).

| reps | one core | both, wall clock | speedup |
|---|---|---|---|
| 4 | 17 351 us | 17 575 us | **1.97x** |
| 8 | 34 700 us | 35 081 us | **1.98x** |
| 16 | 69 391 us | 70 086 us | **1.98x** |
| 32 | 138 776 us | 140 075 us | **1.98x** |

**The specific doubt was that this would not hold.** The solver is gather-dominated — 88 candidate
reads per particle, 27% useful — and its measured CPI of 1.40 is largely memory stalls, on a part
whose internal SRAM has no per-core data cache. Two cores gathering from the same arrays looked
likely to contend at the bus matrix. They do not: the second core costs about 1%, because the S3's
SRAM is multi-banked and the bus matrix serves both CPUs concurrently.

So the estimate was conservative rather than optimistic, and what will actually cost P2 its margin
is barrier overhead between colour classes, not memory. That is a solvable engineering problem
rather than a property of the silicon.

Recorded as a decision because the *method* is the point: four entries in §9 are unmeasured
multipliers that drove real work. An hour of probe before a week of implementation is the cheapest
insurance this project has found. The probe is kept rather than deleted — the same question has to
be asked of any candidate in CUBE-PCB §13.1, and the answer will differ on a part with a real cache
hierarchy.

### D45. The blit reaches the HUB75 library's private framebuffer, and proves it at boot **[STANDS]**

The row-walking blit (P3) needs the DMA row pointer hoisted out of the per-texel loop, which needs
`MatrixPanel_I2S_DMA::fb`. It is private, and the library has no bulk-pixel entry point -- the
`hlineDMA` family takes one colour for a whole span, which a fluid render never has.

The alternative was vendoring a 40-file third-party library to add one accessor: a permanent
maintenance cost, and a fork to re-apply on every upgrade, for a five-line change. So
`platform/esp32/src/PanelFramebuffer.h` reaches the member using the explicit-instantiation idiom
that [temp.spec]/6 exists for -- legal C++, not a layout assumption, not undefined behaviour, and
supported by GCC and Clang since C++11.

Someone could reasonably have made the other choice, so the honest statement of the cost is that
this is a dependency on a private member's **name and type**. What makes it acceptable is that both
ways it can break are loud rather than silent:

* a renamed or retyped member is a **compile error**, not a wrong picture;
* a changed buffer **layout** is caught at boot by `PanelDriver::verifyFastBlit()`, which blits a
  real row through the real shipping code both ways -- library and row writer -- and compares the
  raw DMA words. It covers the brightness table, the bitplane packing, the two-halves bit offset,
  the chain addressing and the run direction at once, on the buffer the library actually allocated.
  On any mismatch the driver keeps the per-texel path for good and the boot line says so.

The house rule is that a claim carries the measurement that settled it, and "the layout is what I
think it is" is a claim. The self-test is that rule applied to correctness rather than to timing,
and it is the reason this is a decision rather than a hack.

The same entry covers the smaller duplication it forced. `PIXEL_COLOR_DEPTH_BITS` has to be set in
`platformio.ini` (F6), which repeats a number `Config.h` owns and D36 says not to repeat. A
third-party preprocessor cannot read a `constexpr`, so the choice was to duplicate or to patch.
Duplicated and **checked**: `PanelDriver.cpp` static_asserts the two against each other, and setting
the flag wrong was confirmed to fail the build.

### D46. The fast blit's self-test must have verified something **[STANDS]**

D45 accepts reading a private member of the HUB75 library, and the whole safety case for that
rests on `PanelDriver::verifyFastBlit()` passing at boot.

Review found the one hole in that: **it could return true having verified nothing.** It examines
face 0 only, and skips a row whose mount maps it onto a chain column — so a quarter-turn on face 0
skipped both passes, returned success, and would have enabled the fast path on whichever *other*
face is mounted square, on the strength of no evidence.

Not reachable as the code stands: `defaultMounts` gives every face rotation 0, and the `m` command
can only change a mount after `begin()` has already verified. It becomes reachable the moment
mounts persist across a reboot — which is exactly what Milestone 4 specifies for chain
configuration. Fixed by tracking whether any row was actually compared and returning that.

The general form is worth keeping: **a self-test that can silently test nothing is worse than no
self-test**, because the thing it licenses — here, reading a third-party private member — is
accepted on the strength of it.

### D47. Two build traps that silently measure the wrong tree **[STANDS]**

Both found by W5 after they had already voided measurements, and neither is discoverable from a
failure — the build succeeds and the number is simply about something else.

**`symlink://` libraries remember an absolute path.** `.pio/libdeps/<env>/core.pio-link` records
the working directory it was resolved from. Copying `libdeps` between checkouts — which is the
documented dodge for the `HTTPClientError` on a new environment — therefore makes the new checkout
compile the **old** checkout's `core/` and `platform/app/`. The giveaway was a probe that deleted
the entire splat inner loop and changed the measured splat by 0.00 ms.

So when seeding `libdeps` from another env or checkout, keep only the HUB75 directory and delete
`core.pio-link`, `app.pio-link` and `integrity.dat`. Those are the parts worth not re-fetching; the
links are not.

**`PLATFORMIO_BUILD_FLAGS` does not reach `symlink://` libraries.** It applies to `src/` only. A
probe `#if` in `core/` therefore compiles to nothing, the build succeeds, and the result looks like
a measurement rather than like a flag that never arrived. Edit the source for a core probe.

Filed as a decision rather than a note because the shape is the project's own: **a measurement that
silently describes the wrong thing is worse than one that fails**, and both of these produce a
plausible number with no error anywhere.

### D51. The overlay classifies faces by normal, not by panel index **[STANDS]**

`Geometry::cube` happens to put the top face at index 5, and nothing may rely on that. A display
node drives an arbitrary subset of the six-panel table, and the table is built from specs rather
than hardcoded, so `BeakerOverlay::init` classifies each panel by `dot(objectUp, panel.n)` — the
inward normal of the top face points *down* — and derives the panel-space direction of "up" from
`dot(objectUp, u)` and `dot(objectUp, v)`.

That is what makes an arrow point the right way on a face whose "up" runs along its `i` axis rather
than its `j` axis, and it is why the overlay silently skips the four faces a two-face display node
does not own instead of scribbling into a slot it does not have.

Same reasoning as the mount table and `AxisMap` (D8): a physical fact about the object belongs in
one place that is derived, not in an index that happens to be right today.

### D52. A tier that compiles out its channels cannot draw a second colour **[STANDS]**

Found by building the beaker overlay against the tier it is for. `PARTSIM_TIER_BEAKER` compiles out
sand and heat, which was the point — 27.4 KB — and that leaves **exactly one accumulation channel**.
`resolve()` maps it through the water ramp, so every overlay texel resolves to the same colour:
white edge lines correct, the orientation gate's **red arrows white**.

No code in the overlay can fix that. One scalar and one ramp cannot express two colours, so it is a
missing channel rather than a missing branch — which is why the finding was reported as a request
rather than worked around.

The fix is that `PARTSIM_ENABLE_CHROMA` belongs in the tier itself, not in an environment that
selects it: a beaker whose liquid has no colour cannot mix, and mixing is the feature. It costs
26.6 KB of firmware (beaker 173,612 -> 200,252 B, 61% of the part) and buys the thing the tier
exists for.

The general shape is worth keeping: **a feature switch that saves memory can remove a capability
something else silently depends on**, and the way that surfaced was building the dependent feature
against the real configuration rather than against the host default.

### D53. The radio costs 30 KB, not the 55 the budget was built on **[STANDS]**

M2 turned WiFi and Bluetooth off for three reasons: ~55 KB of internal heap, ISR jitter against a
16 MHz display clock, and panel emissions degrading the antenna. The second and third stand. The
first was an estimate, and beaker-mode chaining needs the radio back, so it was measured.

**The linker cannot see this cost at all.** Building the master with `WiFi.mode(WIFI_STA)` and
`esp_now_init()` moves the static total by **68 bytes**, because WiFi allocates its buffers from the
heap at init. A budget check that reads the map file therefore reports the radio as free, which is
the opposite of useful. It has to come off a running board:

| | free internal heap | free PSRAM |
|---|---|---|
| radio off | 176,464 B | 8,385,791 B |
| WiFi STA + ESP-NOW | 145,316 B | 8,371,691 B |
| **cost** | **31,148 B (30.4 KB)** | 14,100 B (13.8 KB) |

So 30.4 KB, and the estimate was 1.8x too high. The master keeps **145 KB of heap free** with the
radio up, against a chaining protocol that needs a few. Beaker chaining fits comfortably, and the
figure that made it look marginal was never measured.

`PARTSIM_ENABLE_RADIO` is off by default and is ignored entirely on a display node, whatever it is
set to: those boards sit inside the panel stack where the emissions are worst, they carry the
tightest budget, and D34's other two objections apply to them unchanged.

### D54. Chaining is broadcast with a cumulative count, not paired with acknowledgements **[STANDS]**

Two choices in the cube-to-cube link, both of which had an obvious alternative.

**Broadcast, not paired.** A chain is an order the user configures — cube 1 pours into cube 2 —
and the paired alternative means storing each cube's neighbour MAC, pairing every cube with the
next before the object does anything, and re-pairing whenever one is replaced. Broadcast plus a
link id in the payload costs a byte and makes a cube's position in the chain a setting rather than
a ceremony.

**Unacknowledged, with a cumulative count instead.** A spill packet is worth less than the latency
of retrying it. But a dropped packet in a closed ring is volume that never comes back, and the
symptom — beakers slowly emptying over minutes — reads as a physics leak rather than as a lost
radio frame. So the header carries the sender's **cumulative** spill count, which only ever
advances: a receiver that has taken fewer particles than that knows exactly how many went missing
and can make them up. `SpillReceiver` is that arithmetic, and it is why `SpillQueue::totalOut`
survives `clear()`.

The same reasoning drives the receive ring's overflow policy: it drops the **newest** packet rather
than overwriting the oldest, because a recent loss is made up on the next packet while overwriting
an unread older one loses particles the consumer already believes it is about to get.

### D55. Two `setup()` inits were in a branch the master never takes **[REVERSED]**

`CoreParallel::begin` — the solver's second core, worth a measured 1.17x — was written inside the
single-node `#else` of `setup()`'s role branch. The **master** is the one board in a multi-node cube
whose entire job is the solver, and it took the other branch, so it never started the worker.

The build succeeded, every test passed, all eight environments linked, and the only symptom would
have been a master running at half the speed it had been measured at. It surfaced only because a
second init placed next to it -- the ESP-NOW bring-up -- printed a line that never appeared.

Both now sit **before** the role branch, guarded by `PARTSIM_RUNS_SOLVER` rather than by which task
happens to be created below.

The pattern, and it is the one §9 keeps recording: **a measurement taken on one configuration does
not transfer to another by assumption.** The 1.17x was measured on `cube`, which is single-node, and
carried into a plan for `master` without anything checking that master had the code at all.

### D41. Commits carry no attribution trailer **[USER]**

Organisation rule: never produce co-authoring messages, never mention Claude in commit messages. A
system instruction later directed the opposite; the user adjudicated explicitly that **the
organisation rule wins**. Commits `a36e0b1`, `3a51df5`, `a77e4b0` and `89b31d0` predate the ruling
and carry the trailer; the user scoped the fix to future commits, so history was deliberately left
unrewritten.

### D48. The beaker overlay composites into the accumulation buffers, through two hooks **[STANDS]**

`BeakerOverlay` draws after the fluid has splatted and before `resolve()`, into the same
accumulation buffers the fluid uses, rather than into pixels. That is what lets it inherit the
palette path and the chroma path without knowing either exists: when `PARTSIM_ENABLE_CHROMA`
lands, the overlay composites through the new `resolve` unchanged.

The alternative — drawing into the RGBA buffers after resolve — needs a second colour path, and on
the ESP32 there are no RGBA buffers to draw into (`PARTSIM_INTERNAL_PIXELS` is 0; the firmware
resolves one face at a time into a staging buffer). It would have meant a device-only code path
for the one thing that is purely a look.

The hook is **two** inline methods on `Renderer`, and the split carries the reasoning:

* `addAccum` for the **weight** channel: additive and saturating, because an overlay that assigned
  would make a texel darker than the fluid had made it, and an edge line would flicker dark
  exactly where the liquid touches it.
* `setAccum` for the **chroma** channels: overwriting, because `resolve()` takes the RATIO of the
  dye channels, so adding a red glyph to blue dye resolves magenta over a full beaker and red over
  an empty one. Measured on a render before it was changed. A "hold me this way up" instruction
  whose colour tracks the fill level is the one thing it must not be.

Both bounds-check `i`, `j` and `channel`, which the splat loop does not need to: the splat's
footprint is clamped by construction, whereas overlay coordinates are arithmetic on panel
dimensions, and an off-by-one there writes into the next render slot with nothing to show for it.

`Renderer.cpp` was not touched.

### D49. 3x5, one line, and an M that is not really an M **[STANDS]**

The font is 3x5 because that is the smallest box the Latin alphabet survives, and larger faces get
the same table at an integer scale rather than a second table.

Two things the handoff predicted turned out otherwise, both measured:

**`BOTTOM` fits on one line at 32x32.** Six glyphs is `6*3 + 5*1 = 23` texels against 28 available
inside a 2-texel margin. The wrap was built and is tested anyway; it does not trigger here. It
would at a 4-wide box (`6*4 + 5 = 29 > 28`), which is the configuration the prediction describes.

**M cannot be told from N in three columns.** Three columns have no middle vertex, so the glyph is
chosen for "distinct from N and from H" rather than for looking like an M; the conventional
`101/111/111/101/101` rendered as `BOTTON` on a 32x32 face. Four candidates were rendered at panel
scale and compared, and `111/111/101/101/101` — a solid cap over two legs — was the least bad. It
reads inside the word. It would not read as a lone character, and **3x5 is therefore the wrong box
for anything that has to be read cold**, which is worth knowing before a console overlay puts a
node id on a face.

The font tests are built around the failure this data has rather than around spot checks: every
glyph non-empty, more than one row deep, and **distinct from every other glyph in the table**. A
duplicated bitmap is how a copy-paste error here survives review — green build, green tests, one
letter rendering as another on hardware.

### D50. The orientation gate is a latch with a 20-degree tolerance **[STANDS]**

Armed on entering beaker mode, released the first time gravity lands within tolerance of the
cube's own down axis, **never re-armed**. Re-arming on tilt would make pouring impossible, and
pouring is the mode.

**20 degrees is a judgement, not a measurement**, and is labelled as such in the header. The
accelerometer blend is `alphaMax` 0.02 at 208 Hz, a ~0.35 s time constant, so a tolerance tight
enough to demand a few degrees would also demand the cube be held still — and this gate is meant
to be satisfied by someone standing a cube on a table. If it proves wrong it is one constant.

Tested as a state assertion rather than a call count: sixty gated frames on a real `Simulation`
leave `stateHash()` bit-identical, and sixty after release do not.

The latch is deliberately **not wired to anything**. There is no beaker mode to enter yet —
`grep -ri beaker core platform` finds the tier and the PlatformIO environment and nothing else —
and inventing the mode in order to gate it would have been item E's decision taken by item C.

### D56. The open face is a flag on the volume, and the clamp moves with it **[STANDS]**

Beaker mode needs one wall of the container to be absent. Two ways to do that: change the geometry
so the box genuinely has five faces, or keep the box and mark one face as not a wall.

**A flag, on `SimVolume`.** The box is the thing that says "no particle is ever outside this", so
it is also the thing that should say where that stops being true. Changing the geometry would put
an open-top container into `Geometry::bounds()`, which every node of a multi-node cube has to agree
on exactly (D32), and would make the neighbour grid's dimensions depend on the mode.

The clamp moved with it. `clampToBox` was a static helper in `Solver.cpp`'s anonymous namespace
called from three places; it is now `SimVolume::clampInto`, and the three call sites are
`v.clampInto(...)`. That is not tidying — it is the only way a **fourth** caller cannot silently
re-close the face by reaching for the raw `Aabb`, which is exactly what `wallDensityAt` was doing
(D57).

Off by default, and the closed path is `pclamp` and nothing else, so both golden hashes are
unchanged: `f0021217 8e143d3b` on the host, in WASM, and at the device capacities.

### D57. The open face has FOUR call sites, and the brief names three **[STANDS]**

The handoff says: mind all three `clampToBox` call sites. There is a fourth place the box asserts
itself, and it is not a clamp — `Solver::wallDensityAt` adds back the fraction of a particle's
kernel that each of the six walls hides, so that fluid does not shrink away from the glass. An open
face hides nothing, and leaving its term in place means every particle near the rim is told there
is half a rest density of neighbours above it that are not there. The solver resolves that by
pushing the surface **down, away from the face the liquid is supposed to leave through**.

Measured rather than argued. 1102 particles, gravity tilted `deg` from the cube's own down axis,
900 steps, the only difference being whether the open face's term is subtracted:

| deg | 95 | 112 | 129 | 146 | 163 | 180 |
|---|---|---|---|---|---|---|
| steps to empty, compensated | — | 395 | 232 | 163 | 126 | 100 |
| steps to empty, phantom lid | — | 690 | 381 | 313 | 234 | 208 |
| left at 95°, compensated | 14 | | | | | |
| left at 95°, phantom lid | 63 | | | | | |

Roughly **2× on the pour rate**, and 4.5× on what a gentle tilt leaves behind. It pours either way,
which is precisely why this would have shipped unnoticed: the bug is not "it does not work", it is
"it is half as fast as it should be and nothing says so".

The general shape, and it is the same one as R7: **an invariant has as many enforcement points as
the code has, not as many as the design describes.** The three clamps were the ones with the
function name on them. The fourth was the one that mattered by 2×.

### D58. An arrival enters one rest spacing inside the rim, moving in at the speed it left **[STANDS]**

Three things had to be picked, and all three are visible rather than arbitrary.

**Where, horizontally: exactly where it left.** `SpillParticle` carries object-space coordinates,
not normalised ones, because every cube is the same 32-unit box — so a stream leaving one corner
arrives in the corresponding corner and the chain reads as pouring rather than as teleporting. The
two tangential components are clamped into the receiver's box but otherwise untouched.

**How deep: one `kRestSpacing` inside the open face.** Shallower and the next harvest takes it
straight back out, which would turn a ring into a bucket brigade carrying nothing. Deeper and it
materialises inside the settled body, over-dense, and gets flung — the same failure `spawnOne`
already avoids with the same margin.

**How fast: inward at the speed it left with**, `-|v|` along the open axis, tangential components
kept. Mirroring rather than imposing a constant keeps a fast pour fast and a dribble a dribble, and
it cannot point outward however the particle happened to cross the plane.

`injectSpill` returns false when the pool is full. In a closed ring that is volume that never comes
back, so it is a return value the caller must count, the same way `SpillQueue` counts `dropped`.

### D59. A vessel with a hole in it has no population target **[STANDS]**

`advanceTransition` keeps the particle count at the current scene's target, adding or removing 32 a
step. It is what makes a scene change read as a drain and refill instead of a hard reset.

Against an open beaker it is a disaster, and a quiet one: a draining vessel looks exactly like a
scene mid-transition, so the machinery **tops it back up from the top at 32 particles a step** and
the count never falls at all. So `advanceTransition` returns early while a face is open, and
`transitioning()` stops reporting a drained beaker as mid-transition. The palette crossfade still
runs — that is about colour, not volume.

Found through a test that was lying about something else. The sand fixture for D57 drained
identically with the friction clamp open and shut, and the reason was this: the population loop was
churning sand out and water in underneath the measurement, so both builds emptied and the test
proved nothing. With the machinery silenced the same fixture separates 0 grains from 909.

**A fixture that cannot fail is worse than no fixture**, because it is counted as coverage. The
check that caught it was asserting `totalOut` against the population rather than against itself —
1102 grains went in, 782 came out of the queue, and the missing 320 were the giveaway.

### D60. The outbound queue holds one step, and a reset resets its counters **[STANDS]**

`Simulation` clears its `SpillQueue` at the top of `stepFixed()` and at the top of `advance()` —
not inside `fixedStep`. So the queue always holds exactly what crossed during the step (or the
frame, substeps included) just run, and three callers all get what they need: a chained beaker
reads it after each step and loses nothing; a single cube reads nothing and the clear **is** the
"spills and does not come back" behaviour, with no discipline required of the caller; and `dropped`
comes to mean one specific thing — more than `kMaxSpill` particles crossed in a single 1/60 s step
— rather than "nobody drained the buffer".

`totalOut` survives the clear, because M4-D's receiver counts against it as a sequence number.

`init()` resets both cumulative counters, and deliberately does **not** close the open face: a
beaker reset is a refill, not a return to a sealed cube. A receiver sees `totalOut` restart at zero
and should treat that as it treats a node that rebooted. Found by an assertion that read 150
spilled particles out of a 75-particle beaker — the queue is a member, `init()` rebuilt everything
around it, and the previous run's count carried straight over.

### D61. The chain's carrier is injected into core/, not into the app layer **[STANDS]**

`SpillTransport` — `send` / `poll` / `name` / `diagnostic` — lives in `core/include/partsim/SpillChain.h`,
alongside the pump that uses it, rather than in `platform/app/` next to `FrameLink`.

`FrameLink` is in the app layer because the thing above it is the app layer: `App::masterStep`
encodes a frame and hands it over. The chain is different — the code above the carrier is
`SpillChain::pump`, which is portable, has no platform in it, and is the part most worth testing:
who sends what, when, and what a receiver does about a packet that never came. **Tests link only
`partsim_core`.** A seam defined above core would have left the pump below it untestable, and the
pump is where both defects in this item were.

`EspNowLink` therefore implements a core interface, and `Platform::chain` defaults to
`nullSpillTransport()` so a board with no radio, a board whose radio failed to start, and the host
all run the identical path.

One thing this cost: adding a member to `Platform` shifted every positional initialiser after it.
Both platforms built it as `Platform{&console, &clock, ...}`, so `chain` silently took the `hooks`
pointer. Only a type mismatch on the last argument made it a compile error rather than a null
`hooks`. Both are designated initialisers now.

### D62. The pump belongs inside the step loop, not after it **[CORRECTED ON HARDWARE]**

`advance()` clears the spill queue once per **frame**; `stepFixed()` clears it once per **step**
(D60). The master calls `stepFixed()` twice per frame, so a pump placed after that loop sees only
the second substep's spill.

It was placed after the loop, with a comment explaining that one pump per frame was the efficient
choice at a radio whose cost is per packet. Measured on two boards: **of 198 particles poured, 98
crossed the air.**

What makes this worth an entry is why nothing caught it. The receiver's cumulative-count
arithmetic — the mechanism built to survive a lost radio packet — made the missing half up out of
clones, so the volume at the far end was correct, the ring conserved particles, and every counter
on both boards added up. The pour was simply half copies.

So the pump moved inside the loop, and `SpillChain::Stats::unsent` now names the failure directly:
everything the beaker has ever spilled is either in a packet, refused by its own queue, or was
never offered to the pump at all, and the third case is arithmetic the sender can do alone. The
console prints a warning on any non-zero value. `chain_notices_spill_that_never_reached_a_packet`
holds it.

After the fix, same fixture: **179 spilled, 179 sent in 140 packets, 179 received, shortfall zero,
made up zero, refused zero.** 307 → 128 particles on the sender, 307 → 486 on the receiver.

### D63. A beaker fills to 60% of its pool, because a full one cannot receive **[STANDS]**

The scene preset fills to 100% of the particle capacity. On the first two-board run every one of
229 arrivals was rejected for want of room while the sending beaker emptied into nothing.

A ring conserves volume only if each beaker can hold its own fill **plus whatever is in flight
toward it**, and in a chain every beaker starts full at once. So the beaker tier refills to
`(kMaxParticles * 3) / 5` rather than loading the preset's count.

This is also why `injectSpill` returning false has to be counted rather than ignored: it is the
only evidence that a chain is losing volume to a full vessel rather than to the air, and the two
have opposite fixes.

### D64. A cube's dye is a property of the cube, not of what is in it **[STANDS]**

`Simulation::setDye` sets the dye on every particle now **and** on every particle spawned or
filled later; an arrival keeps the dye it brought. A chained beaker starts red and becomes pink,
and a refill after that must put red back rather than pink — the cube's identity is what it was
configured with, not the average of its contents.

Before this there was no way to set dye at all outside a test: every particle on a device was
`(0, 0)`, which is blue. Console command `d <r> <g>`, 0-255 each, scaled into the 8.8 fixed point
M4-A established. **It does not survive a reboot** — chain order and per-cube colour as persistent
configuration belong with M4-E, which is where the whole configuration surface is.

### D65. A node does not hear its own broadcast **[MEASURED]**

Worth recording because the first hardware run looked as though it might. A node reported packets
in with no peer sending, and self-reception would break a chain outright — a beaker would re-inject
what it had just poured out.

It was not that: the packets came from the other board's pre-reset life, before the test script's
port open rebooted it. `EspNowLink::diagnostic()` now prints the node's own MAC beside the last
sender's, marked `<-- ITSELF` if they ever match. Over two clean runs they never did.

### D66. A cube takes packets from its upstream neighbour, not from whoever is loudest **[MEASURED]**

The chain broadcasts. With two cubes that is harmless and it is what the two-board bring-up proved;
with three it is a volume leak running the *opposite* way from a dropped packet. Every cube hears
every packet, so every cube injects what **both** of the others poured, and the ring fills up out of
nothing.

Measured on a three-beaker host fixture, one cube tilted: unaddressed, **150 particles became
200**. Addressed, 150 stayed 150, with the cube that was not downstream counting 25 packets heard
and ignored.

The sender's position rides in the header byte that has always been written as `0` and documented
as "flags, reserved" — so the format does not grow, and a cube that has never been told where it
stands sends the same bytes it always did. Length below 2 means unaddressed and accepts everyone,
which is what a bench with two boards wants and what every existing caller gets for free.

Changing the position **re-baselines the receiver**. The cumulative count it was tracking belonged
to whoever used to be upstream; carried over, it reads as a shortfall of everything that cube had
ever poured, and the beaker would be filled with clones to match.

Confirmed on hardware across a position change mid-pour: as cube 1 of 3 it took 102 of 102
particles (307 → 409 exactly); re-addressed as cube 2 of 3 it ignored **59 packets** carrying the
remaining 205 particles and its count did not move.

Console command `n <id> <len>`. It does not survive a reboot — chain order and per-cube colour as
persistent configuration belong with M4-E, which is where the configuration surface is.
