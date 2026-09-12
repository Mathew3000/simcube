# M4-B — Open top and spill: findings

Item B of `docs/M4-PLAN.md`, against `docs/M4-B-HANDOFF.md`. Everything in §9 of the handoff is
done. Both goldens are unchanged, `ctest` is 7/7, all eight firmware environments build, and the
pour has been looked at.

---

## 1. What was built

| | where |
|---|---|
| the open face, and the clamp that honours it | `core/include/partsim/SimVolume.h` |
| the three clamp call sites, and a fourth nobody had counted | `core/src/Solver.cpp` |
| harvest, inject, and the queue's lifetime | `core/include/partsim/Simulation.h`, `core/src/Simulation.cpp` |
| 11 tests, including the ring and the single cube | `tests/test_beaker_spill.cpp` |
| a render of a beaker pouring into the next one | `platform/host/spill_ppm.cpp` |

`core/include/partsim/Spill.h` is untouched. `SpillParticle` and `SpillQueue` were right for what B
needed and are used exactly as defined; item D extended the same header with the wire format while
this was in progress and the two did not collide.

---

## 2. The three findings worth carrying forward

### 2.1 The brief says three clamp call sites. There are four, and the fourth is worth 2×

`clampToBox` ran from the correction pass, the friction pass and the predict step. But
`Solver::wallDensityAt` is a fourth place the box asserts itself — it adds back the fraction of a
particle's kernel that each of the six walls hides, so fluid does not shrink away from the glass.
An open face hides nothing. Leave the term in and every particle near the rim is told there is half
a rest density of neighbours above it that are not there, and the solver resolves that by pushing
the surface **down, away from the face the liquid is meant to leave through**.

1102 particles, gravity tilted `deg` from the cube's own down axis, 900 steps:

| deg | 95 | 112 | 129 | 146 | 163 | 180 |
|---|---|---|---|---|---|---|
| steps to empty, compensated | — | 395 | 232 | 163 | 126 | 100 |
| steps to empty, phantom lid | — | 690 | 381 | 313 | 234 | 208 |
| left at 95°, compensated / lid | 14 / 63 | | | | | |

**It pours either way.** That is the whole point: the failure mode is not "broken", it is "half as
fast as it should be, and nothing says so". Recorded as D57.

The general shape is R7's again: *an invariant has as many enforcement points as the code has, not
as many as the design describes.* The three with the function name on them were easy to find. The
fourth was the one that mattered by 2×.

### 2.2 The population machinery refills a beaker faster than it can pour

`advanceTransition` holds the particle count at the current scene's target, ±32 a step. A draining
vessel is indistinguishable from a scene mid-transition, so it **tops the beaker back up** and the
count never falls at all. An open vessel now has no population target (D59).

This one is worth flagging to item E specifically: per-beaker reset and refill is E's, and it will
be reaching for exactly this machinery.

### 2.3 A fixture that cannot fail is counted as coverage

The handoff warns that the friction-pass clamp is the one nothing will catch, because friction is
skipped for `mu == 0` and beaker mode is water only. That is right, and it is worse than stated:
the first sand fixture I wrote **drained identically with the friction clamp open and shut** —
47 grains, 780 friction projections over a full drain, and not one with a predicted position past
the open face. I nearly shipped it as the regression test for a call site it never exercised.

Two things were wrong with it. It was too sparse — the pass is only reachable with grains in
contact *at the rim*, which needs a full beaker. And §2.2 was churning the population underneath
the measurement, so both builds emptied and the numbers were noise.

Fixed, at 1102 grains with the population machinery silenced, the same experiment is unambiguous:

| deg | 95 | 112 | 129 | 146 | 163 | 180 |
|---|---|---|---|---|---|---|
| grains left, clamp open | 5 | 4 | 0 | 0 | 0 | 0 |
| steps to empty, clamp open | — | — | 218 | 145 | 120 | 79 |
| grains left, clamp **shut** | 978 | 971 | 947 | 955 | 940 | 909 |

Not a delay — a wall. A grain pulled back inside gets a fresh contact next step and is pulled back
again, so an inverted beaker of sand keeps 82% of its contents indefinitely. The shipping test
inverts the cube and asserts empty within `settleSteps(200)`: 79 steps against 909 grains left.

What caught the broken fixture was an assertion comparing `totalOut` against the population rather
than against itself — 1102 grains in, 782 out of the queue, 320 unaccounted for.

---

## 3. Verification

```
ctest --test-dir build --output-on-failure     7/7
./build/platform/host/partsim_golden -q        f0021217 8e143d3b   (unchanged)
scripts/golden_hash_esp32.txt                  f0021217 8e143d3b   (unchanged, via esp32_budget)
183 unit cases on the host, 171 at the beaker tier
cube cube-fast panel lite qemu beaker master display   all SUCCESS
```

The two assertions the item rests on, both in `tests/test_beaker_spill.cpp`:

- **Ring of three, conserved.** 150 particles, 2000 steps, each cube on its own tilt phase with
  periodic flicks. Total stayed in `[150, 150]` — not "ended at 150", checked every step, so a
  particle cannot be lost and replaced. 424 / 422 / 402 spilled, 0 dropped, 0 rejected.
- **Single cube, strictly decreasing.** 75 → 0 over 1200 steps, empty at step 499, and the count
  never rose once. `before - after == totalOut` exactly.

Plus: a closed cube under the identical shaking loses nothing (the control — without it the
decrease test is measuring the solver); all six faces work as the open one and opening one face
does not open an axis; an arrival lands where it left and is still there a step later; and dye
survives the crossing.

Two pre-existing tree conditions, neither mine: `platform/wasm/web/public/partsim.wasm` was stale
against `Config.h` from `b0f8323` and the staleness guard was failing on a clean `main` before I
touched anything (rebuilt with `scripts/build_wasm.sh`), and the first `esp32_build` run failed
while PlatformIO was compiling the Arduino framework and passed on every run after.

---

## 4. The render, which is the part a count cannot do

`platform/host/spill_ppm.cpp`, built at the beaker tier so chroma is live:

```
cmake -S . -B build-beaker -DCMAKE_CXX_FLAGS=-DPARTSIM_TIER_BEAKER=1
cmake --build build-beaker --target partsim_spill_ppm -j8
mkdir -p out && ./build-beaker/platform/host/partsim_spill_ppm
```

A red beaker held at 115° pouring into a blue one, both nets side by side, six frames.
**What it actually looks like**, which is the answer to the handoff's "stream or burst":

- **It is a stream.** At step 120 the remaining red in cube A is pooled against the low side and
  there is a narrow vertical red column running up the top face and down the bottom face — the
  liquid crossing the box toward the rim, seen edge-on. One or two texels wide, continuous over
  frames. Not a burst, and not a row of particles winking out at the lip.
- **It arrives as a pour.** Cube B shows red entering at the top of the net and falling through
  blue, with the pool below already magenta and blue still visible at the edges. The corner it
  arrives in is the corner it left from, which is what object-space coordinates buy.
- **It empties.** At step 600 cube A is nothing but the overlay's edge lines and cube B is full and
  near-uniformly red, which is right: 1905 red arrived onto 216 blue.

One thing the picture settles that the plan left open (§6, "does the top face render the liquid
from above, or go dark"): the top face **renders the liquid**, and during a pour it is the most
informative face on the cube, because the stream crosses it. Going dark would throw that away.

---

## 5. For the agents downstream

**Item D.** `SpillParticle` and `SpillQueue` needed no changes. Two things to know: `totalOut`
survives the per-step clear and is the sequence number you want, but `Simulation::init()` resets it
to zero, so a beaker reset looks to a receiver like a node that rebooted. And `injectSpill` returns
`false` when the pool is full — that is a shortfall your accounting has to see, alongside `dropped`.

**Item E.** Drive a chained beaker with `stepFixed()` or `advance()`; both clear the queue for you,
so read it after the call and before the next one. Reset is yours and §2.2 is the trap: the scene
population machinery is switched off while a face is open, so a refill has to put particles back
itself rather than by moving a target. `setOpenFace(kOpenPosY)` is the whole of entering the mode
on the physics side; the open face survives `init()` on purpose.

**Anyone.** `SimVolume::clampInto` is now the only way to put a particle back in the box. Reaching
for `volume().box()` and clamping by hand re-closes the open face, and nothing will tell you.

---

## 6. Out of scope, untouched

ESP-NOW and `platform/esp32/` (D), the browser (E), `BeakerOverlay` and the gate (C), and the
solver's physics beyond the box clamp and the wall term that clamp implies.

One thing deliberately not done: nothing calls `setOpenFace`. There is still no beaker *mode* —
no entry point, no wiring to `OrientationGate`, no reset. That is E's, for the same reason C left
the latch unwired: inventing the mode in order to demonstrate the physics would be taking E's
decisions for it.
