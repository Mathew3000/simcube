# partsim

A particle fluid simulation for HUB75 LED panels — water, sand, and fire in a 3D volume,
driven by an accelerometer so tilting and shaking the object moves the fluid.

The target hardware is an **LED cube**: six 32×32 HUB75 panels around a closed volume,
driven by one ESP32-S3 with a 6-DOF IMU. Tilt it and water pools at the new low corner;
shake it and it splashes.

Because iterating physics on hardware is miserable, the same C++ core also compiles to
**WASM and runs in a browser** as a cube you rotate with the mouse. One physics
implementation, three build targets — what you see in the browser is what the cube runs.

---

## Status

Milestone 1 is the browser cube. Steps 0–5 of 12 are done.

| | state |
|---|---|
| `core/` — portable C++17 simulation | geometry, neighbour grid, PBF solver, splat renderer **working** |
| `platform/host/` — tests, benchmark, image dump | **working**, 57 test cases green |
| `platform/wasm/` — browser frontend | not written yet (toolchain installed) |
| `platform/esp32/` — PlatformIO firmware | not written yet |

Working today: water settles correctly in a 32³ cube under gravity from any direction,
survives violent shaking without leaking, renders to six panel images with a continuous
waterline across every seam, and is bit-deterministic. Not yet: sand behaviour, fire, and
either platform frontend.

---

## Prerequisites

Only the host build works right now, and it needs almost nothing:

```sh
brew install cmake        # the only hard requirement today
```

A C++17 compiler (Apple Clang is fine) and `bash` are assumed. No third-party libraries,
no package manager, no network access at build time — the test harness is hand-rolled and
`core/` deliberately depends on nothing but a handful of standard headers.

Needed later, **not yet**:

- `pipx install platformio` for the ESP32 firmware — Milestone 2.

Emscripten **is** installed at `~/emsdk` for the upcoming browser build. Note that emsdk
needs Python ≥3.10 and macOS ships 3.9 with Xcode, so it must be pointed at a newer one:

```sh
export EMSDK_PYTHON=/opt/homebrew/bin/python3.12
source ~/emsdk/emsdk_env.sh
```

---

## Build and test

```sh
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Expect two ctest entries to pass: `unit` (57 cases, ~6 s) and `no_libm` (a source guard,
explained below).

### Running a subset of tests

The test binary takes a substring filter, which is much faster than a full `ctest` run when
you are iterating on one area:

```sh
./build/tests/partsim_tests              # everything, with per-case results
./build/tests/partsim_tests solver       # only the solver cases
./build/tests/partsim_tests hash         # only the neighbour-grid cases
./build/tests/partsim_tests geometry
```

Some cases print measured numbers as well as passing, which is often the interesting part:

```
       mean|v| 0.0000  rho 1.0108  fill 4.56 (want 4.90)  moving 0/1500
  ok   solver_hydrostatic_rest
```

### Sanitizers

Catches the misaligned-load and signed-overflow class of bugs that would otherwise appear
as heisenbugs on Xtensa. Slow — use a smaller particle pool:

```sh
cmake -B build-asan -DPARTSIM_SANITIZE=ON -DCMAKE_BUILD_TYPE=Debug -DPARTSIM_MAX_PARTICLES=4096
cmake --build build-asan -j
./build-asan/tests/partsim_tests
```

### The `no_libm` guard

`core/` must not call libm transcendentals. `sinf`/`cosf`/`expf`/`powf` are not specified to
any particular accuracy, so they differ in the last bits between newlib (ESP32), musl
(Emscripten) and libSystem (macOS) — enough to diverge a fluid trajectory within a few
hundred steps and break the promise that the browser shows you what the hardware does.

`core/include/partsim/Math.h` is the single sanctioned `<cmath>` include, and only for
`sqrtf` (which IEEE-754 *does* require to be correctly rounded). Everything else uses the
polynomial `partsim::fsin` / `fcos` / `fexp` defined there. The guard enforces it:

```sh
bash scripts/check_no_libm.sh .
# ok   core/ is free of nondeterministic libm calls (13 files)
```

Test code may use libm freely — `tests/test_math.cpp` checks the polynomials *against* it.

---

## The benchmark / inspector

`partsim_bench` is the tuning loop. It dumps the baked panel geometry, then settles a tank
of water and reports diagnostics.

```sh
./build/platform/host/partsim_bench
```

Arguments, all optional and positional:

```
partsim_bench [particles] [steps] [iterations] [damping] [epsilon] [substeps]

./build/platform/host/partsim_bench 3000 500              # 3000 particles, 500 steps
./build/platform/host/partsim_bench 2000 400 2 0.98 0.05 1
./build/platform/host/partsim_bench 2000 400 sweep        # sweep iters x damping x substeps
```

### Reading the output

**Panel table** — one row per panel, verifying the geometry by eye. Every `n` must point
inward, and `u`/`v` are the world steps per texel as seen from *outside* the object:

```
face     origin                 u (right)        v (up)           n (inward)
-Z       ( -16.0  -16.0  -16.0) ( 1.0  0.0  0.0) ( 0.0  1.0  0.0) ( 0.0  0.0  1.0)  32x32
bounds  size (32.0 32.0 32.0)
volume  grid 11x11x11 = 1331 cells @ cell 3.0
```

**Vertical profile** — particle count and mean density per 2-unit band. A *uniform* density
column is the signature of a correct boundary; density falling off with depth means the wall
term is under-compensating (see the note below):

```
profile  band      n   rho   (expected n per band for a full layer: 612)
        -16.0    771  1.018
        -14.0    513  1.006
        -12.0    645  1.000
```

**Summary line:**

```
FINAL mean|v| 0.000  max|v| 0.00  rho 1.0141  moving 0.0%  fill 9.4 (want 9.9)  outside 0  6.485 ms/step
```

| field | healthy | meaning |
|---|---|---|
| `mean|v|` | < 0.5, ideally ~0 | settled water should be still. High values mean energy is being injected somewhere. |
| `rho` | 0.95–1.06 | mean density; rest density is normalised to exactly 1.0 |
| `moving` | < 5% | share of particles faster than 1 unit/s |
| `fill` | within 10% of `want` | column height from the centre of mass vs. the height the particle count implies |
| `outside` | **must be 0** | particles that escaped the container |
| `ms/step` | — | solver only, excluding diagnostics |

`outside > 0` or a NaN is a real bug. Everything else is tuning.

---

## Rendering images (no browser needed)

`partsim_ppm` settles a tank, renders the six cube faces, and writes PPM images plus an
unfolded cube net. This exists so the visual concept can be judged without a browser, WASM
or GL anywhere in the loop.

```sh
mkdir -p out
./build/platform/host/partsim_ppm 3000        # scenes at the default exposure
./build/platform/host/partsim_ppm 3000 exp    # sweep exposure instead
```

Writes `out/net_<scene>.ppm` (the unfolded net) and `out/<scene>_<n>_<face>.ppm` per face.
Scenes are `settled`, `tilted`, `falling`, `half` and `neon`.

The net is laid out as the standard cross, so the horizontal strip reads as a continuous walk
around the cube's sides — **a waterline that jumps at a seam means the panel mapping is
wrong**:

```
       [+Y]
  [-X] [-Z] [+X] [+Z]
       [-Y]
```

The `tilted` scene is the useful one: gravity at an angle should produce a straight slanted
waterline that stays continuous across all four side faces.

If your viewer does not open PPM, any of these work:

```sh
magick out/net_tilted.ppm out/net_tilted.png     # ImageMagick, if installed
open out/net_tilted.ppm                          # macOS Preview handles PPM
```

Exposure is the accumulated intensity mapping to the top of a colour ramp
(`kSplatExposure`). It is **measured, not guessed** — a dense water texel peaks near 6500,
and setting it too low clips everything to near-white and throws the whole ramp away. The
tool prints `peak accum` for exactly this reason.

---

## Layout

```
core/                    portable C++17 — no platform deps, no malloc after init, no libm
  include/partsim/       Math Types Config Rng Geometry SimVolume
                         Particles SpatialHash Solver
  src/
platform/host/           bench.cpp — inspector and benchmark
platform/wasm/           (not yet) browser frontend
platform/esp32/          (not yet) PlatformIO firmware
tests/                   hand-rolled harness (check.h) + test_*.cpp
scripts/                 check_no_libm.sh
```

Compile-time capacities are CMake cache variables, so the ESP32 build can shrink the static
pools without touching source:

```sh
cmake -B build -DPARTSIM_MAX_PARTICLES=3000 -DPARTSIM_MAX_GRID_CELLS=2048
```

`PARTSIM_MAX_PARTICLES`, `PARTSIM_MAX_PANELS`, `PARTSIM_MAX_PANEL_TEXELS`,
`PARTSIM_MAX_GRID_CELLS`, `PARTSIM_MAX_FIELD_CELLS`.

---

## Design notes worth knowing before you touch the code

**Everything is object space.** Gravity arrives at the core *already expressed in the
object's frame*. That single choice is why the browser (rotate the object, gravity stays
world-down) and the IMU (which natively reports gravity in the device frame) are the same
code path with no special cases.

**The container is derived, never authored.** `Geometry` is a table of panels; the simulation
volume is their bounding box. A cube gives 32³. A single panel gives a thin slab, so
single-panel mode runs the *same* 3D solver rather than a second 2D one.

**No allocation after init.** Every pool is fixed-capacity static storage, because the ESP32
has ~230 KB of internal SRAM and no room for surprises. `Particles` is ~700 KB at the host
capacity, so it must live in static or heap storage — never on the stack.

**Determinism is a feature, not an accident.** Fixed timestep, seeded xoshiro128** (never
clock-seeded), `-ffp-contract=off` everywhere, no `-ffast-math`, and no libm. The goal is
that host, browser and ESP32 produce bit-identical trajectories.

**Splatting is cheap; the solver is the cost.** Measured on the host, rendering six 32×32
panels is 3–4% of one solver step. Contrary to expectation the renderer is not the
bottleneck, so optimisation effort belongs in the neighbour search.

**Two iterations, not more.** 2 solver iterations settle *better* than 3 or 6 (mean speed
0.03 vs 0.30 at 3000 particles) and cost 28% less. More is not better here.

**The boundary term is load-bearing.** Density near a wall must be compensated for the
neighbours the wall hides, using the closed-form Poly6 half-space integral in
`Solver::init`. An earlier estimate under-compensated, and the symptom was not a boundary
artifact — water over-packed against the floor by 1.5× while the measured density still read
1.0, and 92% of particles churned forever. If the fluid ever refuses to settle, suspect this
before suspecting convergence.

---

## Known limitations

- **Single-panel slab mode is under-tuned.** It needs a depth of ~1.5×h (a 2-unit slab
  cannot fit two particle layers at rest spacing, and the frustration stops it settling),
  and it only tolerates ~25% of nominal capacity before churning, versus a cube's ~90%. Use
  `Solver::capacity()` as a fill ceiling. Deferred: slab mode is not part of Milestone 1.
- **Non-cube panel arrangements use the bounding box as the container.** For an L-shape the
  physics happens in the full box while the panels illuminate only part of it. Per-panel
  inward half-space constraints are the eventual fix.
- **Cube edges show a slightly dark seam.** A particle at an edge has its splat footprint
  truncated by the panel boundary, so the edge columns receive less energy. Real HUB75 panels
  have a physical frame at exactly those seams, so this is being left alone for now.
- **Water climbs the vertical edges a little.** The wall-density term sums one axis at a
  time, so a corner double-counts the hidden region and the solver pushes fluid out of it.
  The fix is a product of per-axis visible fractions rather than a sum.
- **Rotation is approximated.** Feeding only a gravity vector reproduces sloshing but omits
  Coriolis and centrifugal terms, so a fast spin will look under-energetic.
