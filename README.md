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

**Milestone 1 is complete. Milestone 2 (ESP32 firmware) is written but not yet run on
hardware** — the panels and the board are in transit. Everything that can be verified without a
device has been; see [Hardware bring-up](#hardware-bring-up) for the checklist of what cannot.

| | state |
|---|---|
| `core/` — portable C++17 simulation | geometry, neighbour grid, PBF solver, splat renderer **working** |
| `platform/host/` — tests, benchmark, image dump, memory report | **working**, 120 test cases green |
| `platform/wasm/` — WASM module + three.js cube | **working**, verified bit-identical to host |
| `platform/esp32/` — PlatformIO firmware | **compiles and links**, 163.5 KB static of 320 KB; **never executed** |

Working today: water, sand and fire in a 32³ cube, with seven scene presets, two palettes,
optional auto-cycle, and bloom — running live as a three.js cube in the browser that you tilt
with the mouse to make it slosh. Water settles correctly under gravity from any direction and
survives violent shaking without leaking; sand heaps and sinks through water; flames lean when
the cube is tilted and get pushed around when it is shaken.

Cross-target determinism is **verified** between two of the three targets: the WASM build
produces bit-identical particle state and pixels to the native build across water, sand *and*
fire, and runs within 5% of native speed. The device is the third target and its reference hash
is recorded (`scripts/golden_hash_esp32.txt`), but nothing has run on hardware to compare it
against.

Not yet verified at all: anything that requires the device to execute — frame rate, panel
refresh, IMU behaviour, thermals, power. The firmware's pin assignment and the cube's face mount
table are placeholders by necessity; both are facts about a physical object that does not exist
yet.

---

## Prerequisites

The host build needs almost nothing:

```sh
brew install cmake        # the only hard requirement
```

A C++17 compiler (Apple Clang is fine) and `bash` are assumed. No third-party libraries,
no package manager, no network access at build time — the test harness is hand-rolled and
`core/` deliberately depends on nothing but a handful of standard headers.

For the ESP32 firmware:

```sh
pipx install platformio   # or use the VS Code extension's copy, see below
```

`scripts/build_esp32.sh` also finds PlatformIO inside its own virtualenv at
`~/.platformio/penv/bin/pio`, which is where the VS Code extension installs it — so if you have
that, there is nothing to install.

For the browser build, Emscripten is installed at `~/emsdk`. Note that emsdk needs Python ≥3.10
and macOS ships 3.9 with Xcode, so it must be pointed at a newer one:

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

Expect six ctest entries to pass:

| test | what it checks |
|---|---|
| `unit` | 120 cases, ~75 s |
| `no_libm` | `core/` calls no nondeterministic libm (see below) |
| `wasm_determinism` | WASM output is bit-identical to host — **skips loudly** if the WASM build is absent |
| `esp32_budget` | static pools at the device capacity profile fit internal SRAM, and the physics at those capacities still reproduces `scripts/golden_hash_esp32.txt` |
| `esp32_build` | the firmware compiles and links for Xtensa — **skips loudly** without PlatformIO |
| `golden_hash_current` | `scripts/golden_hash.txt` is not a stale reference |

The two `esp32_*` entries need no hardware. They exist because `core/` is shared: a change that
is fine on Clang and Emscripten can fail on the device — a stray `double` that
`-Wdouble-promotion` catches, a pool that no longer fits, a header newlib lacks. Without them the
first sign would be at flashing time.

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
./build/platform/host/partsim_ppm        # every scene, 400 steps each
./build/platform/host/partsim_ppm 800    # settle longer
```

Writes `out/net_<scene>.ppm` (the unfolded net) and `out/<scene>_<n>_<face>.ppm` per face, for
every scene in the table plus tilted variants of the water tank and the campfire.

The net is laid out as the standard cross, so the horizontal strip reads as a continuous walk
around the cube's sides — **a waterline that jumps at a seam means the panel mapping is
wrong**:

```
       [+Y]
  [-X] [-Z] [+X] [+Z]
       [-Y]
```

The tilted variants are the useful ones. `net_0_water_tank_tilt35` should show a straight
slanted waterline continuous across all four side faces; `net_1_campfire_tilt35` should show the
flame *leaning*, which is the check that heat uses the same object-space gravity the solver
does.

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

## The browser build

```sh
./scripts/build_wasm.sh      # emcc -> platform/wasm/web/public/
./scripts/serve.sh 8080      # then open http://localhost:8080/
```

A server is required — ES modules and WASM cannot load from `file://`. There are three pages:

| page | what it is |
|---|---|
| `/` | the LED cube in three.js. **Drag** tilts the object (gravity stays world-down, so it sloshes), **flick + release** shakes it, **right-drag** orbits the camera, scroll zooms. |
| `/panels.html` | the six panels as flat 2D canvases in the same unfolded-net layout as `partsim_ppm`. Validates the C ABI and zero-copy views with no 3D involved, so a bug can be localised. |
| `/orient.html` | panel orientation checker — see below. |

Controls on the main page: a scene dropdown, `palette` to cycle colour ramps, `auto-cycle` to
drift between scenes on a timer, `bloom` to toggle the glow, and `reset`.

three.js r170 is **vendored** in `platform/wasm/web/vendor/`, not installed via npm, so the repo
stays dependency-free and needs no bundler. See `vendor/README.md` for the versions and how to
upgrade.

Useful query parameters, there so headless screenshots can verify things a level tank cannot:
`/?tilt=35` pre-rotates the cube, `/?scene=6` selects a preset, and `/orient.html?cam=x,y,z`
moves the camera.

### Checking panel orientation

`/orient.html` writes a known asymmetric pattern straight into the panel buffers instead of
running the simulation. **On every face, viewed from outside, you must see a red block at the
bottom-left (texel 0,0), a green bar running right along +x, and a blue bar running up along
+y.** Red anywhere else means that face is flipped or mirrored.

This exists because symmetrically settled water cannot reveal a mirrored face, and it shares its
scene code with the live page (`cubeview.js`) — a checker that built its own scene would prove
nothing about the real one. It is also the fastest way to validate a real cube's panel wiring
later.

### Cross-target determinism

This is the project's central promise — that the browser shows you what the ESP32 will do —
and it is now checked rather than hoped for:

```sh
node scripts/check_determinism.mjs
# wasm  state c2d5daf4  pixels 52a0f576
# host  state c2d5daf4  pixels 52a0f576
```

Both targets run the *same* scripted motion sequence (`goldenHash` in `core/`, deliberately not
duplicated per harness so the two cannot drift), then hash particle state and rendered pixels.
The fixed timestep, the seeded RNG, `-ffp-contract=off` and the libm ban all exist to make this
pass.

If you intentionally change the solver or renderer, regenerate the reference:

```sh
./build/platform/host/partsim_golden -q > scripts/golden_hash.txt
```

The device is the **third** target, and its reference is separate because it runs smaller pools
(scene particle counts get clamped to capacity, so the trajectory legitimately differs):

```sh
./build-esp32/platform/host/partsim_golden -q > scripts/golden_hash_esp32.txt
```

Host and WASM agree today. The device side is a recorded expectation, not a result — the `g`
console command prints the number to compare, and nobody has run it.

---

## The ESP32 firmware

```sh
scripts/build_esp32.sh                    # build the `cube` environment
scripts/build_esp32.sh cube cube-fast panel
scripts/build_esp32.sh --upload cube      # flash, once hardware exists
```

Three environments:

| env | for |
|---|---|
| `cube` | the shipping build: six faces, `-ffp-contract=off`, deterministic |
| `cube-fast` | FMA contraction on. A few percent faster and it **breaks** the determinism check by construction; the firmware says so at boot |
| `panel` | one panel, for bringing a single display up before six exist. Uses the simulation's thin-slab mode — the same 3D code path |

The plan called for `-ffp-contract=fast` on the device unconditionally. That cannot coexist with a
third-target determinism check, so it became a separate labelled environment rather than a silent
loss of the guarantee.

### Static memory

The budget is computed, not estimated:

```sh
./build-esp32/platform/host/partsim_memreport 230
```

At the device profile (1280 particles, 6 panels of 32×32):

| | KB |
|---|---|
| Particles (SoA pool) | 51.3 |
| Renderer (accumulation) | 36.3 |
| FieldGrid (heat, ping-ponged) | 20.9 |
| SpatialHash | 10.5 |
| everything else in `Simulation` | 5.6 |
| **`Simulation` total** | **124.6** |
| HUB75 DMA, 6-bit double-buffered | 72.0 |
| one-face RGB staging | 3.0 |
| **total** | **199.6** |

The linker agrees, and is the authority: 163.5 KB of static data out of 327.7 KB of DRAM (the
extra over `Simulation` is the Arduino core's own `.bss`), plus ~72 KB the DMA buffers take from
the heap at `begin()`. Roughly 85 KB left for stacks and the allocator.

Two decisions came out of this:

- **1280 particles is a CPU limit, not a memory one.** ~5500 cycles/particle/step at 240 MHz and
  30 fps is about 1300; a bigger pool would only buy RAM pressure.
- **No internal RGBA copy of the panels.** Six faces of RGBA is 24 KB, which was the difference
  between fitting and not. The firmware resolves one face at a time into a 3 KB staging buffer,
  through the same splat path the browser uses (`Simulation::accumulate` + `Renderer::resolve`),
  so the two cannot drift apart visually.

### Task layout

| core | prio | task | |
|---|---|---|---|
| 1 | 2 | `simTask` | IMU fusion, solver, splat, blit, buffer flip — all the float |
| 0 | 3 | `imuTask` | sensor reads only, **integer-only by construction** |
| 1 | 1 | `loop()` | the serial console (Arduino's own loopTask) |

FreeRTOS on Xtensa saves FPU context lazily, so float-using tasks must be pinned rather than left
floating between cores. Both are. `imuTask` reads raw registers and does no float at all, so the
two cores never contend for the FPU — which is also why this repo has its own 90-line LSM6DSOX
driver instead of `Adafruit_LSM6DS`, whose `getEvent()` returns floats. It runs at a *higher*
priority than the simulation despite doing less work: it has a 4.8 ms sample deadline, whereas a
late frame is merely a late frame.

Samples cross between them through a lock-free single-producer ring, and the consumer drains
everything pending and runs the filter once per sample. So the filter always sees its designed
208 Hz regardless of frame rate — which is what makes the host tests in `tests/test_motion.cpp`
representative of the device.

### Serial console

115200 baud. `?` for help.

| | |
|---|---|
| `s` / `s <n>` | list scenes / switch (gradual, crossfaded) |
| `c` | toggle auto-cycle |
| `b <0-255>` | panel brightness |
| `m` / `m <face> <rot> <mirror>` | print / edit the face mount table |
| `t` | orientation test pattern |
| `i` | IMU state: down vector, container accel, gyro bias, accel trust |
| `r` | frame timing (sim / splat / blit split) and free heap |
| `g` | run the golden determinism sequence and print the hash |
| `p` | pause the physics |

---

## Hardware bring-up

Nothing below has been done. It is the list of things the host tests **cannot** stand in for,
in the order that finds problems cheapest-first.

**Before six panels exist.** Buy *one* first. 32×32 panels ship as both 1/16- and 1/8-scan with
identical connectors, and many need an FM6126A/ICN2038S init sequence — so bring one up, then buy
five more from the same batch.

1. **Check `platform/esp32/src/Pins.h` against the wiring.** The assignment there is electrically
   valid but arbitrary. The exclusion list in that file is not arbitrary: IO35/36/37 are consumed
   permanently by the octal PSRAM on an N16R8, and using them appears to work until the first
   PSRAM access.
2. `scripts/build_esp32.sh --upload panel`. Expect the boot banner to report the geometry, the
   chain size, and free heap after init. A `FATAL: HUB75 init failed` means the pins.
3. `t` — the test pattern. A white dot at texel (1,1), a 3-pixel red arm along +x and a 5-pixel
   green arm along +y. Two different lengths on purpose, so a 90° rotation is distinguishable
   from a mirror at a glance. If the panel is noise rather than a pattern, suspect CLK or LAT.
4. `r` — frame timing. **This is the number the whole particle budget rests on** and it has never
   been measured. The estimate is ~3 ms blit, and sim+splat within ~28 ms at 1280 particles. If
   sim time is far worse than that, the levers are (in order) particle count, `kSplatInfluence`,
   splatting at half resolution, and replacing `drawPixelRGB888` with a direct row write — the
   `ChainRun` structure in `ChainMap` exists so that last one does not disturb the mapping.
5. `g` — the determinism check. Must print `a55e186f`, the first field of
   `scripts/golden_hash_esp32.txt`. A mismatch here means the device and the browser are running
   different physics, and it is worth stopping to find out why. Blocks the display ~30 s.
6. **IMU.** `i` with the object resting on each face in turn. The `down` vector must read −1 on
   whichever object axis is physically down. If the axes are permuted or flipped, that is the
   `AxisMap` in `setup()` — one 3×3 signed permutation, and the only place mounting orientation is
   described. Then check `trust` falls to ~0 while shaking and returns to ~1 at rest.

**Once six panels exist.**

7. `m` — calibrate the mount table against the built cube, using `t` and adjusting until all six
   faces read the same. This is why it is a runtime command: reflashing once per guess is
   miserable. The mapping is proven to stay a bijection under every rotation/mirror combination
   (`tests/test_chainmap.cpp`), so a typo cannot silently blank a face — it is refused.
   **The calibrated table is not yet persisted**; copy it into `ChainMap::defaultMounts` or add
   NVS storage.
8. **Signal integrity** is the top hardware risk: ~1.4 m of unterminated ribbon clocked at 16 MHz.
   Design in 22–33 Ω series resistors on CLK/LAT/OE/RGB and a 74AHCT245 repeater footprint *now* —
   retrofitting into a finished cube is miserable.
9. **Power: 5 V / 20 A**, per-panel injection on 14–16 AWG, never through the pigtails. 1000 µF
   local bulk, a separate regulated rail for the S3, the PSU outside the cube, and vents or a fan.

---

## Layout

```
core/                    portable C++17 — no platform deps, no malloc after init, no libm
  library.json           makes core/ a PlatformIO library (consumed via symlink://)
  include/partsim/       Math Types Config Rng Geometry SimVolume Particles SpatialHash
                         Solver FieldGrid Renderer Palette Scene Simulation
                         MotionSource  — IMU fusion, testable without an IMU
                         ChainMap      — cube face → HUB75 chain pixel
  src/
platform/host/           bench.cpp ppm_dump.cpp golden.cpp memreport.cpp
platform/wasm/           bindings.cpp (C ABI)
  web/                   index.html (3D cube) panels.html orient.html cubeview.js
  web/vendor/            pinned three.js r170 + OrbitControls (MIT, committed on purpose)
platform/esp32/          platformio.ini
  src/                   main.cpp (tasks + console) PanelDriver Lsm6dsox Pins.h
tests/                   hand-rolled harness (check.h) + test_*.cpp
scripts/                 build_wasm.sh build_esp32.sh serve.sh check_determinism.mjs
                         check_no_libm.sh check_wasm.sh check_golden.sh
                         check_esp32_budget.sh check_esp32_build.sh
                         golden_hash.txt golden_hash_esp32.txt
```

Note where the IMU filter and the panel mapping live: in `core/`, not in `platform/esp32/`. They
are the only parts of the hardware path that are pure logic, so they are the only parts that can be
tested without a device — which is exactly why they are on the portable side of the line. The ESP32
layer above them reads registers and pushes pixels, and nothing more.

### Capacities

The device numbers live in `core/include/partsim/Config.h` behind one `PARTSIM_PROFILE_ESP32`
switch, not as a list of `-D` flags in `platformio.ini`. Spelling them out in the build file would
give the host verification and the firmware each their own copy of the budget, and the first time
one was edited the checks would quietly start measuring a configuration nobody ships.

```sh
cmake -B build-esp32 -DPARTSIM_PROFILE=esp32     # build host tools at device capacities
cmake -B build -DPARTSIM_MAX_PARTICLES=3000      # or force one directly
```

`PARTSIM_MAX_PARTICLES`, `PARTSIM_MAX_PANELS`, `PARTSIM_MAX_PANEL_TEXELS`,
`PARTSIM_MAX_GRID_CELLS`, `PARTSIM_MAX_FIELD_CELLS`, `PARTSIM_INTERNAL_PIXELS`. Each defaults to
empty in CMake so that an unset one falls through to the profile rather than being pinned to a
stale copy of it.

---

## Materials and scenes

Three materials share one solver and one renderer:

- **Water** — PBF particles. Its free surface is the whole point, so it gets the expensive path.
- **Sand** — the same particles with twice the rest density and Coulomb friction on contacts.
  Sinking through water falls out of the density constraint rather than being scripted.
- **Fire/smoke** — a coarse `uint8` field, *not* particles. Fire has no surface and no
  incompressibility, so PBF buys nothing for it, while a buoyant scalar field is a few KB and one
  semi-Lagrangian pass against ~5500 cycles per particle.

Scenes are `const` C++ tables in `core/src/ScenePresets.cpp` (flash rodata on the ESP32, zero
parsing): water tank, campfire, sand pile, water and sand, kettle, neon tank, twin flames. Each
declares its materials, palette, heat emitters and auto-cycle dwell time.

Switching has two modes. `setScene` replaces the population immediately, which is right for a
deliberate click. `transitionToScene` — what auto-cycle uses — drains the old materials and
refills the new ones at 32 particles per step (~1.6 s for a full tank) while crossfading the
palette, because a tank that vanishes and reappears in one frame reads as a glitch rather than as
a change of scene. It drains before refilling, so swapping one material for another never briefly
exceeds capacity. Palettes are ramp
tables in `PaletteData.cpp`; the splat loop only ever accumulates per-material *intensity*, and
colour is applied once at resolve time — which is what makes both data-driven.

---

## Design notes worth knowing before you touch the code

**Everything is object space.** Gravity arrives at the core *already expressed in the
object's frame*. That single choice is why the browser (rotate the object, gravity stays
world-down) and the IMU (which natively reports gravity in the device frame) are the same
code path with no special cases.

**The container is derived, never authored.** `Geometry` is a table of panels; the simulation
volume is their bounding box. A cube gives 32³. A single panel gives a thin slab, so
single-panel mode runs the *same* 3D solver rather than a second 2D one.

**No allocation after init.** Every pool is fixed-capacity static storage, because the ESP32 has
~230 KB of internal SRAM and no room for surprises. `Particles` is ~700 KB at the host capacity,
so it must live in static or heap storage — never on the stack. This is **enforced**, not just
intended: `test_noalloc.cpp` replaces global `operator new` with a trap and arms it around the
step, render, scene-switch and auto-cycle paths. Measured result is zero allocations, even during
init.

**Determinism is a feature, not an accident.** Fixed timestep, seeded xoshiro128** (never
clock-seeded), `-ffp-contract=off` everywhere, no `-ffast-math`, and no libm. Host and browser
are *verified* bit-identical by `wasm_determinism`; the ESP32 joins that check in Milestone 2.

**`ALLOW_MEMORY_GROWTH` must stay off in the WASM build.** Growth swaps in a new `ArrayBuffer`
and detaches the old one, silently zero-lengthing the cached panel views — black canvases with
no error anywhere. Nothing is allocated after init, so growth would buy nothing.

**Panel row 0 is the bottom row.** That matches WebGL's bottom-left texture origin, and
`DataTexture` defaults to `flipY = false`, so the three.js path needs no flip at all. Canvas 2D
is the odd one out (row 0 at the top) and flips when blitting; forget it and the water pools at
the ceiling.

**Panel quads use `THREE.BackSide`.** The basis `(u, v, n)` is right-handed with `n` pointing
*inward*, so a plane built from it faces into the volume while we view it from outside. Building
`(-u, v, -n)` to face outward would mirror the texture horizontally instead.

**Container acceleration has the sign you might not expect.** `addContainerAccel` takes the
acceleration of the *container*, so pushing the cube up presses the fluid down and shoving it
right piles the water left. That is correct — inside a container its acceleration is
indistinguishable from gravity the other way — and it is named for the container precisely
because `addJerk(up)` reads as "throw the water up", which is backwards.

**WASM costs almost nothing.** Measured: 6.52 ms/step in WASM vs 6.24 ms native at 3000
particles — within 5%. The browser is a fair preview of the physics, not a slow approximation.

**Splatting is cheap; the solver is the cost.** Measured on the host, rendering six 32×32
panels is 3–4% of one solver step. Contrary to expectation the renderer is not the
bottleneck, so optimisation effort belongs in the neighbour search.

**Physics rate is not free to lower.** Measured over identical simulated time, only the rate
changing: 60 Hz settles to mean speed 0.000 at density 1.012; 40 Hz gives 0.857; 30 Hz gives
1.314; 20 Hz gives **2.604 at density 1.074** — never settles, and past the ~5% over-compression
where churning starts. The original plan assumed 20 Hz physics would buy 1.5× the particles for
free; it does not. Render interpolation was built instead as a rate-independent feature
(`Renderer::setTimeOffset` advances the splat along each particle's velocity by the accumulator's
unconsumed time), so the display can outrun the physics at whatever rate the hardware can afford.

**Fire needs its own, much longer splat reach.** A plume in the middle of a 32-unit cube sits 16
units from every side face, so at the particle influence of 8 only the floor and ceiling lit up
at all. Fire is emissive and glows through the volume; `kHeatInfluence` spans the box.

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
- **Sand's angle of repose is shallower than reality.** It heaps and holds (peak 5.6 vs water's
  2.6 for the same deposition) but the implied slope is ~20–25° rather than the 30–35° of real
  sand. At 32³ with ~1.2-unit grains the box is only ~27 grains across, and the boundary term
  credits a floor particle with half its neighbourhood, so a thin layer packs looser than bulk.
  Worth revisiting only if it reads wrong on hardware.
- **No water/fire coupling.** No steam, no evaporation, no extinguishing — the `kettle` scene is
  two systems sharing a volume. Deliberate scope choice.
- **Rotation is approximated.** Feeding only a gravity vector reproduces sloshing but omits
  Coriolis and centrifugal terms, so a fast spin will look under-energetic.
- **Real frame rate has never been measured — on either target.** Headless Chrome's software
  rendering and virtualised timers made its readings meaningless in Milestone 1, and the device
  does not exist yet. The `r` console command and the browser overlay both report it; until one of
  them is read on real hardware, the particle budget is an estimate.
- **The firmware has never executed.** It compiles, links and fits, and every piece of it that is
  pure logic is unit-tested — but "compiles and fits" is not "works". See
  [Hardware bring-up](#hardware-bring-up).
- **Pin assignment and mount table are placeholders.** Both describe a physical object that does
  not exist. The mount table is at least editable at runtime (`m`), but is **not persisted** across
  reboots yet.
- **The IMU axis map is identity.** It has to be, until someone can rest the object on each face
  and read `i`. The filter itself is tested against synthetic samples, including a deliberately
  permuted-and-flipped mounting.
