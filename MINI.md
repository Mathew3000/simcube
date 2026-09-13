# MINI — the 8×8×8 WS2812B cube: session brief

**You are picking this up cold.** Everything needed is here or linked from here.

**Branch `mini-cube`.** Work here, not on `main`. Nobody else is in this branch.

---

## 0. Two things to settle before you write code

1. **The LED addressing order.** The user knows it and it is not written down yet. **Ask them**, or
   determine it with the test pattern (§6.3) — do not guess. Everything else in this document holds
   whatever the answer is; this one fact decides one function.
2. **Confirm the panel wiring is six 8×8 matrices in one chain**, not a volumetric lattice. This
   brief assumes six faces on a shell, 384 LEDs. If it turns out to be a 512-LED volumetric cube,
   **stop and say so** — that is a different renderer and a different document.

---

## 1. What this project is

`partsim` — a particle fluid simulation for an LED cube: panels forming a closed volume, driven by
an ESP32 with a 6-DOF IMU, so tilting pools the liquid and shaking splashes it. Water, sand, fire,
and a "beaker" mode where the cube is an open-topped vessel you pour out.

One C++17 core compiles to host tests, WASM/three.js and ESP32 firmware, and all three run
**bit-identical** physics, checked by `ctest` against a 32-bit state hash.

| document | what it gives you |
|---|---|
| [`docs/SIMULATION.md`](docs/SIMULATION.md) | the mathematics and the software structure, end to end |
| [`docs/SIMULATION.md`](docs/SIMULATION.md) §6 | **rendering** — panels, splatting, resolve. Read before §3 below. |
| [`docs/SIMULATION.md`](docs/SIMULATION.md) §11 | beaker mode and the chain |
| [`docs/DECISIONS.md`](docs/DECISIONS.md) §9 | the seven reversals. **Read these.** |
| [`docs/RESOURCES.md`](docs/RESOURCES.md) §5.1 | what the hardware actually does, and how it was measured |
| [`docs/MCU-REQUIREMENTS.md`](docs/MCU-REQUIREMENTS.md) | the throughput unit, and why the plain ESP32 is not obviously worse than an S3 here |
| [`docs/DESIGN-SUGGESTIONS.md`](docs/DESIGN-SUGGESTIONS.md) | a proposal to replace the fluid with a dye field. **Not your task — but read §9 below before you judge a render.** |

The house rule, and it is the whole culture of this repository: **this project has been wrong seven
times by asserting a number instead of measuring one.** A commit message here carries the
measurement that settled it, and a negative result honestly reported is worth more than a confident
one. Four agents before you found real defects by measuring something a brief had merely asserted —
including several in briefs written by this author. **Doubt this document where it can be checked.**

---

## 2. The hardware

| | |
|---|---|
| display | six 8×8 WS2812B matrices, one shell, **384 LEDs**, one data chain |
| MCU | **plain ESP32** (Xtensa LX6, dual core, 240 MHz) — *not* an S3 |
| supply | **5 V, 3 A** |
| IMU | **none yet.** One will be added. |

The user already owns this cube. It exists, it is wired, and it is the fastest route to seeing the
whole simulation run on real hardware — the 32×32 HUB75 panels the rest of the project is aimed at
are still in transit.

### Why this is a much smaller job than it looks

**Six 8×8 faces on a shell is the same topology the project already drives.** The cube is
`Geometry::cube(8, 4.0f)` — six panels, world still 32 units, pitch 4.0 world units per LED. The
solver, the neighbour search, the renderer, the palettes, chroma, the beaker, the spill queue and
the ESP-NOW chain are all **unchanged**. Even the face-mount table is unchanged: `ChainMap` maps a
renderer texel of face `k` to a pixel in a daisy chain of tiles, with per-face rotation and
mirroring, and that is exactly what one WS2812B strip threaded through six matrices is.

`Display` (`platform/app/include/partsim/app/Display.h`) is the seam, and its own comment says why
it will fit: *"ChainMap and the mount table stay in it because they are a property of how panels are
physically arranged, not of the HUB75 protocol — a different display technology in the same cube has
the same mount problem."* That sentence was written speculatively. You are the test of it. **If it
turns out to be wrong, that is a finding worth more than the port.**

So the new code is: one `Display` implementation, one capability profile, one tier, a power limiter,
a motion source, and a PlatformIO project. Everything else is configuration.

---

## 3. The work

### M1. A profile and a tier

Two orthogonal things, and `core/include/partsim/Config.h` explains the distinction at the top: a
**profile** is capacities and panel resolution (what the hardware can hold), a **tier** is what to
simulate and how well. Add one of each.

```
PARTSIM_PROFILE_ESP32_MINI      panel res 8, 64 texels/panel, 6 panels, drives them, runs the solver
PARTSIM_TIER_MINI               water only, chroma ON, d = 4.0, 8-bit colour
```

Follow the existing blocks; each sets only what it means to change.

**Chroma on is not a luxury here — it is free and it is the point.** A WS2812B is 24-bit RGB per
pixel where HUB75 on this project is 6. The beaker tier already turns chroma on, and this display
resolves it better than any panel in the project.

**`d = 4.0`, and this is the one configuration in the project where the coarsest tier is not a
compromise.** The face is 8 texels across and the world is 32 units, so one texel is 4 world units.
A rest spacing of 4.0 puts one particle per texel. Finer particles buy resolution the display
cannot show, and `DECISIONS.md` P5 is the measurement of what they cost: filling a vessel needs
`1/d³` particles.

*Derived, from the beaker tier's measured `n^1.77` sweep — check it with `x` rather than trusting
it:* a full 32-unit vessel at `d = 4.0` is roughly **450 particles**, a half-full beaker about
**225**, and 225 is about **17 ms/step** on an S3 — call it **~29 fps at two substeps**. If the
plain ESP32 lands within 25% of that, **this cube runs the fluid better than any other hardware in
the project**, because a half-full beaker is exactly what beaker mode wants and P5's problem (an
S3 cannot fill a 64×64 vessel) does not exist at this size.

### M2. `Ws2812Display`

Implements `Display`. Per frame, for each driven face: `r.resolve(panel, staging, 3)` gives tight
RGB888 for that face's 64 texels; map each through `ChainMap` to a chain pixel; map the chain pixel
to a strip index; write the LED buffer. Then one `show()`.

`platform/esp32/src/PanelDriver.cpp:309` is the model — `present()` there is eleven lines.

Three things that are yours and not `ChainMap`'s:

- **Serpentine.** WS2812B matrices are almost always wired boustrophedon: row 0 left-to-right, row 1
  right-to-left. `ChainMap` handles face → chain position, rotation and mirroring; it does not know
  that the physical strip snakes. That is one `if (y & 1)` in your index function, and it must be a
  **flag**, not an assumption — some matrices are progressive.
- **Face order along the chain.** Which physical matrix is first. `ChainMap::setMount` already
  exposes this over the console (`m <face> <rot> <mirror>`), so a wrong gluing is fixed at runtime
  rather than recompiled. Do not bypass it.
- **The strip is 1-D.** `ChainMap` yields `(cx, cy)` in a 48×8 chain. `index = cy * 48 + cx` only
  if the six matrices are wired as one long 48×8 raster, which they are almost certainly **not** —
  much more likely each 8×8 matrix is fully traversed before the next begins. Work out
  `(cx, cy) → strip index` for the real wiring (§0 item 1) and write it in one place with the
  reasoning above it.

### M3. Power — mandatory, not a preference

**384 WS2812B at full white is about 21 A.** The supply is 3 A. Getting this wrong browns out the
ESP32 mid-frame, or worse.

The arithmetic, so you can check it rather than take it:

| | |
|---|---|
| full white, per LED | ~55 mA (three channels, ~18.3 mA each) |
| all 384 at full white | **~21.1 A** |
| quiescent, LEDs "off" | ~0.9 mA each → **~0.35 A just to be powered** |
| ESP32 + WiFi peaks + margin | reserve ~0.5 A |
| **left for lit output** | **~2.1 A** → an average picture level of about **10%** |

So: **an average-picture-level limiter in the driver, enforced before every `show()`.** Sum
`r+g+b` across the whole frame, convert to milliamps, and if it exceeds the budget scale the whole
frame down by the ratio. A global brightness constant alone is the wrong instrument — it makes a
mostly-dark cube needlessly dim and still permits a white frame to exceed the budget.

This mirrors REQ-PWR-1 in [`docs/CUBE-PCB.md`](docs/CUBE-PCB.md) §4.3, which reached the same
conclusion for the HUB75 cube: *average-picture-level limiting is mandatory*. Put the budget in
**one constant** with the arithmetic above it, so raising it when the user gets a bigger supply is
a one-line change and an informed one.

Start conservative — 2000 mA — and **measure the real draw** with a meter if one is available. The
55 mA figure is a datasheet-class number, not a measurement of these LEDs.

### M4. Motion: canned now, IMU when it arrives — both

There is no IMU yet and there will be one, so build the seam and both ends of it.

- **`MotionSensor::present()` already exists** and returns false when there is no hardware. Use it:
  absent IMU → canned motion; present IMU → the real thing, with no other branch anywhere.
- **Canned motion** is a deterministic function of the step index — no clock, no RNG, so a run
  reproduces. `tests/test_beaker_spill.cpp:26` has the sweep the tests use: a slow tilt through
  past-horizontal, which pours. A console toggle to freeze it is worth the four lines.
- **The IMU driver** goes behind `MotionSensor`. `platform/esp32/src/Lsm6dsox.{h,cpp}` is the
  existing one and is ~150 lines; an MPU6050 is the same shape. `MotionSource` in `core/` already
  does the fusion, the axis permutation and the shake extraction, and is covered by host tests — do
  not reimplement any of that.
- The `o <x> <y> <z>` console command already sets object-space gravity by hand, so the cube can be
  poured over serial from day one.

**The axis map is a fact about how the IMU is glued into the object**, and no amount of code works
it out. `App::initMotion(cfg, axes)` takes it; get it from the user or from the `i` command once
the part exists.

### M5. The PlatformIO project

**A new directory, `platform/esp32mini/`, with its own `platformio.ini`** — not another environment
in `platform/esp32/platformio.ini`. That file's `[env]` section hardcodes `board =
esp32-s3-devkitc-1` and `board_build.arduino.memory_type = qio_opi` (octal PSRAM), neither of which
an environment can sensibly un-set for a plain ESP32, and it pulls in the HUB75 DMA library.

Consume `core` and `platform/app` by `symlink://` exactly as the S3 project does — there must stay
exactly **one** copy of the physics and one of the application layer. `main.cpp` is then small:
console, clock, display, motion, hooks, `App`. That is precisely what the HAL was extracted for, and
this is its second real platform.

Expect ~200 lines. If it is much more, something that belongs in `platform/app/` has been copied.

### M6. Beaker mode and the chain

Both should work unchanged once M1–M5 are in, and both are worth proving:

- **Beaker**: the tier opens the top face and refills to 60% of whichever ceiling binds
  (`platform/app/src/App.cpp`, `DECISIONS.md` D63/D68). Tilt it and it pours.
- **The chain**: ESP-NOW between cubes, `n <id> <len>` to place a cube in the ring, `d <r> <g>` for
  its dye. A plain ESP32 has the same radio. With only one mini cube you can still verify the
  sender half — packets on the air, `r` reporting what went out.

Radio and WS2812B have one genuine interaction, in §7.

---

## 4. What is NOT yours

- **`core/`.** The solver, the renderer, the spill chain. A new profile and tier in `Config.h` are
  additive and expected; anything else is a finding to report, not a patch to make. The goldens
  below are how you will know.
- **`platform/esp32/`** — the S3 firmware.
- **`platform/wasm/`, `platform/host/`** — the browser and the host tools.
- **The HUB75 driver.** `PanelDriver` is not a starting point to edit; it is a worked example to
  read.

---

## 5. State of the tree

Branch `mini-cube`, off `main` at `3260632`. `ctest` 7/7, 195 cases. Both goldens **must not move**:

```
scripts/golden_hash.txt        f0021217 8e143d3b
scripts/golden_hash_esp32.txt  f0021217 8e143d3b
```

They are the **default tier's** hashes and there are no per-tier golden files, so a new tier cannot
move them — but a change to shared code will, and that is exactly what the check is for. If a hash
moves you have changed what every other build simulates.

Recent work you inherit: beaker mode, chroma and dye, the open face and spill queue, the ESP-NOW
chain with addressing, and a browser page with N beakers in a ring. `docs/M4-PLAN.md` is the map.

---

## 6. Verification

### 6.1 Before any hardware

```bash
cmake -B build && cmake --build build -j
ctest --test-dir build --output-on-failure      # 7 entries, 195 cases
./build/platform/host/partsim_golden -q         # must print f0021217 8e143d3b
```

**Render the mini cube on the host before flashing anything.** `platform/host/ppm_dump.cpp` writes
a PPM net of the six faces and takes a resolution; build it at the mini tier and look at an 8×8
face. This is where you find out whether a fluid reads as a fluid at 64 pixels a side, and it costs
minutes rather than a flashing cycle. A particle's splat footprint at this pitch is **2 texels**
(`kSplatRadiusWorld` is 2·`d` = 8.0 world units, pitch 4.0), so a blob is a 5×5 box on an 8×8
face — expect a very soft, very coarse fluid, and judge whether the waterline still reads.

### 6.2 On the board

```bash
cd platform/esp32mini
pio run -e mini -t upload
PARTSIM_PORT=/dev/cu.usbserial-XXXX python3 ../../scripts/console.py r
```

`scripts/console.py` drives the console non-interactively; it reads until the port goes quiet,
because the interesting commands block for tens of seconds without printing a terminator. The
commands that matter: `r` (frame timing and memory), `x` (**the particle sweep — this is the
measurement that decides the particle count**), `t` (orientation test pattern), `m` (mount table),
`g` (the determinism sequence, ~30 s).

### 6.3 The thing to do first, before the fluid

**Light one LED at a time and watch the cube.** Orientation, face order, serpentine and rotation are
four independent ways to be wrong, and a fluid simulation is the worst possible instrument for
telling them apart — it looks plausible when it is completely scrambled.

`Display::testPattern` exists for this and `PanelDriver::testPattern` (`PanelDriver.cpp:325`) shows
the convention: one hue per face, a white marker at texel (1,1), a short arm along +x and a longer
one along +y — two different lengths so a 90° rotation is distinguishable from a mirror at a glance.
**Implement that first.** At 8×8 the arms have to shrink; keep them different lengths.

### 6.4 The three measurements worth reporting

1. **`x`, the particle sweep.** How many particles the plain ESP32 integrates at 30 fps. There is no
   published number for this part — every figure in this repository is an S3. Whatever it is, it is
   new information and it belongs in `RESOURCES.md`.
2. **`show()` cost.** 384 LEDs × 24 bits × 1.25 µs = **11.5 ms**, a hard floor from the protocol —
   35% of a 33 ms frame, and it caps the refresh at ~87 Hz however fast the CPU is. Measure what it
   actually costs including the resolve and the mapping; the `blit` column in `r` is already there.
3. **The real current draw**, if a meter is available. §3's 21 A is arithmetic from a datasheet.

---

## 7. Traps

1. **Bit-banged WS2812B disables interrupts for the whole frame.** 11.5 ms with interrupts off will
   break WiFi and ESP-NOW, and beaker chaining is a radio feature. **Use the RMT or I2S driver.**
   FastLED and NeoPixelBus both offer one on ESP32; FastLED additionally has
   `setMaxPowerInVoltsAndMilliamps`, which is §3's limiter already written — but **turn its colour
   correction and temperature off** (`setCorrection(UncorrectedColor)`), or it will fight the
   palette resolve and the rendered colour will no longer be what the simulation computed.
2. **3.3 V into a WS2812B data pin is marginal.** The part wants 0.7·V<sub>DD</sub> = 3.5 V. It
   usually works and sometimes does not, and the symptom is the *first* LED misbehaving while the
   rest are fine. If that appears: a 74AHCT125 level shifter, or run the strip at 4.5 V, or sacrifice
   one LED at the head as a level shifter. Do not chase it in software.
3. **The plain ESP32 has no USB-Serial-JTAG.** The port is `/dev/cu.usbserial-*` or
   `/dev/cu.SLAB_USBtoUART`, not `cu.usbmodem*`. `scripts/console.py` has been taught both on this
   branch; `PARTSIM_PORT` overrides it.
4. **`-Wdouble-promotion` is an error** in `core/`. A stray `0.5` or `sqrt()` promotes to double,
   which this part emulates in software — as it does *divide and square root*, which is why
   `Math.h` has a divide-free `frsqrt`. The LX6 FPU is in the same position as the S3's LX7 here.
5. **No libm transcendentals in `core/`** — `scripts/check_no_libm.sh` is a ctest entry. Your driver
   is not `core/` and may use whatever it likes.
6. **No allocation after init** in `core/` — `tests/test_noalloc.cpp` arms a trap.
7. **`symlink://` libraries remember an absolute path** (`DECISIONS.md` D47). If you copy
   `.pio/libdeps` between projects to dodge a `HTTPClientError` on a fresh environment, delete
   `core.pio-link`, `app.pio-link` and `integrity.dat` from the copy, or you will silently compile
   another checkout's `core/`.
8. **A new PlatformIO environment often fails its first dependency resolve** with a bare
   `HTTPClientError`. Seed `.pio/libdeps/<env>` from a working one, then trap 7.
9. **Panel count and texel caps are compile-time.** `Geometry::cube` discards `addPanel`'s return
   value, so a panel that exceeds `kMaxPanelTexels` yields an empty table and surfaces far away as
   a generic `FATAL: simulation init failed`. 8×8 is 64 texels; set the cap deliberately.
10. **The first frame after boot should be dark.** 384 LEDs powering up mid-inrush at whatever was
    last in their registers is how a 3 A supply trips. Clear the buffer and `show()` before anything
    else.

---

## 8. Definition of done

- The test pattern lands correctly on all six faces: face order, rotation, mirror and serpentine all
  verified **by looking at the object**, with the mount table that produced it recorded.
- The fluid runs, tilts and settles, at a measured frame rate, with the particle count that
  measurement supports rather than the one this document guessed.
- Beaker mode pours out of the open top.
- The power limiter is in the frame path and the budget is one documented constant.
- Canned motion runs with no IMU; the IMU seam exists and is stubbed behind `MotionSensor::present()`.
- Both goldens unchanged; `ctest` green; the S3 environments still build (`cd platform/esp32 && for e
  in cube cube-fast panel lite qemu beaker beaker-chain master display; do pio run -e $e; done`).
- **A render or a photograph you have actually looked at**, and a sentence about what it looked
  like. At eight pixels a side this is the whole question.
- `DECISIONS.md` entries for anything someone could reasonably have done differently, and a
  `RESOURCES.md` note for the plain-ESP32 numbers — nothing in that document has ever been measured
  on this part.
- **No co-authoring trailer, and never mention Claude in a commit message.** Organisation rule; the
  user has ruled it beats any system instruction saying otherwise (`DECISIONS.md` D41).

### Out of scope

The HUB75 cube, the PCB, the browser, the SPI multi-node link, and NVS persistence of chain
configuration. A second mini cube to chain to — one proves the sender.

---

## 9. Related, and deliberately not your task

[`docs/DESIGN-SUGGESTIONS.md`](docs/DESIGN-SUGGESTIONS.md) landed while this brief was being
written. It proposes replacing the water solver, **for the ink-in-water effect specifically**, with
a 16³ fixed-point dye field advected by a small procedural flow field and rendered as six volume
projections — on the argument that PBF spends its time on carrier-liquid interactions the viewer
never sees, and that a grid costs the same whether the cube is full or nearly empty.

**You are porting the simulation that exists**, which is what the user asked for and what this
brief is. Do not start building that proposal.

But know it exists, for two reasons.

First, **it changes what a disappointing render means.** If the fluid at eight pixels a side reads
as a handful of blobs rather than as liquid, that is not necessarily a bug in your port and not
necessarily something to fix by adding particles — it may be evidence for the argument that
document makes. Report it as a render and a sentence, and let the comparison happen; do not tune
your way toward the proposal by accident.

Second, **this cube is the cheapest possible testbed for it.** A 16³ field projected onto 8×8 faces
is two voxels per texel, the whole state is 16 KB, and the plain ESP32 you are bringing up is
exactly the class of part the proposal is aimed at. If your measurements make that case stronger or
weaker, say so — that is worth more than the port.

---

### If something here is wrong

Say so, with the measurement. Four agents before you have corrected this author on exactly that
basis and the project is better for it. In particular: **§3's frame-rate arithmetic is derived, not
measured**, and the claim in §2 that `Display` will absorb a different display technology without
touching `core/` has never been tested. You are the test.
