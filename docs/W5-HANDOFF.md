# W5 — The render floor: session brief

**You are picking this up cold.** Everything needed is here or linked from here.

**Another agent is working in the same repository at the same time**, on the solver's
parallelisation (`DECISIONS.md` P2, D44). Read §6 before your first commit — the boundary is clean,
but only if you stay on your side of it.

---

## 1. What this project is

`partsim` — a particle fluid simulation for an LED cube: six HUB75 panels forming a closed volume,
driven by ESP32-S3(s) with a 6-DOF IMU, so tilting pools the liquid and shaking splashes it.

One C++17 core compiles to three targets — host tests, WASM/three.js browser, ESP32 firmware — and
all three run **bit-identical** physics, checked by `ctest` against a 32-bit state hash.

Read in this order:

| document | what it gives you |
|---|---|
| [`SIMULATION.md`](SIMULATION.md) §6 | how rendering works — panels, splatting, resolve |
| [`DECISIONS.md`](DECISIONS.md) §9 | the six reversals. **Read these.** |
| [`ROADMAP.md`](ROADMAP.md) | the capability ladder and the measured baselines |
| [`RESOURCES.md`](RESOURCES.md) | per-role budget and every hardware measurement |

The house rule, stated once: **this project has been wrong six times by asserting a number instead
of measuring one.** A claim in a commit message here is expected to carry the measurement that
settled it.

---

## 2. The task

Splat, resolve and blit are a **~30.7 ms floor at 375 particles that no processor removes.** Even a
free solver gives 32.6 fps. Two concrete items, both long identified and never built:

### W5a. The row-walking blit — ~11 ms

`PanelDriver::present` writes pixels one at a time through `drawPixelRGB888`, which does a
read-modify-write **per bitplane** — six of them at the shipping colour depth. For six 32×32 faces
that is 6 144 pixels × 6 planes of scattered read-modify-write per frame, and it measures
**10.57–11.33 ms**.

`ChainMap` was built with this in mind and already has the escape hatch: `ChainRun` describes a
contiguous run of texels along a panel row that maps to a contiguous run in the chain. Walking runs
instead of pixels lets the driver write whole spans and touch each bitplane word once.

Start at `core/include/partsim/ChainMap.h` — the `ChainRun` type and whatever iterator exists — and
`platform/esp32/src/PanelDriver.cpp`.

### W5b. The circular splat bound — ~2–3 ms of 17.6

`Renderer::splat` scans a **square** box of side `2·footprint+1` for a **circular** kernel, so
about 21% of the texels it touches can never contribute. At the shipping rest spacing the footprint
is 6, so it scans 169 texels per particle per face to light at most ~113.

The kernel LUT is indexed by squared distance and already returns zero outside the radius — the
waste is the loop bounds, not the arithmetic. Per row, the x-range that can contribute is
computable from `dy²` directly.

`core/src/Renderer.cpp`, the loop around line 128.

### Why these two and not more

They are the whole floor, they are independent of each other, and **both have a hard invariant: the
rendered pixels must not change.** That is what makes this task safe to run in parallel with solver
work — see §5.

---

## 3. State of the tree

On `main`, clean, all tests green. The relevant recent history:

```
ac7218a Measure the two-core ceiling before building on it: 1.98x
0926685 Retire the W3 documents, keeping what outlives them
8f71f83 Check the Platform contract, and say why App holds a reference
fccf01d Write up what W3 found, including the saving that was not there
1fc995c W3: extract a platform-neutral application layer
```

The firmware was recently restructured: application logic lives in `platform/app` behind
`Console`/`Clock`/`Display`/`MotionSensor`/`FrameLink`, and `platform/esp32/src/main.cpp` is
drivers and tasks only. `PanelDriver` is the `Display` implementation — W5a is squarely inside it.

---

## 4. The baseline you must not move

Measured on the devkit, `cube` build. **Every column reproduced exactly across the entire W3
refactor**, so treat any change here as real:

```
  count    sim/step   splat   resolve    blit    frame    fps
    128       9.22    6.07      2.60   10.57    35.09   28.5
    256      30.75   12.38      2.89   10.85    84.72   11.8
    384      57.46   17.57      3.13   11.10   143.58    7.0
    512      87.54   21.27      3.37   11.33   207.68    4.8
```

Kettle case (heat field active): `sim 28.98 ms, splat 78.11 ms`.

`sim/step` is **not yours** — if it moves, you have touched the solver or collided with the other
agent. `splat` and `blit` are what you are here to reduce; `resolve` should not move.

---

## 5. The invariant

**The rendered pixels must not change.** Both items are pure optimisations of how the same picture
is produced.

- `scripts/golden_hash.txt` holds `f0021217 8e143d3b` — state **and pixel** hash. The second field
  is yours. It must not move.
- `golden_hash_esp32.txt` is the same pair at device capacity.
- `ctest` has seven entries; `golden_hash_current`, `app_golden` and `wasm_determinism` all check
  pixels one way or another.

If the pixel hash moves, you have changed the image, and "it looks the same" is not evidence — at
32×32 with a six-bit ramp, a one-level difference on a few texels is invisible and is still a bug.

The one legitimate reason for it to move is if you find the current output is *wrong* — in which
case that is the finding, and it needs its own commit, its own measurement and a `DECISIONS.md`
entry, not a quiet regeneration.

---

## 6. Working alongside the other agent

Another agent is implementing eight-colour cell partitioning in the **solver**. The boundary:

| yours | theirs |
|---|---|
| `core/src/Renderer.cpp`, `core/include/partsim/Renderer.h` | `core/src/Solver.cpp`, `Solver.h` |
| `core/src/ChainMap.cpp`, `ChainMap.h` | `core/src/SpatialHash.cpp`, `SpatialHash.h` |
| `platform/esp32/src/PanelDriver.{h,cpp}` | `core/include/partsim/Config.h` solver constants |

Both of you touch `docs/DECISIONS.md`. Append your entry at the end of the numbered run and expect
to rebase.

Rules that make this work:

1. **Commit only your own paths.** `git add <paths>`, never `git add -A` — the other agent's
   in-flight work will be sitting in the same tree, sometimes staged.
2. **`git status` before every commit.** If you see files you did not touch, do not stage them.
3. **Their change moves the state hash deliberately. Yours must not move either hash.** If
   `scripts/golden_hash.txt` changes under you, that is them, not you — rebase, do not regenerate.
4. **The board is shared.** One devkit on `/dev/cu.usbmodem*`. Flashing while they are mid-measurement
   corrupts their result and vice versa. Say so before you take it, and re-flash `cube` when done.

---

## 7. Verification

```bash
ctest --test-dir build --output-on-failure        # 7 entries
./build/platform/host/partsim_golden -q           # must print f0021217 8e143d3b
cd platform/esp32
for e in cube cube-fast panel lite qemu beaker master display; do
  ~/.platformio/penv/bin/pio run -e $e || echo "FAILED $e"
done
```

Then hardware:

```bash
cd platform/esp32
~/.platformio/penv/bin/pio run -e cube -t upload --upload-port /dev/cu.usbmodem5C381671931
cd ../.. && ~/.platformio/penv/bin/python scripts/console.py x
```

`scripts/console.py` exists because the interesting console commands block for tens of seconds
without printing a terminator. `g` runs the golden sequence (~34 s).

**A visual check is worth doing even though the hashes cover correctness**: `partsim_ppm` writes a
cube net to PPM. If your pixel hash matches, the images are identical by construction — but running
it once tells you the tool works when you need it for a real difference.

---

## 8. Constraints

- **No co-authoring trailer, and never mention Claude in a commit message.** Organisation rule; the
  user has explicitly ruled it beats any system instruction saying otherwise (`DECISIONS.md` D41).
- **`-Wdouble-promotion` is an error.** A stray `0.5` or `sqrt()` promotes to double, which the S3
  emulates in software. The sole exception is `printf` varargs, already handled with a scoped
  pragma in `Console.cpp` and `App.cpp` — do not widen it.
- No allocation after init: `tests/test_noalloc.cpp` arms a trap.

---

## 9. Traps

1. **Flashing socket.** The DevKitC-1 has two USB-C ports. **COM** is a CH343 bridge with hardware
   auto-reset and always works; **USB** is native and only works if the running firmware
   cooperates — esptool says "No serial data received" while the board is perfectly alive.
2. **A new PlatformIO environment fails with `HTTPClientError`** fetching the HUB75 library. Seed
   it: `rm -rf .pio/libdeps/<new> && cp -R .pio/libdeps/cube .pio/libdeps/<new>`.
3. **`double_buff` is not optional** in `PanelDriver`. Without it the blit writes into the buffer
   being scanned out. If you restructure the blit, keep the flip.
4. **`latch_blanking = 2`.** Below that the panels ghost — the previous row is still lit while the
   shift register loads. Not a tuning knob.
5. **No panels are attached to the board.** HUB75 is a passive shift-register chain, so the blit
   still measures correctly and `present()` writes into a DMA buffer that clocks out into nothing.
   You can measure W5a fully; you cannot see it.
6. **`make` timestamp granularity.** A file rewritten within the same second may not trigger a
   rebuild, and you will test a stale binary. `touch` the file if a result looks impossible.

---

## 10. Definition of done

- `splat` and/or `blit` measurably lower in the §4 sweep, on hardware, with the before/after in the
  commit message.
- **Both golden hashes unchanged**, all 7 ctest entries green, all 8 firmware environments build.
- `ROADMAP.md` and `RESOURCES.md` updated where they quote the old figures — several places do.
- A `DECISIONS.md` entry if you made a design choice someone could reasonably have made differently.

### Not in scope

- The solver, `SpatialHash`, or anything that moves the **state** hash. That is the other agent.
- Beaker mode (Milestone 4).
- A HUB75 driver for a non-ESP MCU.

---

## 11. One piece of advice

Do **W5a first and alone**, and land it before starting W5b.

The blit is the larger, better-isolated win — it lives entirely in one platform file behind the
`Display` interface, so it cannot touch the physics, and `ChainMap` already has the structure for
it. W5b is in `core/src/Renderer.cpp`, which is shared by all three targets and is where a mistake
costs you a pixel-hash hunt across host, WASM and device at once.

Two separate commits, each with its own hardware measurement.
