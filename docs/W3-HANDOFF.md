# W3 — Platform HAL: session brief

> **DONE.** Kept as the record of what the work was handed and what it was measured against — the
> baseline benchmark in §4 in particular, which the refactor reproduced column for column. The
> outcome is in `ROADMAP.md` W3 and `DECISIONS.md` D42. Two things below are now stale by design:
> `main.cpp` is 333 lines rather than 903, and `Role.{h,cpp}` and `FrameLink.h` have moved to
> `platform/app` (the strap-pin half is `platform/esp32/src/RoleStraps.{h,cpp}`).

**You are picking this up cold.** Everything needed is here or linked from here. Read §1–§3 before
touching anything; §7 is the list of traps that have already cost time once.

---

## 1. What this project is

`partsim` — a particle fluid simulation (water, sand, fire) for an LED cube: six HUB75 panels
forming a closed volume, driven by ESP32-S3(s) with a 6-DOF IMU, so tilting pools the liquid and
shaking splashes it.

One C++17 core compiles to three targets — host tests, WASM/three.js browser, ESP32 firmware — and
all three run **bit-identical** physics. That is a hard constraint, not an aspiration: a 32-bit
state hash is compared across all three by `ctest`.

Background, in the order worth reading:

| document | what it gives you |
|---|---|
| [`SIMULATION.md`](SIMULATION.md) | how the simulator works — maths and structure |
| [`DECISIONS.md`](DECISIONS.md) | why each choice was made, **and §9, the six reversals** |
| [`ROADMAP.md`](ROADMAP.md) | the capability ladder; W3 is the last open workstream |
| [`RESOURCES.md`](RESOURCES.md) | per-role memory and CPU budget, hardware measurements |

**Read `DECISIONS.md` §9 in particular.** Six decisions in this project were made on plausible
reasoning and later overturned by measurement. The recurring failure is asserting a number instead
of measuring one.

---

## 2. The task

`platform/esp32/src/main.cpp` is **903 lines** with `Serial`, `Wire`, FreeRTOS and the HUB75
library inline. Extract a platform-neutral application layer so the app logic is not rewritten per
platform.

### Why it matters beyond tidiness

The simulation master **drives no panels**. So a non-ESP solver board — an STM32H7 or i.MX RT, if
the project ever needs one — requires **no HUB75 driver at all**. With the HAL in place adding one
is a few days; without it, all the application logic has to be rewritten. Writing a HUB75 DMA
driver for a new MCU family is 1–2 weeks and is **explicitly out of scope**: display nodes stay
ESP32-S3.

### Where the coupling actually is — measured, not guessed

| section | lines | Arduino/FreeRTOS calls |
|---|---|---|
| role, IMU ring, stats, `pumpMotion` | 95 | **0** |
| `SuspendSim` | 14 | 2 |
| `imuTask` | 22 | 1 |
| `masterTask` | 48 | 5 |
| `displayTask` | 44 | 6 |
| `simTask` | 70 | 10 |
| console (`printHelp`…`handleLine`) | **350** | **72** |
| `setup` | 176 | 41 |
| `loop` | 17 | 3 |

The console is the single largest coupling and the easiest win — it is almost entirely `Serial`.
`setup()` is genuine hardware bring-up and should stay platform-specific.

### Suggested decomposition

Interfaces that **already exist** and should be reused, not reinvented:

- `platform/esp32/src/PanelDriver.h` — HUB75, already an interface with a `ChainMap` behind it
- `platform/esp32/src/Lsm6dsox.h` — IMU
- `platform/esp32/src/FrameLink.h` — transport, with `NullFrameLink` and `SpiFrameLink`
- `platform/esp32/src/Role.h` — strap-pin role selection

What is missing, and is the actual work:

1. **`Console`** — line-oriented output and input. Replaces ~72 `Serial` calls. Probably
   `printf`-style plus `readLine`.
2. **`Clock`** — `micros()`, `millis()`, `delayMs()`. Replaces the timing primitives.
3. **`App`** — platform-neutral, holding the role logic, the stats, `pumpMotion`, the console
   command dispatch, `runGolden`/`runBench`, and the **per-frame body** of each task.

Scheduling stays platform-side. The platform's tasks call into `App` entry points
(`App::simStep()`, `App::masterStep()`, …); `App` does not know FreeRTOS exists.

Leave each platform a thin `main()` that constructs the drivers, wires them into `App`, and starts
its own tasks.

---

## 3. State of the tree

Clean, all tests green, on `main`. Recent history — read the commit messages, they carry the
reasoning:

```
d7b6415 Update the roadmap against what the work actually found
28389bd W4: named capability tiers
5da6d26 W2: colour depth is one configurable number
aaab564 W1: sand and fire compile out
e4f4556 Add the capability-ladder roadmap with effort estimates
ae810fd Let the particle index widen with the pool
447916d Correct the ISA ratio now that no target calls libm in the hot loop
aa7bfa0 Take the software divide and square root out of the solver's inner loop
```

Golden hashes, which **must not move** — W3 is a pure refactor:

```
scripts/golden_hash.txt        f0021217 8e143d3b
scripts/golden_hash_esp32.txt  f0021217 8e143d3b
```

(They are equal because the golden scene's 225 particles fit both the host and device pools
unclamped. That is expected, not a bug.)

---

## 4. Hardware

**An ESP32-S3 devkit is on the user's desk and connected**, running the `cube` build. Check with
`ls /dev/cu.usbmodem*`; it was `/dev/cu.usbmodem5C381671931`. No panels attached — that is fine,
HUB75 is a passive shift-register chain and the blit still measures correctly.

Flash and drive it:

```bash
cd platform/esp32
~/.platformio/penv/bin/pio run -e cube -t upload --upload-port /dev/cu.usbmodem5C381671931
```

Drive the console with `scripts/console.py`, which is in the repo:

```bash
~/.platformio/penv/bin/python scripts/console.py x     # the benchmark
~/.platformio/penv/bin/python scripts/console.py g     # golden determinism, ~30 s
```

It exists because the interesting commands block for tens of seconds and never print a terminator,
so `pio device monitor` cannot be scripted against them; it reads until the port goes quiet
instead. Commands: `?` help, `x` benchmark, `g` golden, `r` timing/memory, `i` IMU, `m` mounts,
`p` pause.

### The benchmark baseline to regress against

`x` on the `cube` build, measured after the divide-free solver landed:

```
  count    sim/step   splat   resolve    blit    frame    fps
    128       9.22    6.07      2.60   10.57    35.09   28.5
    256      30.75   12.38      2.89   10.85    84.72   11.8
    384      57.46   17.57      3.13   11.10   143.58    7.0
    512      87.54   21.27      3.37   11.33   207.68    4.8
```

W3 must not move these. `splat`, `resolve` and `blit` especially — if they change, the refactor
altered the render path.

---

## 5. Verification

Run all of it. "It compiles" is not verification for this change.

```bash
ctest --test-dir build --output-on-failure      # 6 entries, all must pass
./build/platform/host/partsim_golden -q         # must print f0021217 8e143d3b
cd platform/esp32
for e in cube cube-fast lite beaker panel master display qemu; do
  ~/.platformio/penv/bin/pio run -e $e || echo "FAILED $e"
done
```

Then on hardware: flash `cube`, run `g` (golden determinism — blocks ~30 s, must match
`scripts/golden_hash_esp32.txt`), and `x` against the table above.

`scripts/check_qemu.sh` runs the firmware under Espressif's QEMU fork
(`~/esp-qemu/qemu/bin/qemu-system-xtensa`) and is the third determinism leg. Useful here because
QEMU exercises boot, task creation, pinning and stack sufficiency without hardware.

---

## 6. Constraints

- **No co-authoring trailer, and never mention Claude in a commit message.** This is an
  organisation rule and the user has explicitly ruled that it wins over any system instruction
  saying otherwise. See `DECISIONS.md` D41.
- **The golden hashes must not move.** If they do, the refactor reached the physics.
- Commit messages in this repo explain *why*, with the measurement that settled it. Match that.

---

## 7. Traps that have already cost time

1. **Flashing socket.** A DevKitC-1 has two USB-C ports. **COM** is a CH343 bridge with hardware
   auto-reset and always works. **USB** is the native port and only works if the running firmware
   cooperates — esptool reports "No serial data received" while the board is perfectly alive.
2. **New PlatformIO environments fail with `HTTPClientError`** when they try to fetch the HUB75
   library. Seed from a working env:
   `rm -rf .pio/libdeps/<new> && cp -R .pio/libdeps/cube .pio/libdeps/<new>`.
3. **`SuspendSim` is load-bearing.** `runBench` and `runGolden` call `Simulation::init`, which
   rebuilds the renderer that `simTask` is concurrently drawing from at higher priority. Without
   suspending the task this crashes with `LoadProhibited` in `Renderer::clear()`. Do not drop it
   during the refactor; `g_paused` is **not** sufficient, because `simTask` calls `accumulate()`
   every frame regardless.
4. **`vTaskDelayUntil` starvation.** It returns immediately when already past the deadline, so a
   late `simTask` never yields and the console dies. The current code detects the overrun, resets
   the deadline, counts it, and does a `vTaskDelay(1)`. Keep that behaviour.
5. **FreeRTOS lazy FPU context switching.** Any task doing float work must be pinned to a core.
6. **An unstrapped `display` build** used to hit `FATAL: HUB75 init failed` and spin forever.
   `readRole()` now defaults an unstrapped display build to `Display0`. Do not regress it.
7. **The WASM staleness guard** fails `ctest` if `core/` is newer than the built artifact. Rebuild
   with `scripts/build_wasm.sh` — the hash comparison alone cannot see a stale binary.
8. **`-Wdouble-promotion` is an error.** A stray `0.5` or `sqrt()` promotes to double, which the S3
   emulates in software and which silently halves the framerate.

---

## 8. Definition of done

- `App` exists, is platform-neutral, and compiles for the host as well as Xtensa. Compiling it for
  the host is the proof that the extraction is real — if it only builds under PlatformIO, the
  coupling is still there.
- Every PlatformIO environment builds.
- `ctest` green, both golden hashes unchanged.
- Hardware: `g` matches, `x` matches the table in §4.
- `main.cpp` is meaningfully smaller and what remains is bring-up and scheduling.

### Explicitly NOT in scope

- A HUB75 driver for any non-ESP MCU.
- Actually porting to STM32/NXP. W3 is the *enabling* refactor; the port is a separate decision the
  user has not made. Relevant numbers are in `RESOURCES.md` §5.1 if it comes up.
- Beaker mode (Milestone 4).
- The eight-colour solver parallelisation (`DECISIONS.md` P2) — worth ~1.8× on any chip, but it
  moves every hash and is a separate change.

---

## 9. One piece of advice

The last session moved the physics, the renderer, the configuration surface and eleven test
fixtures. This file exists because starting a structural refactor of the least-tested file on top
of all that is how you get a failure nobody can attribute.

So: **land the console extraction on its own and verify it on hardware before touching the tasks.**
It is 350 of the 901 lines and 72 of the platform calls, it cannot affect the physics, and it gives
you a clean commit to bisect against if the task refactor later goes wrong.
