# W3 — Platform HAL: findings

What the platform-HAL extraction actually found, as distinct from what it was expected to find.
The brief it was handed is [`W3-HANDOFF.md`](W3-HANDOFF.md); the decision record is
[`DECISIONS.md`](DECISIONS.md) D42; the ladder entry is [`ROADMAP.md`](ROADMAP.md) W3.

Implementation commit: `1fc995c`, on top of `720edc6`.

House rule, and the reason this file exists: **six decisions in this project were made on plausible
reasoning and later overturned by measurement** (`DECISIONS.md` §9). Two of the findings below are
in that shape — a saving that sounded obvious and was not there, and a cost that was assumed to be
zero and is not. Both are recorded with the numbers rather than the adjective.

---

## 1. The shape, measured

| | before | after |
|---|---|---|
| `platform/esp32/src/main.cpp` | 903 lines | **332** |
| application logic that compiles for the host | 0 lines | **1,218** (`platform/app`: 625 source, 593 header) |
| platform headers included by the application layer | — | **0** |

`platform/app` includes exactly two kinds of header: `<cstdarg>`, `<cstddef>`, `<cstdint>`,
`<cstdio>`, `<cstdlib>`, and `partsim/...`. No `Arduino.h`, no FreeRTOS, no `esp_*`.

That is a grep rather than a claim, and worth re-running after any change here:

```bash
grep -rnE "Arduino|Serial|vTask|xTask|TickType|heap_caps|digitalRead|freertos" \
  platform/app/src platform/app/include
```

It returns **four** hits today, all of them prose: two in `Console.h` explaining what the interface
replaced, two in `App.h` explaining what deliberately stayed platform-side. A fifth hit that is
not a comment is a regression.

What remains in `main.cpp` is 49 lines that touch Arduino, FreeRTOS or `esp_*`, all of them in one
of two jobs: constructing drivers, and starting tasks.

The handoff's coupling table, resolved:

| section | lines | platform calls | where it went |
|---|---|---|---|
| role, IMU ring, stats, `pumpMotion` | 95 | 0 | `App`, unchanged |
| `SuspendSim` | 14 | 2 | split: `App::SuspendSim` + `SystemHooks::suspendSim` |
| `imuTask` | 22 | 1 | `App::imuPoll` + an 8-line task |
| `masterTask` | 48 | 5 | `App::masterStep` + a 7-line task |
| `displayTask` | 44 | 6 | `App::displayStep` + a 7-line task |
| `simTask` | 70 | 10 | `App::simStep` + a 7-line task |
| console | 350 | 72 | `App`, behind `Console` |
| `setup` | 176 | 41 | stayed — it is genuine bring-up |
| `loop` | 17 | 3 | two lines |

---

## 2. Findings

### F1. A HAL with one implementation is a rename, not a seam

This is the finding that shaped everything else. Extracting interfaces and leaving exactly one
implementation behind each of them proves nothing: the coupling survives in whatever the single
implementation happens to assume, and nobody discovers it until the port — which is the moment the
whole exercise exists to de-risk.

So the host build is not a bonus, it is the check.
[`platform/host/console_main.cpp`](../platform/host/console_main.cpp) is a real second platform in
119 lines — stdio for `Console`, `std::chrono` for `Clock`, `NullDisplay`, `NullMotionSensor`,
`NullFrameLink` — and the new `app_golden` ctest drives the application layer's **own `g` command**
there, comparing the state hash against `scripts/golden_hash.txt`.

It works: the host build of `App::runGolden` printed `f0021217` first time. An extraction that
reached the physics would have said so with no board attached.

The 119 lines are the price of the check. That is cheap enough that there is no argument for
skipping it, which is the general form of the finding.

### F2. The refactor costs SRAM; it does not save it

Expected to be neutral. Measured before and after, per environment, by building `HEAD` in a
detached worktree with the same toolchain:

| env | before | after | delta |
|---|---|---|---|
| `cube`, `cube-fast`, `panel` | 171,660 | 171,740 | **+80** |
| `lite` | 143,580 | 143,660 | **+80** |
| `qemu` | 171,616 | 171,696 | **+80** |
| `beaker` | 172,176 | 172,288 | **+112** |
| `master` | 192,064 | 192,176 | **+112** |
| `display` | 115,928 | 116,760 | **+832** |

Vtables for six interfaces, two 64-byte line buffers, and members that can no longer be
dead-stripped because a class holds them rather than a translation unit. Against a 230 KB budget
this is noise and it changes no decision — but it is the wrong direction, and `+832` on the node
with the tightest budget is the number a future 64×64 display build should know about rather than
rediscover.

### F3. The 12 KB that was not there

`runBench` needs one face of RGB staging, `kMaxPanelTexels * 3`. A display node runs no benchmark,
so guarding that buffer out of the display build looks like **12 KB off the tightest budget in the
project** — a saving obvious enough to write into a comment, which is exactly what happened.

It is not there. The old version was a file-scope static in an anonymous namespace, unreferenced in
a display build, and already dead-stripped. The measured display delta is `+832`, not `-11,456`.

The guard is kept because it is *true*, not because it buys anything, and the comment in
[`App.h`](../platform/app/include/partsim/app/App.h) now says so. Recorded here because the failure
mode is the house one: a plausible number, asserted, never built.

### F4. The benchmark did not move at all

The real risk in this change was disturbing the render path. It did not happen — every column of
the `x` sweep reproduced on the same board, to the hundredth of a millisecond:

```
  count    sim/step   splat   resolve    blit    frame    fps
    128       9.22    6.07      2.60   10.57    35.09   28.5
    256      30.75   12.38      2.89   10.85    84.72   11.8
    384      57.46   17.57      3.13   11.10   143.58    7.0
    512      87.54   21.27      3.37   11.33   207.68    4.8
```

Identical to `W3-HANDOFF.md` §4, `splat`/`resolve`/`blit` included, and the kettle case too
(`sim 28.98 ms, splat 78.11 ms`). One virtual call per frame on `Display::present` and one per IMU
sample on `MotionSensor::read` cost nothing that this instrument can see — which is the answer to
the only performance question the design raised.

### F5. Null implementations dissolved the conditional compilation

`PanelDriver` already guards every entry point on its DMA pointer, so a board that never calls
`begin()` gets a driver that does nothing and reports `ready() == false`. That means it can be
handed over unconditionally — and every `#if !PARTSIM_QEMU` wrapped around a `present()` call
disappeared with no behaviour change, because the QEMU build was already getting exactly that.

Generalised into the interface contract: **a platform that lacks a device passes the Null
implementation, never a null pointer**, so the application has no absent-hardware branch to get
wrong. Before this, the master role, the QEMU environment and the single-panel build each carried
their own `#if` around the same call sites.

### F6. Moving code found two pieces of dead code

Both harmless, both the kind of thing only a move surfaces:

- A display node's draw-only state init (`rxRenderer_`, `rxHeat_`, `rxParticles_`) sat inside
  `setup()`'s `#if !PARTSIM_QEMU` branch. None of it touches hardware, and `display` and `qemu` are
  mutually exclusive environments — `qemu` builds the `cube` profile. It is now unconditional.
- `g_staging` in a display build — see F3.

### F7. What deliberately did *not* move

`vTaskDelayUntil` returns immediately once its deadline has passed, so a task that cannot hit its
frame period stops yielding entirely and the priority-1 console never runs again: the device looks
hung when it is merely late (handoff trap 4). The fix — detect the overrun, reset the deadline,
count it, `vTaskDelay(1)` — is specific to FreeRTOS tick semantics and does not generalise to a
platform whose scheduler works differently.

So it stayed platform-side, and the refactor's contribution is that it exists **once**, in
`frameYield`, rather than copied into three tasks. `App` gets told about it through
`noteOverrun()`.

This is the seam's actual rule, and it is narrower than "App owns the logic": **App owns a frame;
the platform owns when a frame runs.**

One deliberate behaviour difference falls out of it. The test-pattern frame used to take a plain
`vTaskDelayUntil`, bypassing the overrun path; it now goes through `frameYield` like every other
frame. Identical when on time, and it counts an overrun when the pattern blit blows the deadline —
which is accurate.

### F8. `suspendSim` is interface, not implementation

`runGolden` and `runBench` call `Simulation::init`, which rebuilds the renderer's slot tables and
every pool while a higher-priority task is drawing from them. Pausing is **not** sufficient and
never was: that task calls `accumulate()` every frame regardless of the pause flag. Dropping the
suspend reproduces a `LoadProhibited` inside `Renderer::clear()`.

None of that is visible at the call site, so it is documented where `SystemHooks::suspendSim` is
*declared* rather than where the ESP32 implements it — and the host's empty implementation says in
a comment that emptiness there is a decision (one thread, no race), not an oversight.

### F9. Scale factors belong next to the register write that justifies them

`MotionSensor` could have declared `kAccelScaleG` itself. It does not: it asks the driver via
`accelScaleG()`, and `Lsm6dsox` answers with its own constant — the one sitting next to the
`CTRL1_XL` write and the `static_assert` tying the two together.

A copy in the application layer would be a second place for a range change to be missed, and the
failure it produces (a truncated gravity vector during exactly the hand-shake the object exists
for) is invisible in every unit test. Same reasoning as `-DPARTSIM_PROFILE_*` and `Config.h`
(D36).

Related: `Lsm6dsox::Raw` is now `using Raw = partsim::app::ImuSample` rather than a second struct
with the same six fields in the same order. Two structurally identical types stay correct until
somebody reorders one of them.

### F10. The role split is GPIO versus meaning

`Role.h`/`Role.cpp` split cleanly in two, and the line is sharper than "platform vs not":

- **What a role means** — the enum, `roleName`, `roleDrivesPanels`, and `facesFor` (which face pair
  each display node owns) — is a fact about *the cube*, and moved to `platform/app`.
- **Reading the strap pins** is two `digitalRead`s and a settle delay, and stayed in
  `platform/esp32/src/RoleStraps.{h,cpp}`.

Including the display-build default: an unstrapped `display` build must not read as master, or
`facesFor` returns zero faces and `PanelDriver::begin` fails with "check Pins.h against the wiring"
— sending someone after a wiring fault that does not exist (handoff trap 6). That is strap-reading
policy, so it stayed with the straps.

### F11. `printf` is the one place `-Wdouble-promotion` cannot be obeyed

The flag is an error project-wide because a stray double in the solver halves the framerate on a
chip that emulates them in software. Varargs promote `float` to `double` **by definition**, so
every console call site trips it and there is nothing to fix.

Suppressed with a scoped `#pragma` in `Console.cpp` and `App.cpp` only — the same treatment the
original console had — so the warning keeps working everywhere it can still find a real bug.

`Console::printf` formats into a 256-byte stack buffer via `vsnprintf` and truncates. No line the
console prints is close to that, and truncation is a far better failure than a heap allocation in a
steady-state path (`tests/test_noalloc.cpp`). `Serial.printf` does the same thing internally, which
is why output is byte-identical.

Dropping Arduino's `F()` macro is free: on ESP32 flash is memory-mapped and `F()` is effectively
identity, unlike on AVR where it is load-bearing.

---

## 3. Handoff traps: what actually happened

| # | trap | outcome |
|---|---|---|
| 1 | flashing socket (COM vs USB) | not hit — the board was already on the working port |
| 2 | new PlatformIO env → `HTTPClientError` | **not hit.** No new environment; `symlink://../app` resolves locally and triggers no fetch |
| 3 | `SuspendSim` is load-bearing | preserved and promoted to interface — see F8 |
| 4 | `vTaskDelayUntil` starvation | preserved, now in one place — see F7 |
| 5 | FPU context switching / pinning | unchanged; QEMU re-verified task creation and pinning |
| 6 | unstrapped `display` build | preserved in `RoleStraps.cpp` — see F10 |
| 7 | WASM staleness guard | not hit; `wasm_determinism` green throughout |
| 8 | `-Wdouble-promotion` is an error | hit exactly where expected — see F11 |

---

## 4. Two things that looked like regressions and were not

**100% overruns on the `cube` build at scene 0.** `r` reports `fps 6.9, sim 113.22 ms,
frames 766, overruns 766`. This is pre-existing and is precisely what the benchmark table already
says: at 375 particles the frame costs ~113 ms against a 33 ms budget, and the sweep's verdict
column reads `OVER` at every point with "largest sweep point that fits: 0 particles". The console
stayed responsive throughout, which is trap 4's fix working.

**`b 40` printed "unknown command".** `scripts/console.py` treats **each argv as a separate
command**, so that sent `b` (which needs `argc >= 2` and correctly does nothing) and then `40`.
Multi-word commands must be quoted: `console.py 'b 40'`, the way `'s 2'` already was in the
docstring. Verified working once quoted, on hardware and host.

---

## 5. What a non-ESP solver board would now need

This is the point of the work, so it is worth stating concretely. It remains a decision the user
has not made, and the numbers live in `RESOURCES.md` §5.1, under *Choosing the MCU that runs the
solver* — an STM32H743 is projected at 2.9x the S3 and an i.MX RT1176 at 6.3x, both marked
*assumed* rather than measured.

Implement, for a board that solves and drives no panels:

| interface | effort |
|---|---|
| `Console` | a UART: `write` + a non-blocking `readLine` |
| `Clock` | `micros`, `millis`, `delayMs` |
| `Display` | **nothing** — `NullDisplay` already exists |
| `MotionSensor` | reuse `Lsm6dsox` with the I2C calls swapped, or the vendor HAL |
| `FrameLink` | the SPI carrier for that chip |
| `SystemHooks` | heap reporting, plus suspend/resume for its scheduler |
| `main()` | construct the above, call `App::begin`, start tasks on a period |

Everything else — role logic, IMU ring, filter pumping, console, benchmark, determinism sequence,
the per-frame body of every task — is already portable and already proven portable by the host
build.

**Still explicitly out of scope:** a HUB75 DMA driver for a new MCU family is 1–2 weeks. Display
nodes stay ESP32-S3.

---

## 6. Verification record

| check | result |
|---|---|
| `ctest` | **7/7** (was 6; `app_golden` is new) |
| `partsim_golden -q` | `f0021217 8e143d3b` — unchanged |
| host `App::runGolden` | `f0021217` |
| QEMU, emulated Xtensa | `f0021217` — also covers boot, task creation, pinning, stack sufficiency |
| devkit, `cube` build, `g` | `f0021217` |
| devkit, `x` | identical to the §4 baseline, every column |
| PlatformIO environments | 8/8 build (`cube cube-fast lite beaker panel master display qemu`) |
| console commands | all of `? s` `s <n>` `c` `b <v>` `m` `t` `i` `r` `g` `p` `x` plus the unknown-command path, on hardware **and** host |

The board was reflashed from the committed tree afterwards and re-confirmed, so what is on the
desk is what is in git.

---

## 7. Process notes

**The §9 sequencing advice was not followed.** The brief advised landing the console extraction on
its own, verifying it on hardware, and only then touching the tasks — so that a later failure could
be attributed. It went in as one commit instead.

The reasoning: the bisect point that advice buys is insurance against an unattributable failure,
and the exact benchmark reproduction plus three matching determinism legs retire that risk
directly. Reconstructing an intermediate commit afterwards would mean committing a state that was
never built or tested, which is a worse kind of history than one honest commit.

It is recorded here rather than argued away, because the advice was good and the next person should
decide for themselves whether the trade was right. Re-splitting it properly — building and
hardware-testing each half — remains possible and was offered.

**A commit landed on `main` mid-session** (`720edc6`, CUBE-PCB requirements). Docs only; no
interaction with this work beyond the base it sits on.
