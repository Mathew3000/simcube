# M4-E — Beakers in the browser: session brief

**You are picking this up cold.** Everything needed is here or linked from here.

**Other agents may be working in the same repository at the same time.** Read §7 before your
first commit — the boundary is clean, but only if you stay on your side of it.

---

## 1. What this project is

`partsim` — a particle fluid simulation for an LED cube: six HUB75 panels forming a closed volume,
driven by ESP32-S3(s) with a 6-DOF IMU, so tilting pools the liquid and shaking splashes it.

One C++17 core compiles to three targets — host tests, WASM/three.js browser, ESP32 firmware — and
all three run **bit-identical** physics, checked by `ctest` against a 32-bit state hash.

| document | what it gives you |
|---|---|
| [`M4-PLAN.md`](M4-PLAN.md) | beaker mode overall; you are item **E**, the last one |
| [`SIMULATION.md`](SIMULATION.md) | the solver, the renderer, the multi-node seam |
| [`DECISIONS.md`](DECISIONS.md) §9 | the seven reversals. **Read these.** |
| `DECISIONS.md` D56-D65 | everything M4-B and M4-D learned, which is what you build on |

The house rule: **this project has been wrong seven times by asserting a number instead of
measuring one.** A commit message here carries the measurement that settled it, and a negative
result honestly reported is worth more than a confident one. Three agents before you found real
defects by measuring something a brief had merely asserted — including several of this author's.

---

## 2. The task

**N beakers on one page, chained into a ring, debugged where it is cheap to look.**

The cube is an open-topped vessel: tilt it and the liquid pours out of the top face. What spills
from beaker 1 lands in beaker 2, and so on, wrapping back to 1. Each starts with its own dye, which
**mixes** — red pouring into blue converges to pink. A single beaker loses what it spills.

This is where chaining gets debugged: on hardware a chain is two boards and a serial console, and
you cannot see a pour at all.

### E1. Bindings for N beakers

`platform/wasm/bindings.cpp` today holds **one** `Simulation` (`g_sim`) plus the multi-node preview
(`ps_node_*`, one master and three display nodes). You need a third set alongside them — do not
repurpose either, the same way `ps_node_*` was added alongside `ps_*` rather than replacing it.

Sketch, adjust as the work tells you:

```
ps_beaker_init(count, panelRes)        N sims, top face open, each with its own dye
ps_beaker_set_dye(i, r, g)             0-255 each; blue is what is left over
ps_beaker_reset(i)                     refill and restore the configured dye
ps_beaker_orient(i, qx, qy, qz, qw)    tilt one beaker
ps_beaker_step(i, dt)                  advance + pump; returns bytes now in the outbox
ps_beaker_out_ptr(i) / ps_beaker_out_len(i)     this frame's packets, for JS to mangle
ps_beaker_deliver(i, ptr, len)         hand a (possibly dropped, delayed, corrupted) packet over
ps_beaker_fill(i)                      particle count, for the readout
ps_beaker_panel_ptr(i, face)           that beaker's pixels
```

### E2. **Use the pump. Do not write a second one in JavaScript.**

`core/include/partsim/SpillChain.h` is the pump: it splits a frame's spill into packets, carries a
cumulative count so a receiver can tell a lost packet from a quiet link, injects arrivals, and makes
up what the air lost. It runs on the device today and it is the same code the browser must run —
that is the entire argument that put `SimFrame` in core for the multi-node preview.

So implement a `SpillTransport` in `bindings.cpp` whose `send()` appends to a byte buffer JS can
read and whose `poll()` drains a queue JS pushes into. **JS physically holds the bytes**, which is
what makes fault injection honest: dropping a packet in JS is a real dropped packet, not a
simulation of one.

`SpillChain::pump(sim, transport)` is then called once per step, and the cadence matters — see
trap 5.

### E3. The page

`platform/wasm/web/nodes.html` is the closest existing model: several module-side instances, JS
carrying the bytes between them, per-instance state on screen. Build `beakers.html` beside it.

- per-beaker **reset** (refill, restore the original dye) — also exists as a console command on
  hardware, so this is not the only way to reset a cube
- per-beaker **colour picker**, **fill readout**, and the beaker's current *mixed* colour
- fault injection: drop a packet, delay it, corrupt a byte, and watch the shortfall get made up
  rather than the ring silently draining
- the chain order visible, since it is the thing a user configures on real cubes

---

## 3. What is NOT yours

- **`core/`** — `SpillChain`, `Simulation`, the solver, the renderer. If the pump is wrong for what
  you find, **say so rather than changing it**: it runs on hardware and its behaviour was measured
  across two boards. A finding is worth more here than a patch.
- **`platform/esp32/`, `platform/app/`** — the firmware and the application layer.
- **The existing `ps_*` and `ps_node_*` entry points**, and `index.html` / `nodes.html` /
  `panels.html` / `orient.html`. Add; do not repurpose.

---

## 4. The trap that will cost you a day if nobody tells you

**The shipped WASM artifact has chroma compiled out, and beaker mode is entirely about colour.**

`PARTSIM_ENABLE_CHROMA` is set by the **beaker tier**, not by the default build. With it off there
is exactly one accumulation channel, `Particles` has no `cr`/`cg` arrays, dye does not exist, and
every pixel resolves through one ramp. With it on, `kChannelCount` is 4 and `resolve()` computes a
colour ratio — so **the pixel golden moves**, and `scripts/check_determinism.mjs` compares the
artifact against the host's.

So: build a **second artifact** for the beaker page —
`scripts/build_wasm.sh -DPARTSIM_ENABLE_CHROMA=1` into `public/partsim_beaker.wasm` — and leave the
default one exactly as it is. `beakers.html` loads the beaker module; every other page keeps the
one the determinism check guards.

Two consequences to handle rather than discover:

1. The staleness guard in `scripts/check_wasm.sh` watches **one** artifact. A second one that goes
   stale is invisible. Extend the guard, or say in the commit message why you did not.
2. `build_wasm.sh` writes into a single `build-wasm` directory. Two configurations in one directory
   is a stale-object bug waiting to happen; give the second its own.

---

## 5. State of the tree

On `main`, clean, `ctest` 7/7, 191 test cases. Both goldens **must not move**:

```
scripts/golden_hash.txt        f0021217 8e143d3b
scripts/golden_hash_esp32.txt  f0021217 8e143d3b
```

What landed, and what each gives you:

| | |
|---|---|
| **A** `9c88b27` | chroma: `cr`/`cg` in 8.8 fixed point, mixing in the XSPH pass, split-kernel splat |
| **C** `773dede` | `BeakerOverlay`, a 3x5 font, an orientation latch that never re-arms |
| **B** `623c71a`, `d848873` | the open face, `SimVolume::clampInto`, `SpillQueue`, `injectSpill` |
| **D** `4a3babc` | `SpillChain`, the wire format, ESP-NOW, and two defects worth reading about |

Measured on two devkits, 20 s of pouring: 179 spilled, 179 sent in 140 packets, 179 received;
shortfall, made-up, refused and bad all zero. That is the behaviour the browser must reproduce with
a perfect link, and depart from in exactly the ways you inject.

---

## 6. Verification

- **Colour**: pouring red into blue converges toward magenta and **conserves total chroma weight**
  — mixing must not create or destroy dye. Dye is conserved to ~5% and that drift *plateaus*
  (measured in A); do not assert exactness.
- **Ring**: total particle count conserved around a 3-beaker ring across many tilts, with a perfect
  link. **Single beaker: strictly decreasing, never recovers.**
- **Loss**: drop packets and watch `shortfall` rise and `madeUp` follow it — the ring must not
  drain. This is the mechanism the whole cumulative count exists for.
- **`unsent` must stay zero.** If it is not, your JS is stepping a beaker more often than it pumps
  it, and the far end will look fine anyway. See trap 5.
- **Fill**: a beaker fills to 60% of its pool, not 100% — a full vessel cannot receive, and on the
  first hardware run all 229 arrivals were rejected for want of room (`DECISIONS.md` D63).

```bash
ctest --test-dir build --output-on-failure      # 7 entries, 191 cases
./build/platform/host/partsim_golden -q         # must print f0021217 8e143d3b
scripts/build_wasm.sh                           # the DEFAULT artifact, unchanged
```

**Look at it.** A pour is a visual thing and this item is the only place in the project where it can
be watched. Whether liquid leaves as a stream or as a burst, whether an arrival reads as pouring in
or as materialising, and whether the mixed colour reads as mixing are all judgements a count cannot
make.

No hardware needed.

---

## 7. Constraints

- **No co-authoring trailer, and never mention Claude in a commit message.** Organisation rule; the
  user has ruled it beats any system instruction saying otherwise (`DECISIONS.md` D41).
- **No allocation after init** in `core/` — `tests/test_noalloc.cpp` arms a trap. The bindings are
  not core, but the pools they hold are.
- **`-Wdouble-promotion` is an error** in core. A stray `0.5` or `sqrt()` promotes to double.
- **`ALLOW_MEMORY_GROWTH` is off** and `INITIAL_MEMORY` is 32 MB, because growth detaches every
  `Uint8Array` view JS is holding. **One `Simulation` is 3.19 MB at host capacity** (measured), and
  the module already holds `g_sim` plus three display nodes. Four beakers fits; six does not.
  Either cap the count or raise `INITIAL_MEMORY` deliberately and say what it cost.
- **Commit only your own paths.** `git add <paths>`, never `git add -A`.

---

## 8. Traps

1. **The chroma artifact.** §4. It is the big one.
2. **The WASM staleness guard** fails `ctest` if `core/` is newer than the built artifact — rebuild
   with `scripts/build_wasm.sh`. It is there because an artifact 18 days behind `core` once sailed
   through the determinism comparison, which cannot see a stale binary.
3. **Browser cache.** A rebuilt `.wasm` served from cache looks exactly like a change that did
   nothing. Hard-reload before believing a negative result.
4. **`three.js` is vendored**, not from a CDN, and the pages are ES modules — they need a real HTTP
   server, not `file://`.
5. **Pump cadence.** `Simulation::advance()` clears the spill queue once per **frame**;
   `stepFixed()` clears it once per **step**. Pump at the wrong one and you send a fraction of the
   pour — and *the far end still looks right*, because the cumulative count makes the rest up out of
   clones. This exact bug shipped half a pour as copies and every counter on both boards agreed it
   was fine (`DECISIONS.md` D62). `SpillChain::Stats::unsent` is what names it; surface it in the UI.
6. **An arrival keeps its own dye; a refill gets the beaker's.** `Simulation::setDye` is the cube's
   identity, not the average of its contents — a beaker that has turned pink must refill red
   (D64).
7. **Quantisation is real.** Positions cross the wire as uint16 over the container box and
   velocities as int8. A round-trip is not the identity, and the tests bound the error rather than
   asserting equality (`tests/test_spill.cpp`).

---

## 9. Definition of done

- N beakers on one page, chained into a ring through `SpillChain`, pouring visibly between each
  other with dye that mixes.
- Per-beaker reset, colour picker, fill readout, mixed-colour readout.
- Fault injection: drop, delay and corrupt, with the shortfall visibly made up rather than the ring
  draining.
- Ring conserves; single beaker strictly decreases; `unsent` zero. All three visible on the page,
  not just true in the code.
- Both goldens unchanged, `ctest` green, the default WASM artifact unchanged and its determinism
  check still passing.
- A `DECISIONS.md` entry for anything someone could reasonably have done differently — where the
  second artifact came from, how the ring is ordered, what the UI does when a beaker is full.
- **Say what you saw.** Two open questions in `M4-PLAN.md` §6 are yours to settle on a render:
  whether the split kernel reads as colour *in* the fluid or floating *on* it, and whether ~10
  distinguishable colour regions at `cube`'s spacing read as mixing — the latter decides whether
  beaker mode runs at 8.5 fps or at 26.

### Out of scope

The firmware, the solver, the ESP-NOW link, NVS persistence of chain order and colour (it belongs
with the configuration surface you are building, but only after the browser proves what the
configuration *is*), and anything that moves a golden hash.
