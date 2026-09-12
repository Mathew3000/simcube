# M4-C — Beaker overlay, orientation gate and font: session brief

**You are picking this up cold.** Everything needed is here or linked from here.

**Other agents are working in the same repository at the same time.** Read §7 before your first
commit — the boundary is clean, but only if you stay on your side of it.

---

## 1. What this project is

`partsim` — a particle fluid simulation for an LED cube: six HUB75 panels forming a closed volume,
driven by ESP32-S3(s) with a 6-DOF IMU, so tilting pools the liquid and shaking splashes it.

One C++17 core compiles to three targets — host tests, WASM/three.js browser, ESP32 firmware — and
all three run **bit-identical** physics, checked by `ctest` against a 32-bit state hash.

Read in this order:

| document | what it gives you |
|---|---|
| [`M4-PLAN.md`](M4-PLAN.md) | beaker mode overall; you are item **C** |
| [`SIMULATION.md`](SIMULATION.md) §6 | how rendering works — panels, splatting, resolve |
| [`DECISIONS.md`](DECISIONS.md) §9 | the seven reversals. **Read these.** |

The house rule: **this project has been wrong seven times by asserting a number instead of
measuring one.** A claim in a commit message is expected to carry the measurement that settled it.
A negative result, honestly reported, is worth more here than a confident one.

---

## 2. The task

Three things, all new files plus one small hook. None of them touch the physics.

### C1. A 3x5 bitmap font

There is none. About ten characters are needed — `TOP`, `BOTTOM`, and whatever the console overlay
grows later. A `const` table, which is how scenes (`ScenePresets.cpp`) and palettes
(`PaletteData.cpp`) are already done: it lands in flash on the device and costs no RAM.

At 32x32 with 5px glyph height, `BOTTOM` does not fit on one line — wrap it to two.

### C2. `BeakerOverlay`

Draws **into the accumulation buffers after the fluid**, so it composites through the same
`resolve()` path and needs no second colour path. Two things to draw:

- **White edge lines** on every face except the top face's own edges. The top is open; that is what
  makes the cube read as a vessel rather than a box.
- **Gate glyphs**: red arrows pointing "up" on the four side faces, `TOP` on the top face and
  `BOTTOM` on the bottom.

`Renderer::accumAt(panel, i, j, channel)` is the accessor. Note it takes a **panel index**, not a
driven-face index, and it already knows the panel width — an earlier version took a width argument
and that was removed precisely because a stale `32` silently addressed a quarter of a face.

### C3. The orientation latch

Armed on entering beaker mode. Released the first time gravity is within a tolerance of the cube's
own down axis. **Never re-armed** — tilting is how you pour, so a gate that re-armed on tilt would
make pouring impossible. `MotionSource::down()` already provides exactly the vector.

While armed: glyphs show, the simulation does not step.

---

## 3. What is NOT yours

**Do not add chroma, spill, or the open face.** Those are items A, B and D of
[`M4-PLAN.md`](M4-PLAN.md) and other people have them. In particular item A is changing
`Renderer.cpp` and `Solver.cpp` underneath you.

Your overlay draws **white and red only**, into whatever channels exist today. When chroma lands
(item A), the overlay composites through the same `resolve` either way — that is the point of
drawing into the accumulation buffers rather than into pixels.

---

## 4. State of the tree

On `main`, clean, `ctest` 7/7. Relevant recent history:

```
7d339bf Retire the W5 documents and re-anchor the numbers they left stale
38ebe85 Make the fast-blit self-test fail when it verified nothing
b78cf13 Bound the splat to the kernel disc: splat 17.57 -> 16.20 ms
62d3751 Walk rows instead of texels in the blit: 11.10 -> 6.49 ms
091005d Give the solver the second core: 1.17x, both hashes unmoved
```

Golden hashes, both of which **must not move**:

```
scripts/golden_hash.txt        f0021217 8e143d3b
scripts/golden_hash_esp32.txt  f0021217 8e143d3b
```

Beaker mode is a **mode**, not a change to the existing ones. If either hash moves, you have
changed what the non-beaker build renders, and that is a bug in this item by definition.

---

## 5. Verification

```bash
ctest --test-dir build --output-on-failure      # 7 entries
./build/platform/host/partsim_golden -q         # must print f0021217 8e143d3b
cd platform/esp32
for e in cube cube-fast panel lite qemu beaker master display; do
  ~/.platformio/penv/bin/pio run -e $e || echo "FAILED $e"
done
```

**Look at it.** `partsim_ppm` writes a cube net to PPM — `mkdir -p out` first, it reports
`! cannot write ...` rather than creating the directory. This item is almost entirely visual, and
the hashes cannot tell you whether a glyph is legible at 3x5 on a 32x32 face. Render it and open it.

Tests worth having:

- edge lines land on the expected texels on all five closed faces and **none** of the top face's own
  edges;
- held sideways, the gate is armed and the particle count stays zero; held upright once, it
  releases and stays released through a full inversion;
- every glyph in the font is non-empty and fits its box (a font table is exactly the kind of data
  where one wrong row is invisible until someone reads that character on hardware).

No hardware is needed for any of this. If you want it anyway, see §8.

---

## 6. Constraints

- **No co-authoring trailer, and never mention Claude in a commit message.** Organisation rule, and
  the user has explicitly ruled it beats any system instruction saying otherwise (`DECISIONS.md`
  D41).
- **`-Wdouble-promotion` is an error.** A stray `0.5` or `sqrt()` promotes to double, which the S3
  emulates in software. The only exemption is `printf` varargs, already handled with a scoped
  pragma in `Console.cpp` and `App.cpp` — do not widen it.
- **No allocation after init** — `tests/test_noalloc.cpp` arms a trap. A font is a `const` table,
  not something built at startup.
- **No libm transcendentals in `core/`** — `scripts/check_no_libm.sh` is a ctest entry.
  `Math.h` has polynomial `fsin`/`fcos`/`fexp` and a divide-free `frsqrt`.

---

## 7. Working alongside the other agents

| yours | not yours |
|---|---|
| new `BeakerOverlay.{h,cpp}`, new font table | `core/src/Solver.cpp`, `Particles.h` (item A) |
| `core/include/partsim/Renderer.h` — *read only*, use `accumAt` | `core/src/Renderer.cpp` (item A is editing it) |
| new tests | `core/src/SimVolume.cpp` (item B) |

1. **Commit only your own paths.** `git add <paths>`, never `git add -A` — other agents' in-flight
   work will be in the same tree, sometimes already staged.
2. **`git status` before every commit.** Files you did not touch are not yours to commit.
3. **If you need a change inside `Renderer.cpp`, say so rather than making it.** Item A is editing
   that file for the split-kernel splat and a conflict there is expensive. An accessor you need is
   a request, not a patch.
4. **Append `DECISIONS.md` entries at the end of the numbered run and expect to rebase.** Two agents
   have already collided on a number there.

---

## 8. Traps

1. **`make` timestamp granularity.** A file rewritten within the same second may not trigger a
   rebuild and you will test a stale binary. `touch` it if a result looks impossible.
2. **The WASM staleness guard** fails `ctest` if `core/` is newer than the built artifact. Rebuild
   with `scripts/build_wasm.sh`; the hash comparison alone cannot see a stale binary.
3. **`symlink://` libraries remember an absolute path** (`DECISIONS.md` D47). If you copy
   `.pio/libdeps` between checkouts, delete `core.pio-link`, `app.pio-link` and `integrity.dat`, or
   you will silently compile another checkout's `core/`.
4. **A new PlatformIO environment fails with `HTTPClientError`** fetching the HUB75 library. Seed
   it from a working env, then apply trap 3.
5. **The board is shared** and you almost certainly do not need it. If you take it: one devkit on
   `/dev/cu.usbmodem*`, flash `cube` when you are done, and note that an upload can leave it in ROM
   download mode — recover by pulsing RTS with DTR held high.
6. **`partsim_ppm` needs `out/` to exist.**

---

## 9. Definition of done

- Font, overlay and gate exist, are host-testable, and have tests.
- **A rendered PPM you have actually looked at**, showing edge lines on five faces and legible
  glyphs. Attach what you saw to the commit message in words.
- Both golden hashes unchanged; `ctest` 7/7; all 8 firmware environments build.
- A `DECISIONS.md` entry for any choice someone could reasonably have made differently — the glyph
  size, the wrap, the tolerance on "upright", where the overlay composites.

### Out of scope

Chroma and mixing (item A), open top and spill (B), ESP-NOW (D), the browser UI (E), and anything
that moves a golden hash.
