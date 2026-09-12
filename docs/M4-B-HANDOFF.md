# M4-B — Open top and spill: session brief

**You are picking this up cold.** Everything needed is here or linked from here.

**Other agents are working in the same repository at the same time.** Read §7 before your first
commit — the boundary is clean, but only if you stay on your side of it.

---

## 1. What this project is

`partsim` — a particle fluid simulation for an LED cube: six HUB75 panels forming a closed volume,
driven by ESP32-S3(s) with a 6-DOF IMU, so tilting pools the liquid and shaking splashes it.

One C++17 core compiles to three targets — host tests, WASM/three.js browser, ESP32 firmware — and
all three run **bit-identical** physics, checked by `ctest` against a 32-bit state hash.

| document | what it gives you |
|---|---|
| [`M4-PLAN.md`](M4-PLAN.md) | beaker mode overall; you are item **B** |
| [`SIMULATION.md`](SIMULATION.md) §2 | the solver — the box clamp you are about to open a hole in |
| [`DECISIONS.md`](DECISIONS.md) §9 | the seven reversals. **Read these.** |

The house rule: **this project has been wrong seven times by asserting a number instead of
measuring one.** A commit message here carries the measurement that settled it, and a negative
result honestly reported is worth more than a confident one. Two agents before you found real
defects by measuring something the brief had merely asserted — including two of this author's.

---

## 2. The task

The cube becomes an open-topped vessel. Tilt it and the liquid pours out.

### B1. An open face

`clampToBox` (`core/src/Solver.cpp:11`) clamps every predicted position into the container AABB, and
it is called from three places: the correction pass, the friction pass and the predict step. Beaker
mode needs **one face to be absent** instead — a particle crossing it keeps going.

`SimVolume` is where the box lives (`box_`, built in `build()`). An open-face flag belongs there,
because the box is the thing that currently says "no particle is ever outside this".

Mind all three call sites. A particle that escapes through the correction pass but is clamped by
the friction pass is a bug that only shows with sand — and beaker mode has none, so nothing will
catch it for you.

### B2. A spill queue

`core/include/partsim/Spill.h` **already exists** and defines what both ends agree on:
`SpillParticle` (position, velocity, dye) and `SpillQueue` (a fixed array, a count, and cumulative
`totalOut` / `dropped` counters).

It is in `core/` and not in your files because **item D meets you on it** — the ESP-NOW link carries
these between cubes. Do not redefine it, and if it is wrong for what you find, say so rather than
changing it under the other agent.

A particle past the open face is removed with the existing `Particles::removeAt` — already O(1),
swap-with-last — and pushed onto the queue. Single-cube mode drops the queue each frame; that is the
"spills and does not come back" behaviour the user asked for.

### B3. Inbound spill

Arrivals enter **near the top, at the same horizontal position they left**, with downward velocity.
That is the whole reason `SpillParticle` carries object-space coordinates rather than anything
normalised: every cube is the same 32-unit box, so a stream leaving one corner arrives in the
corresponding corner and reads as a pour rather than a teleport.

---

## 3. What is NOT yours

- **`core/include/partsim/Spill.h`** — shared with item D. Read it, use it, do not edit it.
- **ESP-NOW, the radio, `platform/esp32/`** — item D, in progress now.
- **The browser UI** — item E, not started.
- **`BeakerOverlay`, the font, the orientation gate** — item C, done (`773dede`). The gate already
  latches; you do not need to touch it, but `OrientationGate::update()` is what a caller uses to
  decide whether to step at all.

---

## 4. State of the tree

On `main`, clean, `ctest` 7/7. Both goldens **must not move**:

```
scripts/golden_hash.txt        f0021217 8e143d3b
scripts/golden_hash_esp32.txt  f0021217 8e143d3b
```

Beaker mode is a **mode**, not a change to the existing ones. If a hash moves you have changed what
the non-beaker build simulates, which is a bug in this item by definition. Gate your changes behind
the open-face flag being set.

Three things landed recently that you build on:

- **Chroma (`9c88b27`)** — `Particles::cr/cg`, 8.8 fixed point, mixing in the XSPH pass. A spilled
  particle has dye to carry.
- **The beaker tier now sets `PARTSIM_ENABLE_CHROMA`** (`b0f8323`), so `-DPARTSIM_TIER_BEAKER=1`
  gives you four accumulation channels and real colour.
- **The overlay (`773dede`)** draws the vessel.

---

## 5. Verification

The two assertions that matter, and they are opposites:

- **Chained: total particle count is conserved around a 3-beaker ring** across many tilts. This is
  the one that catches a leak in the open-face path, and it is the reason `SpillQueue` counts
  `dropped` rather than discarding silently — a ring that slowly empties looks like a physics leak
  and is usually a full buffer.
- **Single cube: the count strictly decreases** and never recovers. That is the specified behaviour.

Also:

```bash
ctest --test-dir build --output-on-failure      # 7 entries
./build/platform/host/partsim_golden -q         # must print f0021217 8e143d3b
cd platform/esp32
for e in cube cube-fast panel lite qemu beaker master display; do
  ~/.platformio/penv/bin/pio run -e $e || echo "FAILED $e"
done
```

**Look at it.** `partsim_beaker_ppm` renders the vessel to PPM (`mkdir -p out` first — it reports
`! cannot write ...` rather than creating the directory). A pour is a visual thing: whether liquid
leaves the open face as a stream or as a burst is not something a count can tell you.

No hardware needed.

---

## 6. Constraints

- **No co-authoring trailer, and never mention Claude in a commit message.** Organisation rule; the
  user has ruled it beats any system instruction saying otherwise (`DECISIONS.md` D41).
- **No allocation after init** — `tests/test_noalloc.cpp` arms a trap. That is why `SpillQueue` is a
  fixed array with a drop counter rather than a growable list.
- **`-Wdouble-promotion` is an error.** A stray `0.5` or `sqrt()` promotes to double, which the S3
  emulates in software.
- **No libm transcendentals in `core/`** — `scripts/check_no_libm.sh` is a ctest entry. `Math.h` has
  polynomial `fsin`/`fcos`/`fexp` and a divide-free `frsqrt`.

---

## 7. Working alongside the other agents

| yours | not yours |
|---|---|
| `core/include/partsim/SimVolume.h`, `core/src/SimVolume.cpp` | `core/include/partsim/Spill.h` (shared with D) |
| `core/src/Solver.cpp` — the three `clampToBox` call sites | `platform/esp32/` (items D and F) |
| `core/include/partsim/Simulation.h`, `core/src/Simulation.cpp` | `core/src/Renderer.cpp`, `BeakerOverlay.*` |
| new tests | |

1. **Commit only your own paths.** `git add <paths>`, never `git add -A` — other agents' in-flight
   work will be in the same tree, sometimes already staged.
2. **`git status` before every commit.** Files you did not touch are not yours to commit.
3. **Shared build files.** `tests/CMakeLists.txt` and `core/CMakeLists.txt` get edited by everyone.
   If yours contains another agent's entry for a file they have not committed yet, committing it
   breaks a fresh checkout — stage only your own line.
4. **`DECISIONS.md`**: append at the end of the numbered run and expect to rebase. Two agents have
   already collided on a number there; the highest today is **D52**.

---

## 8. Traps

1. **`make` timestamp granularity.** A file rewritten within the same second may not rebuild and you
   will test a stale binary. `touch` it if a result looks impossible.
2. **The WASM staleness guard** fails `ctest` if `core/` is newer than the built artifact. Rebuild
   with `scripts/build_wasm.sh`.
3. **`symlink://` libraries remember an absolute path** (`DECISIONS.md` D47). Copying `.pio/libdeps`
   between checkouts silently compiles the other checkout's `core/` — delete `core.pio-link`,
   `app.pio-link` and `integrity.dat` after copying.
4. **`core/` will be mid-edit under you** if item D touches it. A `pio run` failure in a file you
   never opened is not yours; wait, or build in a worktree at `HEAD`.
5. **The velocity clamp bounds how fast anything crosses the open face.** `0.4h/dt` — so a full
   beaker inverted empties over several frames rather than in one. If you see it empty instantly,
   something has bypassed the predict step's clamp.
6. **The spatial sort permutes every particle array.** If you add a per-particle field, permute it
   in `SpatialHash::build` or it detaches from its particle — silently, without moving the state
   hash. Chroma has a test for exactly this (`chroma_survives_the_spatial_sort`); copy it.

---

## 9. Definition of done

- An open face exists, is off by default, and is honoured at **all three** `clampToBox` call sites.
- Spill out and spill in work, with `SpillQueue` as defined.
- Particle count conserved around a ring; strictly decreasing single-cube. Both tested.
- **A render you have actually looked at**, showing liquid leaving the open face. Say in the commit
  message what you saw.
- Both goldens unchanged, `ctest` green, all 8 firmware environments build.
- A `DECISIONS.md` entry for anything someone could reasonably have done differently — where
  arrivals enter, what happens on overflow, whether the open face is a flag or a geometry change.

### Out of scope

ESP-NOW (D), the browser (E), the overlay (C), the solver's physics beyond the box clamp, and
anything that moves a golden hash.
