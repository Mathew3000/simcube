# W5 — The render floor: what was measured

Three commits. Two were the brief (`W5-HANDOFF.md`); the third was found while writing the first
and is a display defect that no test could see.

```
a622df7 Give the HUB75 library the brightness table for the depth it runs at
b78cf13 Bound the splat to the kernel disc: splat 17.57 -> 16.20 ms
62d3751 Walk rows instead of texels in the blit: 11.10 -> 6.49 ms
```

Both golden hashes are unchanged, all 7 `ctest` entries pass, all 8 firmware environments build,
and the device state hash is still `f0021217`.

---

## 1. The numbers

Devkit, `cube` build, six 32×32 faces, `x` on the console.

```
  count    sim/step   splat   resolve    blit    frame    fps
    128       8.06    5.59      2.60    5.96    27.67   36.1
    256      26.14   11.50      2.89    6.25    70.02   14.3
    384      49.05   16.20      3.13    6.49   120.78    8.3
    512      74.65   19.59      3.37    6.73   175.63    5.7
```

against the brief's baseline:

```
  count    sim/step   splat   resolve    blit    frame    fps
    128       9.22    6.07      2.60   10.57    35.09   28.5
    256      30.75   12.38      2.89   10.85    84.72   11.8
    384      57.46   17.57      3.13   11.10   143.58    7.0
    512      87.54   21.27      3.37   11.33   207.68    4.8
```

`sim/step` moved because of someone else's commit, not this work — the other agent landed the
second-core change during the session. Every measurement below was taken in a separate worktree
with a before and an after at the same `main`, so no row here mixes the two.

Kettle: `sim 24.96 ms, splat 78.11 -> 77.35 ms`.

Firmware RAM 171 740 → 172 976 bytes, +1 236. That is the blit's 1 KB compensation table, its
128-byte row scratch and a few members.

### The floor

**28.67 → 22.69 ms at 384 particles, −20.9%.** A free solver would give 44.1 fps rather than 34.9.

That "34.9" needs saying out loud, because the brief and `SIMULATION.md` both quoted the floor as
**30.7 ms and 32.6 fps** and those figures are wrong — by about 3.1 ms, in the safe direction.
`App::runBench` times `resolve` on its own and then times `present()`, which *also* resolves. The
floor is `splat + blit`, and adding `resolve` to it counts resolve twice. The `frame` column has
always been right; only the prose derived from it was not.

| | old | new |
|---|---|---|
| floor as published (double-counted) | 30.7 ms → 32.6 fps | 25.8 ms → 38.7 fps |
| floor as measured (`splat + blit`) | 28.67 ms → 34.9 fps | 22.69 ms → 44.1 fps |

---

## 2. W5a — the row-walking blit: 11.10 → 6.49 ms

### The premise was half right

The brief attributed the cost to "6 144 pixels × 6 planes of **scattered** read-modify-write". Two
throwaway builds settled which half of that sentence was doing the work:

| inner loop | blit at 384 |
|---|---|
| baseline | 11.10 ms |
| every texel written to (0, 0) — same call, no scatter | 10.90 ms |
| no driver call at all — loop and `ChainRun` arithmetic only | 3.60 ms |

So `drawPixelRGB888` costs **7.50 ms** and the scattered addressing accounts for **0.20 ms** of it.
Internal SRAM on the S3 has no data cache to miss, which is why the scatter is nearly free; the
cost is per-call work. The largest single item is that the library recomputes

```c
&fb->rowBits[y]->data[depth * fb->rowBits[y]->width]
```

for every bitplane of every texel — a pointer, a vector, a `shared_ptr`, two more loads and a
multiply, six times per pixel.

### What shipped

The row pointer is fetched once per bitplane per **row**, and brightness compensation and bitplane
interleave are folded into one 1 KB table so a texel costs three loads and two shifts instead of
six mask-and-test rounds. Per-texel remains as the fallback: a quarter-turn mount maps a renderer
row onto a chain *column*, where there is no span to walk.

Pixel pushing, with the 0.47 ms of loop overhead taken off both ends: **7.50 → 2.89 ms, 2.6×.**

### Two variants measured and rejected

| variant | blit at 384 |
|---|---|
| row walk, straightforward | 6.75 ms |
| + two chain pixels per 32-bit read-modify-write | 6.59 ms |
| + half-of-panel shift folded into the row pass instead | **6.49 ms** |

The 32-bit pairing buys 0.16 ms and costs an alignment guard and a little-endian assumption. It
was dropped, and the reason is the same finding as above: **the read-modify-write is not the
bottleneck**, the per-texel arithmetic is. Folding the half-of-panel shift into the pass that
already touches every texel removes six shifts per texel for nothing, and is less code than what
it replaced.

### A number this corrected downstream

`RESOURCES.md` §2 costed every candidate PCB topology from an assumed **~130 cycles** per
`drawPixelRGB888`. Measured, it is **293** — 7.50 ms for 6 144 texels at 240 MHz. The row-walking
path is **113**.

Scaled by texel count to the 64×64 faces that table is about:

| option | was published | per-texel, measured | row-walking, measured |
|---|---|---|---|
| 3 × 2 faces | 4.4 ms | 10.0 ms | **3.9 ms** |
| 2 × 3 faces | 6.7 ms | 15.0 ms | **5.8 ms** |
| 1 × 6 faces | 13.3 ms | 30.0 ms | **11.6 ms** |

On a 33 ms frame, one board driving six 64×64 faces was never viable on the per-texel path — 90% of
the frame, before the solver runs at all. The table's *ordering* never changed, which is why nobody
caught it, but the margins it implied were 2.3× too generous. Corrected in place.

### The uncomfortable part

`MatrixPanel_I2S_DMA::fb` is private, and the library has no bulk-pixel entry point — `hlineDMA`
and friends take one colour for a whole span, which a fluid render never has. The options were to
vendor a 40-file third-party library to add one accessor, or to reach the member.

`platform/esp32/src/PanelFramebuffer.h` reaches it, using the explicit-instantiation idiom that
[temp.spec]/6 exists for. It is legal C++, not a layout assumption and not undefined behaviour —
but it *is* a dependency on a private member's name and type, and that is a real cost.

Both ways it can break are loud:

- a renamed or retyped member is a **compile error**, not a wrong picture;
- a changed buffer **layout** is caught at boot by `PanelDriver::verifyFastBlit()`, which blits a
  real row through the real shipping code both ways and compares the raw DMA words. It covers the
  brightness table, the bitplane packing, the two-halves bit offset, the chain addressing and the
  run direction at once, on the buffer the library actually allocated. On any mismatch the driver
  keeps the per-texel path for good.

The boot line says which path is live:

```
panels: chain 192x32, rows all horizontal, blit row-walking
```

The library is pinned at `^3.0.11` and resolves to 3.0.15.

---

## 3. W5b — the circular splat bound: 17.57 → 16.20 ms

### 7.8%, where the brief estimated 2–3 ms

The brief is right that ~21% of the scanned texels can never contribute. It does not follow that
21% of the time is recoverable, and it is not — **the wasted texels are the cheapest ones.** A
rejected texel computes `dx`, a squared distance and a compare. An accepted one also does a LUT
load, a multiply, a shift and a saturating read-modify-write. Removing a fifth of the iterations
removes under a tenth of the work.

That is the finding: counting texels was never the same as counting time. The measured saving is
1.37 ms at 384 particles and 1.68 ms at 512.

### Why an outward walk rather than a computed bound

The chord half-width wants a square root, and the S3's FPU has none (`DECISIONS.md` F1) — a
per-row `sqrtf` would cost more than the texels it saves, and `core/` links no libm anyway.

So the loop walks outward from the texel nearest the particle and stops at the first miss. That is
**exact, not approximate**, which is why the pixel hash does not move: `|dx|` grows monotonically
away from the centre texel, so does `dx*dx + dy2`, so does the product with `kernelScale_`, and so
does its truncation to `int`. A texel past the first failure cannot pass the test the old loop
applied to it. The same argument in `dy` rejects whole rows. Sweeping right-then-left visits a row
in a different order, and every texel in a row is a different cell, so the saturating add sees the
same operands either way.

A second implementation was written and rejected on measurement: a 65-byte table of the disc's
half-width per quantised `dy²`, giving one tight loop instead of two.

| bound | splat at 384 |
|---|---|
| none (square box) | 17.57 ms |
| half-width table, one tight loop | 17.26 ms |
| outward walk, break on first miss | **16.20 ms** |

The table bound has to be conservative across a whole quantisation bucket, so it scans wider than
the real chord and gives back most of what the single loop wins.

---

## 4. The display defect this found

**Every pixel above input 144 was being sent to the panels wrong, and nothing could see it.**

The row-walking blit has to reproduce the library's brightness compensation exactly, which meant
reading what that compensation actually does. `cie_luts.h` picks its CIE table from
`PIXEL_COLOR_DEPTH_BITS` **at compile time**. Nothing defined it, so it defaulted to the 8-bit
table — while `setPixelColorDepthBits()` was being handed 6 at runtime from `kColourBits`. The
library then takes the low 6 bits of an 8-bit value, and the top two bits are simply dropped.

Read straight out of the DMA buffer on the devkit, red channel, one input every 8:

```
before   ... 136 -> 54   144 -> 62   152 ->  7   160 -> 16   168 -> 26 ...
              192 -> 60   200 -> 10   208 -> 24   216 -> 39   224 -> 55   232 ->  8 ...
after    ... 136 -> 13   144 -> 15   152 -> 17   160 -> 20   168 -> 22 ...
              192 -> 31   200 -> 34   208 -> 37   216 -> 41   224 -> 45   232 -> 49 ...
```

The top 44% of the input range was three sawteeth. Above 144, a brighter pixel was usually a
*dimmer* one — the core of every splat would have read as dark banding and the fluid's whole upper
range as noise.

It has never been seen because no panels are attached to the board (brief, trap 5), and it is
invisible to every hash in the project: both goldens come from `Renderer::resolve`, which is
correct and unchanged. The defect lives entirely between the driver and the panel, which is the
one stretch nothing tests.

### The fix, and the rule it bends

One flag, `-DPIXEL_COLOR_DEPTH_BITS=6`, which duplicates a number `Config.h` owns — exactly what
D36 says not to do. A third-party preprocessor cannot read a `constexpr`, so the choice is between
duplicating the number and patching the library.

It is duplicated and **checked**: `PanelDriver.cpp` static_asserts `kColourBits` against
`PIXEL_COLOR_DEPTH_BITS`. Setting the flag to 7 was confirmed to fail the build rather than ship a
wrapped ramp. The `lite` tier is 4-bit, so it unflags and re-sets the macro.

The blit's self-test needed no change and still passes — it builds its table from `lumConvTab` and
compares against the library's own `drawPixelRGB888`, so it follows the corrected table by
construction.

---

## 5. Traps met that the brief did not list

**`symlink://` libraries remember an absolute path.** `.pio/libdeps/<env>/core.pio-link` records
the `cwd` it was resolved from. Copying `libdeps` between checkouts to dodge trap 2 therefore makes
the new checkout compile the **old** checkout's `core/` and `platform/app/`, silently. Two W5b
measurements were void before this surfaced — the giveaway was a probe that deleted the entire
splat inner loop and changed `splat` by 0.00 ms. Delete `core.pio-link`, `app.pio-link` and
`integrity.dat` after copying; keep only the HUB75 directory, which is the one worth not
re-fetching.

**`PLATFORMIO_BUILD_FLAGS` does not reach `symlink://` libraries.** It applies to `src/`. A probe
`#if` in `core/` compiles to nothing and the build succeeds, so the stale-looking result is not
obviously stale. Edit the source for a core probe.

**`partsim_ppm` needs `out/` to exist** and reports `! cannot write ...` per file rather than
creating it. `mkdir -p out` first.

---

## 6. What is still on the table

The floor is now **22.69 ms at 384 particles**, of which splat is 16.20 and blit is 6.49.

- **Splat is now three quarters of the floor** and is the only remaining lever of any size. Its
  cost is the accepted texels, not the rejected ones, so the levers are the blob radius (which is
  a look decision — D21, F3) or splitting brightness from chroma (P1), not tighter loop bounds.
- **Blit: ~2.9 ms of pixel pushing over ~3.6 ms of loop and resolve.** The remaining pushing is
  about 20 cycles per plane-texel, and the 32-bit pairing measurement says that is arithmetic and
  not memory. Another 2× would need the per-texel work itself to go, and is worth perhaps 1.5 ms.
- **`SPIRAM_DMA_BUFFER` at 64×64** now has its hard prerequisite (`RESOURCES.md` §2). The
  per-texel path was known unviable there; the row writer exists and is sequential along each
  plane row, which is the pattern PSRAM can absorb. The measurement is still owed, and it is the
  one that decides whether a single-board six-face cube is real.
- **The 64×64 blit scaling is still assumed.** Everything above was measured on a 192×32 chain.
  Four times the pixels on a wider DMA row need not scale linearly, and that is now the largest
  remaining [A] in the PCB decision.
