# M4-C — beaker overlay, orientation gate and font: what was built and what was measured

Item **C** of [`M4-PLAN.md`](M4-PLAN.md), against [`M4-C-HANDOFF.md`](M4-C-HANDOFF.md).

Everything asked for exists, is host-testable and has tests. Three of the handoff's own numbers
turned out to be assertions rather than measurements, and one of them is a **blocker on red** that
belongs to someone else. Those are §2, §3 and §4 below.

---

## 1. What landed

| file | what |
|---|---|
| `core/include/partsim/Font3x5.h`, `core/src/Font3x5.cpp` | 3x5 bitmap font, `const` table: A-Z, 0-9, space and nine symbols (45 glyphs, 90 B of flash) |
| `core/include/partsim/BeakerOverlay.h`, `core/src/BeakerOverlay.cpp` | `BeakerOverlay` (edge lines, arrows, text) and `OrientationGate` (the latch) |
| `core/include/partsim/Renderer.h` | **the hook**: two inline accessors, `addAccum` and `setAccum`. `Renderer.cpp` untouched |
| `platform/host/beaker_ppm.cpp` | renders the overlay to PPM, because this item is judged by eye |
| `tests/test_font.cpp`, `tests/test_beaker.cpp` | 16 cases |

The overlay is drawn by an explicit call between `accumulate()` and `resolve()`. It is **not**
wired into `Simulation::render()`, which is why the rendered-pixel golden hash is unmoved: a
caller that does not ask for the overlay does not get it.

---

## 2. Red is not representable in the `beaker` tier as it stands

**This is the one finding that needs someone else to act.**

The handoff says to draw "white and red only, into whatever channels exist today". Measured, not
assumed — `kChannelCount` by configuration:

| build | channels | red? |
|---|---|---|
| host default (`cube`-equivalent: sand + heat) | 3 | no — the overlay only ever writes `kChWater` |
| `-DPARTSIM_TIER_BEAKER=1` (what `env:beaker` builds today) | **1** | **no** |
| `-DPARTSIM_TIER_BEAKER=1 -DPARTSIM_ENABLE_CHROMA=1` | 4 | **yes, exactly** |

The beaker tier sets `PARTSIM_ENABLE_SAND 0` and `PARTSIM_ENABLE_HEAT 0`, so it has exactly one
accumulation channel, and `resolve()` maps that channel through the water ramp. **Every overlay
texel in that build resolves to the same colour** — the top of the water ramp, which is
(235,250,255) naturalistic and (230,255,255) neon. White lines are right. Red arrows come out
white. This is a missing channel, not a missing branch: no amount of code in `BeakerOverlay` can
produce a second colour out of one scalar and one ramp.

With item A's `PARTSIM_ENABLE_CHROMA` the overlay's three dye writes land in `kChCR/kChCG/kChCB`,
`resolve()` recovers the ratio, and the arrows are red — **verified on a render**, not inferred.
Both cases are asserted in `tests/test_beaker.cpp`, in a `#if` pair, so the limitation is tested
rather than merely noted, and the day chroma lands the honest-limitation case compiles out and the
red-arrow case takes over.

**The request** (`platform/esp32/platformio.ini` is not mine, and neither is the flag): `env:beaker`
needs `-DPARTSIM_ENABLE_CHROMA=1`. Beaker mode wants chroma anyway — it is what item A exists for —
so this is a line in the environment, not a design question. Until it is there, beaker mode's gate
glyphs are monochrome.

---

## 3. `BOTTOM` fits on one line at 32x32

The handoff states "At 32x32 with 5px glyph height, `BOTTOM` does not fit on one line — wrap it to
two." Measured:

```
6 glyphs x 3 texels + 5 gaps x 1 texel      = 23 texels
32 - 2 x (2-texel margin, clear of the edge lines) = 28 texels available
```

23 of 28. It fits, with 5 texels to spare, and the render confirms it. The wrap machinery was
built anyway and is tested (`beaker_text_wraps_when_it_does_not_fit`) — it simply does not trigger
for this word at this size, and `beaker_bottom_reads_BOTTOM_on_one_line_at_32` pins that.

Where the prediction **does** come true is at a 4-wide font: `6 x 4 + 5 = 29 > 28`. If a later
legibility pass widens the glyph box (see §4 for why it might), `BOTTOM` wraps, and the code
already handles it.

---

## 4. M and N are not cleanly separable at 3x5 — a negative result

The first render read `BOTTON`. Three columns cannot hold the middle vertex that distinguishes an
M, so the glyph has to be chosen for "distinct from N and H" rather than for "looks like an M".
Four candidates were rendered at panel scale and compared:

| candidate | reads as |
|---|---|
| `101/111/111/101/101` (the conventional one) | N — two full middle rows are a solid block, not a vertex |
| `101/111/101/101/101` | H — a bar one row too high |
| `111/111/101/101/101` | **chosen**: a solid cap over two legs; not N (one leg, one shoulder), not H |
| `101/111/111/111/101` | a filled rectangle |

In isolation none of them is an unambiguous M. In the word `BOTTOM` the chosen one reads, which is
the only place the font is used today. **If a glyph ever has to be read out of context — a status
code, a node id — 3x5 is the wrong box and the finding above about wrapping applies.**

The font tests are built around this class of error rather than around spot checks: every glyph is
asserted non-empty, more than one row deep, and **distinct from every other glyph in the table**. A
duplicated bitmap is how a copy-paste error in a table like this survives review — green build,
green tests, one letter rendering as another on hardware.

---

## 5. Chroma is assigned, not added — measured on a render

The first chroma-enabled render drew a **magenta** `BOTTOM` over a blue beaker, and a red one over
an empty face. Cause: `resolve()` takes the ratio of the dye channels, so adding the glyph's red to
the fluid's blue mixes them exactly as intended for two fluids and exactly wrong for an overlay.

So the hook is two methods, and the split is the point:

- **`addAccum`** — weight. Additive and saturating, because the overlay must never make a texel
  *darker* than the fluid already made it; an assigning overlay would punch holes in bright liquid
  wherever it wrote a dimmer value, and an edge line would flicker dark exactly where the water
  touches it.
- **`setAccum`** — dye. Overwriting, because a glyph's colour is its own statement. A "hold me this
  way up" instruction whose colour depends on the fill level is the one thing it must not be.

Both are bounds-checked on `i`, `j` and `channel`, unlike the splat loop: the splat's footprint is
clamped by construction, whereas overlay coordinates are computed from panel dimensions, and an
off-by-one there would silently scribble into the next panel's slot.

Both are inline, in `Renderer.h`. **`Renderer.cpp` was not touched** — item A is editing it.

---

## 6. The overlay writes `kSplatExposure`, not a magic number

`resolve()` maps accumulated intensity to a ramp level as `255 / fullScale_`, and every caller in
the tree — `Simulation`, `App`, the WASM bindings — sets `fullScale_` to `kSplatExposure`. So the
overlay writes exactly `kSplatExposure` and lands on ramp level 255 by construction, with no
knowledge of palettes anywhere in `BeakerOverlay`.

Worth recording because the number is not the 7200 the constant's comment opens with: it is a
ratio against the reference spacing, and at the host defaults it measures **5184**. A literal here
would have been wrong at every spacing but one, and silently.

`beaker_overlay_resolves_to_the_top_of_the_water_ramp` asserts the full path — accumulate, draw,
resolve, compare against the palette's top stop — rather than asserting the intensity alone.

---

## 7. Faces are classified from normals, not from panel indices

`Geometry::cube` happens to put the top face at index 5, but a display node drives an arbitrary
subset of a six-panel table and the panel table is built from specs. So `BeakerOverlay::init`
classifies each panel by `dot(objectUp, panel.n)` — the inward normal of the top face points *down*
— and derives the panel-space direction of "up" from `dot(objectUp, u)` and `dot(objectUp, v)`.

That is what makes the arrows correct on a face whose `up` runs along its `i` axis instead of its
`j` axis, and `beaker_overlay_stays_on_the_panels_this_node_drives` covers the display-node case:
draw on the two faces this renderer owns, silently skip the other four.

Geometry as built: all four sides have `v = +Y`, the bottom cap has `v = -Z`, the top `v = +Z`.

---

## 8. The gate

A latch. Armed on entering beaker mode, released the first time `dot(down, objectDown)` clears the
tolerance, **never re-armed** — tilting is how you pour. `OrientationGate::update` returns the
armed state, so a frame gates in one line.

**Tolerance: 20 degrees (`kUprightCos = 0.9396926`). This is a judgement, not a measurement**, and
is labelled as such in the header. The reasoning: `MotionConfig::defaults` blends toward the
accelerometer at `alphaMax` 0.02 at 208 Hz, a ~0.35 s time constant, so a tolerance tight enough to
demand a few degrees would also demand the cube be held still, and this gate is meant to be
satisfied by someone standing a cube on a table. 20 degrees is roughly "visibly the right way up".
If it proves too loose or too tight it is one constant.

Tested both halves:

- `beaker_gate_releases_only_when_upright_and_never_re_arms` — armed at 90 degrees, armed at 45,
  armed at ~21 (just outside), released at ~11 (just inside), then **stays** released through a
  full inversion.
- `beaker_gate_holds_the_simulation_still_while_armed` — "the simulation does not step" as an
  assertion about state, not about a call count: 60 gated frames on a real `Simulation` leave
  `stateHash()` bit-identical, and the next 60 after release do not.

### The gate has no caller, and that is deliberate

`grep -ri beaker core platform` finds the tier in `Config.h`, the PlatformIO environment, and
nothing else. **There is no "beaker mode" to enter anywhere in the app, the firmware or the
browser.** So the latch is built, tested and unwired; whoever creates the mode (item E, or the App
layer) calls `arm()` on entry and gates `advance()` on `update(motion.down())`. Wiring it into a
mode that does not exist would have meant inventing the mode, which is not this item.

---

## 9. What was looked at

`out/beaker_armed_at_rest_net.ppm` and two more, from `partsim_beaker_ppm` (`mkdir -p out` first).
The cube net, `[+Y]` on top, the four sides as a strip, `[-Y]` below.

**Armed, 32x32, default build:** the four side faces each carry two full-height white verticals and
a white bottom row, and **no top row** — the rim is open, and the strip reads as four beakers
rather than four boxes. The bottom cap carries all four borders. The top face carries nothing but
the word `TOP`, floating, which is what "the top is open" looks like. Arrows are centred, one
stroke thick, about half the face tall, clearly pointing at the open end. `TOP` and `BOTTOM` are
legible over the fluid; `BOTTOM` sits on one line with room either side.

**Armed, 64x64:** the same at glyph scale 2 and the same physical size, visibly crisper. The arrow
scales with the face (`reach = h/4`), so it does not shrink into a tick mark.

**Armed, 32x32, chroma enabled:** white lines, **red** arrows, red `TOP`, red `BOTTOM` — pure red
over blue fluid, after the fix in §5, and the same red over an empty face.

**Released, tilted 35 degrees:** no glyphs, edge lines only, and the waterline stays continuous
across the seams — the overlay does not disturb the thing the seams are there to check.

---

## 10. Verification

```
ctest --test-dir build --output-on-failure        7/7
./build/platform/host/partsim_golden -q           f0021217 8e143d3b   (unchanged)
scripts/golden_hash.txt / golden_hash_esp32.txt   f0021217 8e143d3b
pio run -e {cube,cube-fast,panel,lite,qemu,beaker,master,display}   8/8 SUCCESS
```

Also built and run with `-DPARTSIM_ENABLE_CHROMA=1` (16/16 unit cases, plus the red-arrow case that
only exists there), because a `#if` branch that is never compiled is not a branch.

`scripts/build_wasm.sh` was re-run twice: the staleness guard fires on any `core/` edit and the
hash comparison alone cannot see a stale binary (handoff §8 trap 2).

---

## 11. Open, for whoever picks up next

1. **`env:beaker` needs `-DPARTSIM_ENABLE_CHROMA=1`** (§2). Until then the gate glyphs are
   monochrome on the only tier that runs beaker mode.
2. **Nothing enters beaker mode** (§8). The latch, the overlay and the tier exist; the mode does
   not.
3. **The top face while armed.** It shows `TOP` and no frame, which is correct per the brief, but
   `M4-PLAN.md` §6 still lists "does the top face render the liquid from above, or go dark" as
   open. The overlay does not decide that either way.
4. **3x5 is at its limit** (§4). Fine for `TOP`/`BOTTOM`; reconsider before a console overlay puts
   an M or an N somewhere it has to be read cold.
