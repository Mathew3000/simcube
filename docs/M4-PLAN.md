# Milestone 4 — beaker mode

The cube becomes an open-topped vessel: white edge lines on every face but the top, liquid only,
and tilting it pours the contents out. Several cubes chain in a loop — what spills from cube 1
lands in cube 2 and so on, wrapping back to 1 — each starting with its own colour, which **mixes**
as fluids combine, so red pouring into blue converges to pink. A single cube loses what it spills.

Entering the mode while the cube is not upright shows red arrows on the four side faces, `TOP` and
`BOTTOM` on the other two; the simulation starts once it has been held upright and **never gates
again**, because tilting is how you pour.

Every beaker gets a reset: refill, restore the original colour.

This supersedes the M4 sketch in `~/.claude/plans/pure-yawning-eclipse.md`, which was written
before the capability tiers existed and is wrong in one structural way (§2).

---

## 1. Prerequisites, all met

| | state |
|---|---|
| particle count that runs | `lite` measured live at **26.3 fps**, `cube` at 8.5 |
| render floor | 22.69 ms at 384 particles, splat three quarters of it |
| second core | solver hazard-free passes parallel, 1.17x |
| capability tiers | `lite` / `cube` / `beaker` / `future`, each with its own goldens |
| platform HAL | app logic runs on the host, so all of this is host-testable |

---

## 2. The correction the tiers force

The old sketch reinterpreted the three accumulation channels:

| channel | normally | beaker mode |
|---|---|---|
| 0 | water intensity | weight |
| 1 | sand intensity | chroma r |
| 2 | heat intensity | chroma g |

**That no longer works.** W1 made sand and fire compile out, and the `beaker` tier sets both to 0 —
so `kChannelCount` is **1**, not 3, and there are no spare channels to reinterpret. The saving was
the point: it is 27.4 KB, and it is why a display node can consider three faces.

So beaker mode needs its **own** channel configuration rather than borrowing dead ones:

```
PARTSIM_ENABLE_CHROMA  ->  kChannelCount = 4   (weight, chroma r, chroma g, chroma b)
```

**Four, not three, and blue is stored rather than implied** — which the sketch got wrong for a
reason that no longer applies. Colour resolves as the ratio of the chroma channels, and the
denominator that ratio needs is the chroma channels' *own* narrow-disc weight, which is exactly
`cR + cG + cB`. Imply one and the denominator becomes circular. The sketch crammed into three to
dodge "+16 KB on a display node at 200 KB of 230"; W1 dropped that node to 115 KB and the
constraint went with it.

`PARTSIM_ENABLE_CHROMA` is set **by the tier**, not by an environment: a beaker whose liquid has no
colour cannot mix, and with one channel the gate's red arrows render white (`DECISIONS.md` D52).

### And a refinement the sketch predates

**Colour resolution is set by the blob, not by the particle** (`DECISIONS.md` F3). Chroma splatted
through the same kernel as brightness smears over `2d`, giving `32/(2d)` ≈ **5 distinguishable
regions across the whole cube** at the shipping spacing — red into blue would read as a few coloured
lumps converging, not as mixing.

So the chroma channels get a **narrower kernel than the weight channel** (`DECISIONS.md` P1):
brightness at `2d` because a continuous surface needs it, chroma at `d`. Colour resolution doubles
to `32/d` at the same particle count — worth 8x the particles it would otherwise take.

This is a **design decision inside block A, not a separate step**, and it is the thing most likely
to look wrong. Judge it on a render before building anything on top.

---

## 3. Work items

### A. Chroma, mixing, and the split-kernel splat — **DONE** (`9c88b27`, `b0f8323`)

`core/`: `Particles` gains `cr[]`/`cg[]` (2 B/particle, 1 KB at device capacity). Mixing rides the
existing XSPH neighbour loop — it already gathers neighbours and accumulates a kernel-weighted
velocity, so a kernel-weighted chroma alongside it is nearly free and inherits the iteration order,
so it stays deterministic. `Renderer` splats weight at `kSplatRadiusWorld` and chroma at half that,
and `resolve` recovers colour as the ratio.

A separate `colourHash()` rather than folding chroma into `stateHash()`: the state goldens are
stable references across four tiers and three targets, and chroma has nothing to do with positions.

Landed with three corrections worth carrying into B: dye is **8.8 fixed point**, because a byte
destroyed 98% of it over 600 steps — diffusion moves fractions of a unit per particle per step and
truncating each one throws the remainder away. Mixing is Gauss-Seidel, so dye is conserved only to
**5%, and that drift plateaus** rather than accumulating. And the split kernel is worth less than
claimed: it sharpens a dye boundary from 4 texels to 2 only when the gradient is sharper than the
weight kernel, so it buys a crisp **pour** — which is exactly what B creates — and nothing at all on
a settled mix.

### B. Open top and spill — **DONE** (`623c71a`, `d848873`)

`SimVolume` gains an open-face flag; `clampToBox` stops clamping there. Particles past it leave via
the existing O(1) `removeAt` into an outbound list carrying position, velocity and chroma. Inbound
spill enters near the top at the **same horizontal position** it left, so a stream leaving one
corner arrives in the corresponding corner and reads as a pour rather than a teleport.

**DONE.** `SimVolume::clampInto` replaced the free `clampToBox`, which is what found the FOURTH
call site the brief did not count: `wallDensityAt` was compensating for a wall that is not there,
inventing half a rest density of neighbours above the rim and pushing the surface away from the
face the liquid leaves through — worth ~2x on the pour rate, and it pours either way, which is why
nothing noticed. See `DECISIONS.md` D56-D60.

### C. Orientation gate, edge lines, glyphs — **DONE** (`773dede`)

A `BeakerOverlay` drawing into the accumulation buffers after the fluid, so it composites through
the same `resolve`. Needs a 3x5 bitmap font — there is none — for about ten characters, as a
`const` table. The gate is a latch: armed on entering the mode, released the first time gravity is
within tolerance of the cube's own down axis, never re-armed.

**DONE** — `773dede`. `BeakerOverlay`, `OrientationGate`, a 45-glyph 3x5 font, 16 tests, and two
inline hooks in `Renderer.h` (`addAccum` for weight, additive; `setAccum` for dye, overwriting).
The overlay is an explicit call between `accumulate()` and `resolve()`, not part of
`Simulation::render()`, which is why the pixel golden is unmoved. See `DECISIONS.md` D48-D52.

### D. Chaining over ESP-NOW — **DONE** (`635357e`, and the pump)

Radio on the master only (`DECISIONS.md` D34). Spill packets are tiny and neither latency- nor
loss-critical, but a dropped packet in a closed loop silently drains total volume — so carry a
cumulative spilled count per link and let the receiver make up a shortfall.

**DONE.** The wire format landed first (`635357e`); what completes it is `SpillChain` — the pump
between `Simulation::spill()` and the carrier — plus a `beaker-chain` environment, a `d <r> <g>`
console command for a cube's own dye and an `o <x> <y> <z>` one to tilt a board that has no IMU.

Measured on two devkits, 20 seconds of pouring: **179 spilled, 179 sent in 140 packets, 179
received; shortfall, made-up, refused and bad all zero.** 307 → 128 particles on the sender,
307 → 486 on the receiver — conserved exactly.

Two defects the measurement found, both of which every counter in the system had agreed were fine:
the pump ran once per frame where the queue is cleared twice (half the pour crossed as clones, see
`DECISIONS.md` D62), and a beaker filled to 100% of its pool has nowhere to put an arrival (D63).

### E. Browser, UI, reset — **agent, ready now** (`M4-E-HANDOFF.md`)

N beakers on one page, spill lists wired into a ring through the same core code the firmware runs.
Per-beaker reset, colour pickers, fill readout. This is where chaining gets debugged.

### F. `SpiFrameLink` bring-up — **agent, ready when two boards are wired**

Not M4, but the largest block of never-executed code in the repo and the thing the PCB rests on.
Five jumper wires. See `SPI-HANDOFF.md`.

---

## 4. Order, and why

```
   A (done) ────────┬──> B (done) ──> E (agent, now)
                    │                 ↑
   C (done) ────────┘    D (done) ────┘
   F (agent, needs wiring)
```

A and C are in. **B and D run alongside each other**, meeting at one shared type — the spill
packet — which D defines in `core/` so both ends agree on it rather than converging by accident.
B produces those packets and consumes them; D carries them between cubes.

E stayed unwritten until B and D landed, because a browser handoff whose spill interface did not
exist yet would have been speculative about the thing it is mostly made of. Both have landed, and E
now inherits a pump (`SpillChain`) rather than having to invent one in JavaScript — which is the
same argument that put `SimFrame` in core for the multi-node preview.

---

## 5. Verification

- **Colour**: pouring red into blue converges toward magenta and **conserves total chroma weight** —
  mixing must not create or destroy dye.
- **Spill**: total particle count conserved around a 3-beaker ring across many tilts, and strictly
  decreasing in single-cube mode. This is the assertion that catches a leak in the open-face path.
- **Gate**: held sideways, glyphs show and the particle count stays zero; held upright once, the
  sim starts and keeps running through a full inversion.
- **Overlay**: edge lines land on the expected texels on all five closed faces and none of the top
  face's own edges.
- Both golden hashes unchanged in the non-beaker tiers throughout. Beaker mode is a mode, not a
  change to the existing ones.
- Per-tier: `lite`, `cube`, `beaker` and `future` all build and pass.

---

## 6. Open questions, to settle on a render rather than on paper

- **Does the split kernel read as colour in the fluid, or as colour floating on it?** The risk is
  chroma looking detached from brightness at the surface.
- ~~**Does the top face render the liquid from above, or go dark to read as "open"?**~~ **Settled
  by M4-B's render**: it draws the liquid, and during a pour it is the most informative face on the
  cube.
- **How coarse can the particles stay?** F3 says colour resolution is `32/d` with the split kernel.
  At `cube`'s d=3.0 that is ~10 regions. Whether ten reads as mixing is a judgement, and it decides
  whether beaker mode runs at 8.5 fps or at 26.

- **Is a quarter-full beaker at 7 fps acceptable?** Measured since this plan was written
  (`DECISIONS.md` P5): a full 32-unit vessel at `d = 2.5` is **1905 particles**, and the device
  runs 512 of them at **7.0 fps** and 128 at 78.9. So the hardware beaker is a quarter full at
  best, or a 2-unit film at a comfortable frame rate. The spacing that makes mixing legible and the
  spacing that fills the vessel are a factor of two apart. **Look at a quarter-full beaker before
  concluding anything** — it may read perfectly well, and it is the only one of the three ways out
  that costs nothing.
