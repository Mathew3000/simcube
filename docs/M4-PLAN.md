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
PARTSIM_ENABLE_CHROMA  ->  kChannelCount = 3   (weight, chroma r, chroma g)
```

Same three channels, same memory as today's full build, but named for what they are and available
independently of whether sand exists. `kChBlue` is implied as `weight − r − g`.

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

### A. Chroma, mixing, and the split-kernel splat — **mine**

`core/`: `Particles` gains `cr[]`/`cg[]` (2 B/particle, 1 KB at device capacity). Mixing rides the
existing XSPH neighbour loop — it already gathers neighbours and accumulates a kernel-weighted
velocity, so a kernel-weighted chroma alongside it is nearly free and inherits the iteration order,
so it stays deterministic. `Renderer` splats weight at `kSplatRadiusWorld` and chroma at half that,
and `resolve` recovers colour as the ratio.

A separate `colourHash()` rather than folding chroma into `stateHash()`: the state goldens are
stable references across four tiers and three targets, and chroma has nothing to do with positions.

**Mine because** it is foundational, it is a look decision, and it touches `Renderer.cpp` and
`Solver.cpp` — the two files every other item would collide with.

### B. Open top and spill — *agent, blocked on A*

`SimVolume` gains an open-face flag; `clampToBox` stops clamping there. Particles past it leave via
the existing O(1) `removeAt` into an outbound list carrying position, velocity and chroma. Inbound
spill enters near the top at the **same horizontal position** it left, so a stream leaving one
corner arrives in the corresponding corner and reads as a pour rather than a teleport.

Blocked on A only for the chroma field on a spilled particle. Everything else is independent.

### C. Orientation gate, edge lines, glyphs — **agent, ready now**

A `BeakerOverlay` drawing into the accumulation buffers after the fluid, so it composites through
the same `resolve`. Needs a 3x5 bitmap font — there is none — for about ten characters, as a
`const` table. The gate is a latch: armed on entering the mode, released the first time gravity is
within tolerance of the cube's own down axis, never re-armed.

**Ready now** because it is new files plus one small `Renderer` hook, and touches nothing A does.
See `M4-C-HANDOFF.md`.

### D. Chaining over ESP-NOW — *agent, blocked on B*

Radio on the master only (`DECISIONS.md` D34). Spill packets are tiny and neither latency- nor
loss-critical, but a dropped packet in a closed loop silently drains total volume — so carry a
cumulative spilled count per link and let the receiver make up a shortfall. Needs two devkits.

### E. Browser, UI, reset — *agent, blocked on A+B+C*

N beakers on one page, spill lists wired into a ring through the same core code the firmware runs.
Per-beaker reset, colour pickers, fill readout. This is where chaining gets debugged.

### F. `SpiFrameLink` bring-up — **agent, ready when two boards are wired**

Not M4, but the largest block of never-executed code in the repo and the thing the PCB rests on.
Five jumper wires. See `SPI-HANDOFF.md`.

---

## 4. Order, and why

```
   A (mine) ────────┬──> B ──> D
                    │         
   C (agent, now) ──┴──> E
   F (agent, now, needs wiring)
```

**A first** because B, D and E all carry chroma, and because if the split-kernel look is wrong it
is better to find that before three items are built on it.

**C and F run alongside A from the start.** C is new files plus a hook; F is `platform/esp32` only.
Neither touches what A touches.

Handoffs for **B, D and E are deliberately not written yet.** A handoff for work whose foundation
does not exist would be speculative about the interfaces it must use, and this project has a
documented habit of that going wrong. They follow when A and B land.

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
- **Does the top face render the liquid from above, or go dark to read as "open"?** The request only
  says it draws no border lines.
- **How coarse can the particles stay?** F3 says colour resolution is `32/d` with the split kernel.
  At `cube`'s d=3.0 that is ~10 regions. Whether ten reads as mixing is a judgement, and it decides
  whether beaker mode runs at 8.5 fps or at 26.
