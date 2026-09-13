# What the simulation master has to be

**A specification for a part that does not exist yet, so that a real one can be checked against
numbers rather than against a marketing page.**

Extracted from [`CUBE-PCB.md`](CUBE-PCB.md) so that checking a part needs one document rather than
three. **This file is authoritative for the requirements** (§3-§4, §6-§7) and the PCB document
points here for them. §5's candidate list is reproduced from `CUBE-PCB.md` §13.1, which stays the
place a part choice is actually recorded — it is here because a list of requirements with no
example of what meets them is hard to calibrate against. Everything marked measured was measured on hardware — an ESP32-S3
DevKitC-1 — and the derivations say what they assume.

Companion documents: [`RESOURCES.md`](RESOURCES.md) for the memory and CPU budget and the
measurement method, [`DECISIONS.md`](DECISIONS.md) for why each choice was made (P4 is the MCU
question, P5 is what beaker mode costs), [`SIMULATION.md`](SIMULATION.md) for what the solver
actually does.

---

## 1. The board this describes

The cube is one **simulation master** plus three **display boards**. The master runs the solver and
nothing else: it drives no panels, so it needs **no HUB75 peripheral, no large DMA buffer and no
display clock**.

That is what makes it the one board where a different vendor is realistic. The display boards stay
ESP32-S3 because the HUB75 driver exists only for that family; the master has no such tie.

---

## 2. The unit: particle-steps per second

One particle advanced by one solver step. Hardware-independent, and everything the fluid does is a
multiple of it:

```
required throughput = particles × substeps_per_frame × target_fps
```

**Measured baseline: an ESP32-S3 at 240 MHz delivers ~6 900 particle-steps/s.** 74.65 ms for 512
particles in one step with the solver's hazard-free passes on the second core (`DECISIONS.md` D44).
Independently reproduced at 71.56 ms — 7 155/s — on a different build (the beaker tier, with dye
mixing in the XSPH pass), which is the same number within the difference between the two builds.

It was 5 849/s before the second core landed, and every multiplier below moved with it.

### What each rung of the ladder needs

| tier | `d` | representative scene | particles | needed | **vs S3** | solver working set |
|---|---|---|---|---|---|---|
| `lite` | 4.0 | water tank | 158 | 9 480/s | **1.4×** | 28 KB |
| `cube` | 3.0 | water tank | 375 | 22 500/s | **3.3×** | 65 KB |
| `beaker` | 2.5 | half-full beaker | 1 049 | 62 940/s | **9.2×** | 177 KB |
| `future` | 1.0 | half-full beaker | 16 384 | 983 040/s | **143×** | 2.7 MB |
| `max` | 0.5 | half-full beaker, 60 fps | 131 072 | 15 728 640/s | **2 293×** | 38 MB |

All at two substeps per frame and 30 fps except `max`.

**Read the first two rows carefully: `lite` is essentially there** — measured live at 26.3 fps on a
real scene — **and `cube` still needs 3.3× an S3 and does not have it.** That gap is the headline
requirement, and the software levers are spent: coarser particles, the neighbour cache, the
divide-free solver, the second core, the row-walking blit and the splat bound together bought about
12×, and nothing of that size is left.

Working set is solver-only — particles, neighbour cache, sort grid, heat field. It excludes the
accumulation buffers and the DMA framebuffer, which live on the display boards.

### And a correction the beaker row understates

`DECISIONS.md` P5, measured after the table above was written: a **completely full** 32-unit vessel
at `d = 2.5` is **1 905 particles**, not the 1 049 of a half-full one. The device's own sweep, on
two cores with no panels attached:

| particles | sim/step | fps at 2 substeps |
|---|---|---|
| 128 | 6.32 ms | 78.9 |
| 256 | 22.86 ms | 21.8 |
| 384 | 45.88 ms | 10.9 |
| 512 | 71.56 ms | **7.0** |

So one S3 gives a beaker **a quarter full at 7 fps**, or a two-unit film in a 32-unit box at a
comfortable frame rate. Cost grows as `n^1.77`, so the shortfall is worse than proportional.

### And a caveat the whole section depends on

**Every requirement above is a property of the representation, not of the picture.** The figures
assume the carrier liquid is simulated as particles. `DESIGN-SUGGESTIONS.md` §13 measures the
alternative -- dye as a small fixed-point field, with the water implicit -- at **12-29% of one S3
core for a full volume**, against the 0.68 fps PBF would give for the same full beaker. The
`n^1.77` growth that makes the shortfall "worse than proportional" is exactly what disappears: a
full field and an empty one cost the same.

That does not make this document wrong; it makes its scope explicit. If the cube keeps PBF for the
bulk liquid, the 9.2x stands. If it adopts the field, the binding constraint moves to the **blit**
-- ~9.0 ms per display node per frame, unaffected by the solver -- and REQ-MCU-1's hardware divide
and square root stop being decisive, because a fixed-point kernel uses neither.

Settle which representation ships before buying silicon against this table.

---

## 3. REQ-MCU: what a candidate must have

### REQ-MCU-1 — Single-precision hardware FPU, **with hardware divide and square root**

Non-negotiable, and the easiest thing to get wrong: a datasheet saying "FPU" implies neither. The
ESP32-S3's FPU has neither — GCC emits calls to `__divsf3` and `sqrtf`, and the inner loop makes
~260 of them per particle per step.

**How to check**, and it takes ten minutes: compile `core/src/Solver.cpp` for the candidate and run
`nm -u` on the object file. **A clean candidate shows no libm symbols.** Cortex-M7 shows none;
Xtensa LX7 shows both.

### REQ-MCU-2 — The working set must fit zero-wait-state memory

TCM, or SRAM with no flash-XIP stall in the path. The solver is a scattered gather: **88 candidate
reads per particle per step, of which 27% are useful.** Served through a small cache with a slow
backing store, none of the throughput figures above survive. Sizes per tier are in §2's table —
28 KB for `lite`, 177 KB for `beaker`.

### REQ-MCU-3 — Single-core throughput is what counts, today

The solver is Gauss-Seidel: each correction is applied in place and later particles see it, so
extra cores currently buy **nothing**. Judge a candidate on `clock × IPC`, not on core count.

This is the one requirement a software change could lift. The eight-colour cell partitioning
(`DECISIONS.md` P2) makes cores usable and is worth ~1.8×; it was built once, measured at only
1.08×, and reverted — the correct version needs 27 colours, not 8. Until it exists, cores past the
first are worth nothing to this workload.

### REQ-MCU-4 — Deterministic scalar float

All three build targets compare a bit-identical state hash, so the part must support
`-ffp-contract=off` semantics: **no unconditional fused multiply-add, and no flush-to-zero that
cannot be turned off.**

### REQ-MCU-5 — A radio, or a companion budgeted for one

The master needs one for beaker-mode chaining. **ESP-NOW costs 30.4 KB of internal heap, measured**
— not the ~55 KB earlier estimates carried. A part without a radio implies a companion: for
ESP32-P4 that is Espressif's own C6/C5 over SDIO. Budget the second package and its pins, not just
the first.

---

## 4. What does NOT help, and should not be paid for

Worth stating explicitly, because the specs that sell a modern MCU are mostly irrelevant here.

- **NPU / TPU / "AI accelerator."** Integer or bf16 matrix-multiply hardware. This workload is
  scalar single-precision with an irregular neighbour gather and a sequential dependency between
  particles. None of it maps. An NPU contributes **exactly zero**.
- **SIMD / vector extensions** (Helium, RVV, the S3's PIE). The gather is irregular and
  Gauss-Seidel serialises the writes, so there is nothing to vectorise without first doing
  REQ-MCU-3's re-colouring — and even then the gain is in parallel *particles*, not lanes.
- **GPU or 2D blitter.** The display boards do the drawing, and their cost is a fixed ~30 ms floor
  that no processor removes.
- **Large flash, PSRAM bandwidth, high core count, Ethernet, USB 3.** None are on the critical path
  for this board.

---

## 5. Candidates, if the S3 is not enough

Ranked by measured instruction count and the CPI the S3 was measured at; the derivation and its
assumptions are in `RESOURCES.md` §5.1. **Only the relative figures are measured — the CPI of every
non-Espressif part below is an assumption, and it is the term that decides the ranking.**

| part | core | MHz | vs S3 | reaches | note |
|---|---|---|---|---|---|
| ESP32-S3 | Xtensa LX7 | 240 | **1.0×** | `lite` | measured; the baseline |
| ESP32-P4 | RISC-V ×2 | 400 | ~2.3× | `lite` | no radio at all — needs a C6/C5 companion |
| STM32H743 | Cortex-M7 | 480 | ~2.9× | `cube` | LQFP, internal flash, 4-layer |
| STM32H7S3 | Cortex-M7 | 600 | ~3.6× | `cube` | |
| i.MX RT1062 | Cortex-M7 | 600 | ~3.8× | `cube` | |
| i.MX RT1176 | Cortex-M7 | 1000 | ~6.3× | `beaker` | BGA, external flash, 6+ layers |
| *(aspirational)* | — | — | ~2 700× | `max` | not a microcontroller — §6 |

Two practical notes the throughput column does not carry:

- **Package and layer count matter more than the last 20% of speed.** An STM32H7 in LQFP with
  internal flash is a 4-layer board an experienced hobbyist finishes; a 0.65 mm BGA with mandatory
  external QSPI is not the same project. The step from `cube` to `beaker` is also a step from LQFP
  to BGA.
- **Cortex-M7 wins here mostly on tightly-coupled memory** (REQ-MCU-2), not on clock. A part with
  the same clock but a small cache and flash-XIP behind it will not deliver the figure in the
  table.

**None of this is a decision.** The master stays an ESP32-S3 until something forces otherwise, and
the two software levers — the 27-colour parallelisation and the render-floor work — are worth more
than one step down this table and cost no silicon.

---

## 6. Sizing an aspirational part

For the top of the ladder — `d` = 0.5 at 60 fps — the requirement is **~2 700× an ESP32-S3 in
sustained scalar single-precision throughput, with 38 MB of low-latency working memory.**

A useful sanity check on what that means: eight cores at 2 GHz is ~66× the S3 in clock-cores, and
perhaps 100× once better IPC is allowed for. **That is still 27× short** — and it only counts at
all if REQ-MCU-3 has been lifted first, since without the re-colouring the other seven cores do
nothing.

So the honest statement: the `max` rung is **not a microcontroller target at all.** It wants a
many-core application processor or a GPU, and it is in the ladder to keep the parameter space open
rather than because a part is expected.

The rungs a real part can reach are `lite` through `beaker`: **1.4× to 9.2× an ESP32-S3**, which is
squarely in Cortex-M7 territory.

And if beaker mode — a vessel that actually looks full rather than a quarter full — is the reason
to change silicon, the requirement is **8–10×**, not the 2× that would comfortably serve the
existing tiers. Nothing in §5 reaches it except the i.MX RT1176, and that is a BGA.

---

## 7. Checklist for evaluating a part

In the order that fails fastest:

1. `nm -u` on a compiled `core/src/Solver.cpp` — **no libm symbols** (REQ-MCU-1).
2. Zero-wait-state memory ≥ the working set of the tier you want, from §2 (REQ-MCU-2).
3. `clock × IPC` against 6 900 particle-steps/s × the multiplier you need (REQ-MCU-3).
4. `-ffp-contract=off` honoured, FTZ defeatable (REQ-MCU-4).
5. Radio, or a companion part and its pins budgeted (REQ-MCU-5).
6. Package and layer count — see §5.
7. Then, and only then, build one and run `scripts/check_esp32_budget.sh`'s equivalent plus the
   golden determinism sequence. **The hash is the acceptance test**: a part that produces a
   different one is not running the same simulation, whatever its throughput.
