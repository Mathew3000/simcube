# Roadmap — the capability ladder

One codebase, configured from a cheap single-board water cube up to whatever silicon exists later.
Named tiers select points on the ladder; the parameter space itself is deliberately open-ended.

Effort figures are implementation **and verification**, since an unverified tier is not a tier.

---

## The span

`kRestSpacing` is the axis. A half-full beaker (16 units) in the 32-unit cube:

| d | particles | S3 ms/step | 6.3× MCU | fps @30 Hz | verdict |
|---|---|---|---|---|---|
| 0.5 | 131072 | 22410 | 3557 | 0.3 | **runs on host**, no MCU for years |
| 1.0 | 16384 | 2801 | 445 | 2.2 | host only |
| 2.0 | 2048 | 350 | 56 | 18 | plausible top rung |
| 2.5 | 1049 | 179 | 28 | 35 | beaker target |
| 3.0 | 607 | 104 | 16 | 61 | **ships today** |
| 4.0 | 256 | 44 | 7 | 144 | lite tier |
| 5.0 | 131 | 22 | 3.6 | 281 | reads as blobs, not fluid |

The range that *looks* like fluid is roughly **d = 4.0 → 2.0**. Everything outside it is still
buildable, because the ceiling should be a property of the hardware and not of a type chosen years
earlier — see W0.

---

## W0. Open-ended parameter range — **DONE**

`ParticleIndex` is `uint16` below 65536 and `uint32` above, chosen at compile time, so what ships
today is byte-for-byte unchanged (both golden hashes held at `f0021217 8e143d3b`).

The wire format deliberately did **not** widen: it carries the count as `uint16` and `encodeFrame`
refuses a larger pool rather than wrapping it, because a truncated frame looks like particles
vanishing rather than like a protocol error. The format exists for the three-node cube at
~21 KB/frame; 131072 particles would be 650 KB/frame, where SPI broadcast is the wrong answer.

Verified by running the far end: d=0.5, 131072 particles, 309 ms/step on the host, nothing escaping
the box.

**Known next limit, not yet fixed:** the neighbour cache is a fixed stride, so a 160000 pool
reserves 41 MB. A CSR layout would cut it to what is used. Do it when a configuration needs it.

---

## W1. Feature flags — sand and fire compile out — **DONE**

`PARTSIM_ENABLE_SAND` and `PARTSIM_ENABLE_HEAT`, both defaulting on.

This pays in **memory**, not tidiness. Accumulation is `texels × channels × 2 B`:

**Measured**, device profile, whole image:

| configuration | host budget | firmware RAM |
|---|---|---|
| water + sand + fire | 204.5 KB | 171,660 B |
| water + sand | 189.1 KB | — |
| water + fire | 192.5 KB | — |
| **water only** | **177.1 KB** | **143,580 B** |

27.4 KB, taking free space from 25.5 KB to 52.9 KB — the difference between a display node driving
two faces and three.

Coupling is containable: `kSand` appears in 7 places, heat and `FieldGrid` in ~30.

**Risk:** `kChannelCount` reaches `Renderer`, `RenderState`, `SimFrame` and the palettes. Channel
indices must stay contiguous, and scenes that need a disabled feature must become unselectable
rather than silently empty.

---

## W2. Colour depth — **DONE**

Two independent things that "4 colours vs full colour" conflates:

- **HUB75 BCM bit depth** — already a driver field, 6-bit today. Trades DMA memory linearly: 4-bit
  at 64×64×2 faces is 64 KB against 96 KB.
- **Palette ramp resolution** — already data, not code.

Low risk, immediate memory payoff, no physics involvement.

---

## W3. Platform HAL — **DONE**

`platform/esp32/src/main.cpp` was 903 lines with `Serial`, `Wire`, FreeRTOS and the HUB75 library
inline. It is now 332, and what remains is bring-up and scheduling: constructing drivers, and
deciding when a frame runs.

Everything else moved to `platform/app` as `partsim::app::App` — the role logic, the IMU ring, the
console, the benchmark, the determinism sequence and the per-frame body of every task. It talks to
hardware through five interfaces (`Console`, `Clock`, `Display`, `MotionSensor`, `FrameLink`) plus
`SystemHooks`, and it knows nothing about any scheduler.

**Why it matters beyond tidiness:** the master drives no panels, so a non-ESP solver board needs
**no HUB75 driver at all** — it satisfies `Display` with `NullDisplay`, which is nothing. Writing
a HUB75 DMA driver for a new MCU family is 1–2 weeks and remains explicitly **out of scope**;
display nodes stay ESP32-S3.

**What the work found.** In full in [`W3-FINDINGS.md`](W3-FINDINGS.md) — including a 12 KB saving
that sounded obvious and was not there. The three worth carrying:

*A HAL with one implementation is a rename, not a seam.* So the host build is not a bonus, it is
the check: `platform/host/console_main.cpp` is a ~120-line second platform, and the `app_golden`
ctest drives the application layer's own `g` command there and compares the state hash against
`scripts/golden_hash.txt`. Had the extraction reached the physics, that test would say so without
a board attached.

*The refactor is free on the device and costs 80–832 B of SRAM.* Measured, per environment, before
and after:

| env | before | after | delta |
|---|---|---|---|
| `cube`, `cube-fast`, `panel` | 171,660 | 171,740 | +80 |
| `lite` | 143,580 | 143,660 | +80 |
| `qemu` | 171,616 | 171,696 | +80 |
| `beaker` | 172,176 | 172,288 | +112 |
| `master` | 192,064 | 192,176 | +112 |
| `display` | 115,928 | 116,760 | +832 |

Vtables, two 64-byte line buffers and members that can no longer be dead-stripped because a class
holds them. Against a 230 KB budget this is noise, but it is the wrong direction and it is written
down rather than rounded to zero.

*The benchmark did not move at all*, which was the actual worry — `splat`, `resolve` and `blit`
especially, since a change there would mean the refactor had altered the render path. Every column
reproduced to the hundredth of a millisecond on the same board (see the table in
`docs/W3-HANDOFF.md` §4).

---

## W4. Tier configurations — **DONE**

Named tiers, **not** free-form flags. 19 knobs is 2^19 combinations and none of them are tested; a
combination nobody built will break silently, and there will be no golden hash to catch it.

| tier | panels | materials | d | notes |
|---|---|---|---|---|
| `lite` | 32×32 ×6 | water | 4.0 | one S3, 4-bit colour, no heat field |
| `cube` | 32×32 ×6 | water, sand, fire | 3.0 | what ships today |
| `beaker` | 64×64 | water | 2.5 | 60 Hz — see below |
| `future` | 64×64 | all | 1.0 | host-verified only; no MCU runs it |

**30 Hz is NOT available to beaker mode**, which this work disproved. It was expected to be, since
the only recorded objection was that it collapses a sand heap and a beaker has no sand. It fails
for a second, independent reason at the finer spacing: same fixture, 2500 steps, d=2.5 settles to
mean|v| 0.031 at 60 Hz and to 1.011 at 30 Hz, still 0.616 after 5000. Finer particles need more
steps to shed momentum, not fewer.

`future` is **build-and-run only**. It builds, runs and conserves particles; it does not pass the
physics fixtures, because they assert a pool has reached rest within a step budget and a fine fluid
needs far more than a linear extension of one — d=1.0 still reads 1.69 after 7500 steps where d=3.0
reaches 0.05 in 2500. A property of the fluid, not a fault, but a green run there means "it did not
crash" and nothing more.

**The ongoing tax is golden hashes.** Each tier has different physics constants, so each needs its
own state and pixel hash on each target, and every future physics change regenerates N pairs
instead of one. Mitigation: device hashes only for tiers that ship; the extremes get "builds and
runs".

---

## Out of scope

- **Beaker mode itself** (Milestone 4) — ~1–2 weeks, independent of this work.
- **A HUB75 driver for a non-ESP MCU** — see W3.
- **Parallelising the solver** — the eight-colour scheme (`DECISIONS.md` P2) is worth ~1.8× and
  applies to every tier, but it is a separate change and it moves every hash.

---

## Order

W1 → W2 → W4 → W3. **All four are done and committed.**

W3 was left until last deliberately and was genuinely the largest piece: it rewrote the structure
of a 903-line file that had no test of its own beyond "the firmware boots". It got a fresh session
rather than being tacked onto one that had already moved the physics, the renderer, the config
surface and eleven test fixtures — and it now has a test of its own, which is the part of it that
outlasts the refactor.
