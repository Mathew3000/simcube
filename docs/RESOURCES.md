# Resource allocation — master vs display

Companion to [CUBE-PCB.md](CUBE-PCB.md). Where every byte, cycle, pin and megabyte-per-second
lives, per ESP32-S3 role. **[M]** = measured, **[A]** = assumed and needs bench confirmation.

**Revision:** 0.1. Supersedes the "one S3 cannot drive six 64×64 panels" claim in CUBE-PCB §1 —
see §1 below.

---

## 1. Correction: PSRAM does change the memory answer

CUBE-PCB and the milestone plan both said a single S3 cannot drive six 64×64 panels, and that
`SPIRAM_DMA_BUFFER` must never be defined. **The first is wrong and the second was overstated.**
The HUB75 library provides that flag precisely so large arrays can put the framebuffer in PSRAM,
and it works.

What is true is narrower: with the DMA buffer in *internal* SRAM, six 64×64 faces need 288 KB of
buffer against ~230 KB of usable internal RAM, so **that** configuration is impossible. Move the
buffer to PSRAM and the constraint moves from memory to bus bandwidth and blit cost.

Two figures were also wrong in CUBE-PCB and are corrected here:

| | was | is | why |
|---|---|---|---|
| Master internal SRAM | 172.6 KB | **112.6 KB** <span title="measured">[M]</span> | the report charged it 48 KB of DMA buffer and 12 KB of staging for a HUB75 chain it does not have |
| Splat cost, 6 faces at 64×64 | 4.84× the 32×32 baseline | **1.91×** [M] | 4.84× is the cost per particle-panel pair *in range*; the panel rejection test dominates and most particles are out of range of most faces |

Splat was therefore never the reason to split the work. **The blit is.**

---

## 2. The topology question, with numbers

All figures assume the render-only state container (see §4.1). Without it add 55.7 KB to every
display row.

### Internal SRAM per display node [M]

| faces driven | accumulator | DMA buffer | internal, DMA inside | internal, DMA in PSRAM |
|---|---|---|---|---|
| 1 | 24.0 KB | 48.0 KB | 116.4 KB | 68.4 KB |
| 2 | 48.0 KB | 96.0 KB | **188.4 KB** | 92.4 KB |
| 3 | 72.0 KB | 144.0 KB | 260.4 KB — over | 116.4 KB |
| 6 | 144.0 KB | 288.0 KB | 476.4 KB — over | **188.4 KB** |

Shared regardless of face count: **44.4 KB** — staging, render-only particle state, the heat
field, geometry and LUTs. It is shared because the particle payload arrives whole over SPI: any
particle can light any face, so there is nothing to cull per node.

The symmetry is worth pausing on. **Three boards driving two faces each, and one board driving all
six with the buffer in PSRAM, have the identical 188.4 KB internal footprint.** The choice between
them is not memory at all — it is CPU time and bus contention against board count.

### What each option costs

| option | boards | blit/node | PSRAM traffic | risk |
|---|---|---|---|---|
| 3 × 2 faces, DMA internal | 4 | 4.4 ms (13 % of frame) | none | lowest — no new mechanism |
| 2 × 3 faces, DMA in PSRAM | 3 | 6.7 ms (20 %) | 9.6 MB/s | moderate |
| 1 × 6 faces, DMA in PSRAM | 2 | 13.3 ms (40 %) | 19.2 MB/s | highest |

Blit at ~130 cycles per `drawPixelRGB888` at 240 MHz, on a 33 ms frame. [A]

### The prerequisite that makes PSRAM viable at all

`drawPixelRGB888` performs a read-modify-write on **each of the six bitplanes**, at six widely
separated addresses. Per frame:

| faces | pixels | PSRAM accesses/frame | rate |
|---|---|---|---|
| 2 | 8 192 | 49 152 | 1.47 M/s |
| 3 | 12 288 | 73 728 | 2.21 M/s |
| 6 | 24 576 | 147 456 | 4.42 M/s |

Scattered small accesses are the one pattern PSRAM handles worst — each is a potential cache miss
at ~100 ns. At six faces that is on the order of 15 ms of stall per frame on its own, on top of
the 13.3 ms of blit work.

> **A PSRAM framebuffer requires the row-walking blit.** With per-pixel `drawPixelRGB888` it
> cannot work. `ChainMap::row` already returns the start coordinate and per-texel step for exactly
> this, so the mapping does not have to change — but the row writer itself does not exist yet, and
> it is a hard prerequisite rather than an optimisation.

### Recommendation for the PCB

Do not let the layout foreclose the choice, because the deciding numbers can only be measured on
hardware. **Give the display board two HUB75 outputs, each able to drive a chain of one to three
panels.** Then the same board covers every option above and the decision moves from layout time to
bring-up time:

- populate 3 boards, one panel per output → the safe configuration
- populate 2 boards, one and two panels → the middle
- populate 1 board, three panels per output → the ambitious one

The cost is the 5 V rail: a board that might drive six panels needs **≥12 A** rather than 8 A, and
the APL cap must be enforced *from boot* — including during the bring-up test pattern — or an
unlimited board would ask for 24 A at full white.

12 A is not padding. An 8 A rail cannot run the measured worst-case scene (kettle, 31 % duty) at
full brightness — it caps out at brightness 230 — so the limiter would be constraining real content
rather than guarding against pathological input. At 12 A every real scene runs at 255 untouched.
Full derivation in [CUBE-PCB.md](CUBE-PCB.md) §4.4; see also REQ-PWR-1/3/4.

---

## 3. CUBE-MASTER

### Internal SRAM — 112.6 KB [M]

| pool | KB | note |
|---|---|---|
| `Particles` (SoA) | 51.3 | full solver pool: positions, velocities, predicted, λ, material |
| `FieldGrid` ×2 | 20.9 | ping-pong; advection needs both buffers here |
| `SpatialHash` | 10.5 | counting-sort grid + index |
| `Renderer` | 24.3 | one render slot, kept for a diagnostic face — droppable for 24 KB |
| `scratch_` | 5.0 | sort scratch |
| `Geometry`, `Solver` | 0.5 | |
| **total** | **112.6** | of ~230 KB usable → **117 KB spare** |

No DMA buffer, no staging: it drives no panels.

### PSRAM — optional, none required

Nothing the master does needs it. The particle pool must **stay internal**: the solver's neighbour
gather is random access across the whole array, three times per step, and PSRAM latency there would
dominate the frame. Legitimate uses, all cold:

- staging a display-node firmware image before pushing it over SPI
- diagnostic capture — particle or IMU traces for replay
- log buffers

### CPU — per 33 ms frame

| task | core | prio | work |
|---|---|---|---|
| `simTask` | 1 | 2 | IMU fusion, 2 × `stepFixed`, `SimFrame` encode, SPI DMA kick |
| `imuTask` | 0 | 3 | LSM6DSOX register reads at 208 Hz, **integer-only** |
| `loop` | 1 | 1 | serial console |

Float lives only on core 1 and is pinned, because FreeRTOS on Xtensa saves FPU context lazily.
`imuTask` does no float at all, so the two cores never contend for the FPU. It runs at *higher*
priority than the simulation despite less work: a 4.8 ms sample deadline versus a late frame.

Stacks: `simTask` 6144 B, `imuTask` 2560 B, `loop` 8192 B (Arduino default), plus FreeRTOS idle
and timer tasks ≈ **20 KB** total. Counted separately from the pools above.

### Peripherals

| | |
|---|---|
| SPI2 (host) | to display nodes, 20 MHz, DMA, 3 CS |
| I2C0 | LSM6DSOX + charger + pack AFE on one bus |
| USB-Serial-JTAG | console and flashing |
| WiFi (ESP-NOW only) | beaker chaining. **~55 KB of internal heap** — the master's 117 KB spare is what pays for it |
| GDMA | SPI2 TX only |
| LCD_CAM | unused |

### Bandwidth out

21 KB per frame, 630 KB/s at 30 fps, against 2.5 MB/s at 20 MHz. **25 % utilised** — the fallback
to 10 MHz still leaves 2× headroom. [M]

### Flash

510 KB of 3 MB (`huge_app`) for the current single-node build. [M] ESP-NOW and the frame encoder
add little; there is no pressure here.

---

## 4. CUBE-DISPLAY

### 4.1 Internal SRAM

Per §2, depending on faces driven and where the DMA buffer lives. The shared 44.4 KB:

| pool | KB | note |
|---|---|---|
| staging | 12.0 | one 64×64 face of RGB, reused across faces |
| render-only particle state | 20.0 | 1280 × 16 B: position, velocity, material |
| heat field, `cur_` only | 10.4 | read sequentially by `splatField` |
| geometry + LUTs | 2.0 | full six-panel table — the container must match every node |

**What a display node must NOT carry**, and what it costs today because it does — 55.7 KB, which is
the whole reason the node was over budget:

| carried | KB | why it is dead weight |
|---|---|---|
| `Particles` `sx/sy/sz`, `lam` | 31.3 | predicted positions and Lagrange multipliers — solver-only |
| `FieldGrid` second buffer | 10.3 | ping-pong for advection; only `cur_` is splatted |
| `SpatialHash` | 10.5 | a neighbour grid it never builds |
| `scratch_` | 3.6 | the sort's scratch space |

### 4.2 PSRAM

| candidate | verdict |
|---|---|
| HUB75 DMA buffer | **Yes, and this is the point.** Requires the row-walking blit (§2). Sequential DMA read; contention shows as reduced max refresh, not corruption |
| Accumulator | **No.** Zeroed in full every frame — 48 KB per face per frame, 1.5 MB/s of pure writeback at two faces — then scattered read-modify-write on top |
| Render-only particle state | Possible: written once from SPI, read sequentially. But it is only 20 KB, so there is no reason |
| Heat field `cur_` | Possible, sequential read. Also small |
| Staging buffer | No: it is the DMA feed and is rewritten per face |

Two constraints that apply to anything in PSRAM: it cannot be touched from an ISR or while the
cache is disabled (during a flash write), and octal PSRAM permanently consumes **IO33–37** whether
you use it or not.

### 4.3 CPU — per 33 ms frame

| stage | 2 faces | 3 faces | 6 faces |
|---|---|---|---|
| SPI receive + decode | ~1 ms [A] | ~1 ms | ~1 ms |
| splat (particles + heat) | 0.64× base [M] | 0.70× base [M] | 1.91× base [M] |
| resolve | ~1 ms/face [A] | | |
| blit, internal FB | 4.4 ms | 6.7 ms | 13.3 ms |
| blit, PSRAM FB, row-walking | needs measurement | | |

Splat figures are relative to six faces at 32×32 — a ratio, because host absolute times mean
nothing for Xtensa. The counter-intuitive part is real: **two faces at 64×64 cost less than six at
32×32**, because the per-particle panel rejection test dominates and there are a third as many
panels to reject against.

No solver, no field advection, no IMU. The display node is receive → splat → resolve → blit.

### 4.4 Peripherals

| | |
|---|---|
| LCD_CAM + GDMA | HUB75 output. On the S3 this is what makes 6-bit at >100 Hz affordable |
| SPI2 (device) | from the master |
| GPIO ×2 | board ID straps |
| WiFi / BT | **off.** No display node has a radio |
| I2C | unused |

### 4.5 GPIO — 20 of 26 available

| function | pins |
|---|---|
| HUB75 RGB | 4, 5, 6, 7, 15, 16 |
| HUB75 address | 17, 18, 8, 9, **21 (E)** |
| HUB75 control | 10 (LAT), 11 (OE), 12 (CLK) |
| SPI device | 13 (SCK), 14 (MOSI), 47 (MISO), 48 (CS) |
| Board ID straps | 38, 39 |

A second HUB75 connector shares all 14 signals — panels chain, they do not each need a port. Only
the chain length changes.

Unavailable on an N16R8: **IO33–37** (octal PSRAM), IO26–32 (module flash), IO19/20 (USB),
IO43/44 (UART0), IO0/3/45/46 (strapping; IO46 input-only).

### 4.6 Bandwidth

| | 2 faces | 3 faces | 6 faces |
|---|---|---|---|
| SPI in | 630 KB/s | 630 KB/s | 630 KB/s |
| DMA read, if PSRAM | 4.9 MB/s | 7.4 MB/s | 14.7 MB/s |
| Blit write, if PSRAM | 1.5 MB/s | 2.2 MB/s | 4.4 MB/s |
| **PSRAM total** | 6.4 MB/s | 9.6 MB/s | **19.2 MB/s** |

DMA read is at 100 Hz refresh. SPI traffic is identical for every option — the payload is
broadcast, not per-face.

Against octal PSRAM at 80 MHz DDR: 160 MB/s peak, and rather less sustained. [A] **This is the
figure most worth measuring early**, because it decides whether the six-face option is real. It is
also shared with instruction-cache fills, so it is not a private budget.

---

## 5. Rules

1. **Never** put the particle pool or the accumulator in PSRAM. Random access and per-frame full
   rewrites respectively.
2. The **panel table is never trimmed** per node. `Geometry::bounds()` derives the container from
   it and every node must agree on that container exactly; only the *render set* is per-node.
3. Float-using tasks are **pinned**. Lazy FPU context saving on Xtensa makes this a correctness
   issue, not a performance one.
4. The master's step index is the **authoritative clock**. It must not use `advance(wallDt)`, which
   drops backlog when saturated and makes step count a function of frame timing.
5. A display node **holds its last frame** on a dropped packet, and reports its last received step
   index so the master can flag it. A stale face is visually obvious and otherwise silent.
6. **`-ffp-contract=off` on every target.** Without it Xtensa fuses multiply-add differently to
   WASM and the trajectories diverge within a few hundred steps.

---

## 6. What has to be measured before the topology is settled

1. **Sustained octal PSRAM bandwidth** under concurrent CPU load. Decides whether six faces on one
   board is real.
2. **Row-walking blit cost**, internal and PSRAM. The per-pixel path is known unviable in PSRAM;
   the row writer does not exist yet.
3. **Achieved refresh** at 128×64 and 384×64, 6-bit. The 141 Hz measured on a 192×32 chain will
   not survive 4× the pixels.
4. **Panel power** [A] — 20 W/panel drives the pack, the converter and the runtime requirement.
5. **Whether the master's 117 KB spare survives ESP-NOW.** ~55 KB is an estimate, and it is the
   headroom the radio was justified against.
