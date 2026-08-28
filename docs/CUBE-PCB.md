# CUBE-PCB — hardware requirements

**Status:** requirements, not a validated design. Nothing here has been built. Every figure marked
**[M]** is measured from the firmware in this repo; every figure marked **[A]** is an assumption
that must be confirmed against a real panel datasheet or on the bench before layout is committed.

**Revision:** 0.3 — IMU returns to the geometric centre with the cells mounted off-centre, which
supersedes 0.2's offset-compensation approach (§6.1 for why the centre is provably optimal, not
merely convenient).

0.2 — single-board layout, 4S1P 18650 pack, 12 A display rail, 5-pin magnetic dock. Superseded
0.1's 3S1P 21700 pack and 8 A rail. Topology and PSRAM corrections live in
[RESOURCES.md](RESOURCES.md) §1–2.

---

## 1. What this describes

The electronics for an LED cube: six 64×64 HUB75 panels around a closed volume, running the
particle fluid simulation in this repository. It is a **handheld object** — tilting it pools the
liquid, shaking it splashes — so weight and battery are first-class constraints, not afterthoughts.

Four boards, two designs:

| board | qty | role |
|---|---|---|
| `CUBE-MASTER` | 1 | physics, IMU, radio, battery, charging, power distribution. Drives no panels. |
| `CUBE-DISPLAY` | 3 | two 64×64 faces each: receives particle state over SPI, splats, blits. |

Why four boards rather than one — **superseded, see [RESOURCES.md](RESOURCES.md) §1–2.** A 384×64
chain at 6-bit double-buffered needs 288 KB of DMA buffer against ~230 KB of usable internal SRAM,
so that configuration is impossible **[M]** — but moving the buffer to PSRAM removes the memory
constraint entirely, and one display node driving all six faces then fits in 188.4 KB internal.
**[M]** The remaining argument for splitting is blit cost (13.3 ms for six faces, 40 % of a 33 ms
frame) and PSRAM contention, not memory. Board count is therefore an open decision; RESOURCES.md §2
recommends a display board that can drive one to three panels per output so the layout does not
foreclose it.

Why distributed rather than one central PCB: HUB75 is 14 signals switching at 16 MHz and is
visually unforgiving, while the inter-board SPI is 4 signals and tolerant of a longer run. Keeping
the fragile bus short is worth more than keeping the robust one on-board. Display boards therefore
mount at their own panel pair, with ~10 cm ribbons instead of ~40 cm.

---

## 2. Confirmed decisions

| | |
|---|---|
| Module | ESP32-S3-WROOM-1**U** N16R8 (16 MB flash, 8 MB **octal** PSRAM), external antenna on master |
| Panels | 6 × 64×64 **P2**, 1/32 scan, HUB75**E** (the E address line is required) |
| Cube | ~128 mm faces, ~13 cm outside, ~1.3–1.6 kg all-in |
| Battery | **4S1P 18650**, ≥3.5 Ah — 50.4 Wh nominal, ~45 Wh usable, ~188 g of cells, removable holders |
| Runtime | **1.79 h** at brightness 96, auto-dimming below 20% charge |
| Charging | USB-C PD sink, 20 V, plus a **5-pin magnetic dock**. **Charge or run, not both** — see §5.3 |
| 5 V rail | **12 A per display board**, so no real scene is ever brightness-limited — see §4.5 |
| Distribution | Raw pack voltage (**12.0–16.8 V**) to the display boards; 5 V regulated **locally** |
| Board count | **Open** — 1, 2 or 3 display boards. See [RESOURCES.md](RESOURCES.md) §2 |
| Inter-board | SPI, master as host, 20 MHz, DMA, broadcast to all three at once |
| Cube-to-cube | ESP-NOW, **master only** (beaker mode chaining) |

### 2.1 Face pairing — not arbitrary

Each display board drives two faces as a HUB75 daisy chain (board → face A → face B), so the two
faces must be **physically adjacent** or the second ribbon crosses the cube.

A cube's face-adjacency graph is the octahedron graph: every face is adjacent to four others and
non-adjacent only to its opposite. So an all-adjacent perfect matching exists, and the obvious
opposite-face pairing is the one to avoid:

| board | faces (indices per `Geometry::cube`) | share an edge? |
|---|---|---|
| DISPLAY-A | `-Z` (0) + `-X` (2) | yes |
| DISPLAY-B | `+Z` (1) + `+Y` (5) | yes |
| DISPLAY-C | `+X` (3) + `-Y` (4) | yes |

Each board mounts near the shared edge of its pair. **A pairing of opposite faces would work
electrically and force a ribbon diagonally through the interior** — do not let the panel indices
in the firmware suggest otherwise.

---

## 3. Block diagram

```
                    USB-C  (PD sink 20V, + USB2 console/JTAG)
                      |
              +-------+---------------------------+
              |  STUSB4500  autonomous PD sink    |
              +-------+---------------------------+
                      | VBUS 20V
              +-------+--------+
              |  BQ25792       |  1-4S buck-boost charger, I2C
              |  charger       |
              +-------+--------+
                      |
        +-------------+--------------+
        |  4S1P 18650  3.5Ah 50.4Wh  |
        |  BQ76920 AFE (3-5S):        |
        |  balancing, coulomb count   |
        +-------------+--------------+
                      | VPACK 12.0 - 16.8 V
              +-------+--------+
              | load switch    |  opened during fast charge (§5.3)
              +-------+--------+
                      |
     +----------------+----------------+----------------+
     |                |                |                |
 CUBE-MASTER     CUBE-DISPLAY-A   CUBE-DISPLAY-B   CUBE-DISPLAY-C
 3V3 buck        5V buck 12A      5V buck 12A      5V buck 12A
 ESP32-S3        3V3 buck         3V3 buck         3V3 buck
 LSM6DSOX        ESP32-S3         ESP32-S3         ESP32-S3
 ext. antenna    |                |                |
     |           +-- HUB75 --> face 0 --> face 2
     |                            +-- HUB75 --> face 1 --> face 5
     |                                           +-- HUB75 --> face 3 --> face 4
     |
     +----- SPI (SCK/MOSI/MISO + CS x3) ---------> all three display boards
```

---

## 4. Power budget

### 4.1 Measured content duty cycle **[M]**

Datasheet "max power" assumes a white screen. This simulation is mostly dark, so the mean LED duty
cycle is what actually sets current. Measured from `Renderer::resolve` output across all six faces
after 600 settled steps, per scene:

| scene | mean duty | lit texels |
|---|---|---|
| kettle | **31.0 %** | 76.8 % |
| water tank | 23.9 % | 40.7 % |
| neon tank | 22.6 % | 38.5 % |
| water and sand | 19.3 % | 36.3 % |
| twin flames | 18.4 % | 74.4 % |
| sand pile | 12.5 % | 23.2 % |
| campfire | 13.1 % | 60.3 % |

**Kettle at 31 % is the design case.** Reproduce with `./build/platform/host/partsim_dutyreport` —
a straight mean of resolved RGB over every texel of every face, after 600 settled steps. Re-run it
after any renderer change, because REQ-PWR-1 is sized against it.

> Duty is essentially **resolution-independent**: the blob radius is a fixed world size
> (`kSplatRadiusWorld`), so the fluid occupies the same fraction of a face at 32×32 and 64×64.
> Confirmed by `renderer_peak_accumulation_is_pitch_invariant` — peak accumulation moved 0.99×
> across a pitch halving.

### 4.2 System power **[M]** from §4.1, **[A]** for the 20 W/panel figure

Assumes 20 W max per panel at full white and a 1.5 W/panel scan-logic baseline. **Both need
confirming against the panels actually purchased — panel ratings vary widely at the same pitch.**

| condition | panels | + electronics, ÷92 % buck | pack current @ 14.8 V |
|---|---|---|---|
| brightness 96, kettle | 21.9 W | **25.3 W** | 1.71 A |
| brightness 160, kettle | 30.6 W | 34.7 W | 2.35 A |
| brightness 255, kettle | 43.4 W | **48.6 W** | 3.28 A |
| **all white, brightness 255** | 120.0 W | **131.8 W** | **8.9 A** |

Electronics: 4 × ESP32-S3 at ~0.25 W plus radio duty ≈ 1.3 W total.

### 4.3 REQ-PWR: average-picture-level limiting is mandatory

The 5.2× spread between the design case and a white screen is too wide to size hardware for.

- **REQ-PWR-1** Firmware shall compute the average picture level per frame and reduce brightness so
  system draw never exceeds **60 W** (4.05 A pack). The accumulation buffers already hold everything
  needed — this is a sum over the resolve pass, not new measurement hardware.
- **REQ-PWR-2** Pack wiring, fusing and the load switch shall be rated **15 A** regardless, so a
  firmware fault or a solid-white diagnostic browns out rather than damaging anything.
- **REQ-PWR-3** Each display board's 5 V rail shall deliver **12 A continuous**, sized for a board
  driving all six panels. Derivation in §4.4. Deliberately generous: it is the point at which the
  brightness cap stops constraining anything you would actually want to display.
- **REQ-PWR-4** **Soft-start on the 5 V rail.** Six panels carry ≥6000 µF of local bulk
  (REQ-D-5); without a controlled ramp the converter current-limits and hiccups at power-on rather
  than starting.

### 4.4 Sizing the 5 V rail — why 12 A

5 V current for **one board driving all six panels**, by brightness and content duty. The
scan-logic baseline is 1.8 A and is independent of both; LEDs add up to 22.2 A at full white.

| brightness | kettle, 31 % | water tank, 24 % | all white |
|---|---|---|---|
| 64 | 3.53 A | 3.13 A | 7.37 A |
| 96 *(design case)* | **4.39 A** | 3.80 A | 10.16 A |
| 128 | 5.25 A | 4.46 A | 12.94 A |
| 160 | 6.11 A | 5.13 A | 15.73 A |
| 255 *(full)* | **8.68 A** | 7.10 A | **24.00 A** |

What each rail size actually buys:

| rail | all-white up to brightness | worst real scene up to brightness |
|---|---|---|
| 8 A | 71 | **230 — cannot reach full brightness** |
| 10 A | 94 | 255 |
| **12 A** | **117** | **255, with headroom** |
| 15 A | 152 | 255 |

**8 A is the wrong number, and that is the whole argument.** It cannot run the measured worst-case
scene at full brightness, so the APL limiter would be intervening on *real content* — constraining
what the object is allowed to look like rather than merely guarding against pathological input. At
12 A every real scene runs at brightness 255 untouched, and the limiter engages only above
brightness 117 on near-white synthetic content, which no scene produces.

Turning brightness down remains the right *operating* policy, and REQ-PWR-1 still enforces a 60 W
system cap. The 12 A rail is what makes that a choice rather than a constraint.

Thermal consequence: at 92 % efficiency a 12 A output dissipates **5.2 W** in the converter — 3.8 W
at the 8.68 A worst real scene, 1.9 W at the design case. That is a meaningful heat source inside a
sealed ~1 L box; see REQ-THM-1 and REQ-THM-3.

### 4.5 Why distribute at pack voltage

At 14.8 V the whole cube draws 1.7 A typical. At 5 V the same power is 5.1 A, and a single central
5 V rail for all six panels would have to carry 24 A at full white. Regulating locally on each
display board means **that current never exists anywhere in the design**, thins the inter-board
harness by 3× for the same power, and spreads converter heat across three boards instead of one.

---

## 5. Battery and charging

### 5.1 Pack

- **REQ-BAT-1** **4S1P**, 18650 Li-ion, ≥3.5 Ah, ≥6 A continuous discharge, in **removable
  holders**. 50.4 Wh nominal, 12.0–16.8 V.
- **REQ-BAT-2** Runtime at the design case shall be ≥1.75 h. Calculated: 45.4 Wh usable ÷ 25.3 W =
  **1.79 h**. Marginal — if the panels measure worse than the 20 W **[A]** assumption, this is the
  requirement that fails first. **Two cells is not a configuration:** 22.7 Wh gives 0.90 h. Build
  the holders for four and treat two as bring-up only.
- **REQ-BAT-3** **4S1P, and specifically not 2S2P**, because the holders are removable. A parallel
  pair accepts a cell at a different state of charge with an inrush limited only by cell internal
  resistance — tens of amps, enough to stress cells and weld holder contacts, and nothing in the
  design can prevent a user doing it. Series-only has no parallel path; series mismatch is what the
  AFE is for. 2S2P is the better choice only for a welded pack, where cells are matched once at
  assembly.
- **REQ-BAT-3a** Not 1S. A single cell would need a boost converter drawing 7 A at the design case
  and 36 A at a white screen; 4S bucks down instead. 14.8→5 V also halves the pack current versus
  2S for the same power (1.71 A against 3.42 A at the design case), which thins the harness.
- **REQ-BAT-4** Pack shall stay **under 100 Wh** so the object remains air-travel legal as carry-on
  without airline approval. 50.4 Wh has ample margin, and six cells (75.6 Wh, 2.69 h) would still
  fit under it if runtime matters more than 94 g.
- **REQ-BAT-5** Cell-level protection and balancing via a dedicated AFE (BQ76920 or equivalent,
  which covers 3–5S): over-voltage, under-voltage, over-current, short-circuit, cell balancing.
- **REQ-BAT-5a** Per-cell fusing, and a soft-start on the pack output. Removable holders mean the
  pack can be made and broken live.
- **REQ-BAT-6** Under-voltage lockout at 3.0 V/cell (12.0 V pack). Firmware shall step brightness down below
  20 % state of charge rather than cutting out abruptly.
- **REQ-BAT-7** State of charge by coulomb counting, not voltage. Li-ion's discharge curve is flat
  enough that voltage-only SoC is worthless across the middle 70 % of the pack.

### 5.2 Charger

- **REQ-CHG-1** USB-C PD sink, autonomous (STUSB4500 or equivalent) so it negotiates before any
  firmware runs and a bricked module can still be charged.
- **REQ-CHG-2** Request 20 V. Fall back gracefully to 15 V/9 V/5 V with a proportionally reduced
  charge current; **never** assume a 100 W adapter is present.
- **REQ-CHG-3** Charge at 1.75 A (0.5 C) to **16.8 V** CC/CV. ~29 W at the top of charge, inside a
  20 V/3 A (60 W) adapter. Full charge from empty ≈ **2.5 h** including the CV tail. Note 20 V in
  to 16.8 V out is a narrow buck ratio — a buck-boost charger (REQ-CHG-4) removes the concern and
  also lets 9 V and 15 V sources work.
- **REQ-CHG-4** Charger IC shall be multi-cell capable with I2C telemetry (BQ25792 or equivalent),
  so charge state is readable by firmware and reportable over the console.
- **REQ-CHG-5** Thermistor on the pack, wired to the charger's TS input. Charging shall be
  inhibited outside 0–45 °C. This is not optional for Li-ion.

### 5.3 Charge or run, not both — and what that costs

The chosen simplification is that the load sits on the pack side with **no power-path
multiplexing**. That deletes a power-path IC, a second inductor and a control loop.

The consequence is real and should be stated plainly: **the cube goes dark while fast-charging**,
which is exactly when an ambient object would most like to be lit.

- **REQ-CHG-6** A high-side load switch shall isolate the display rails during fast charge.
  Firmware opens it when charge current exceeds ~1 A.
- **REQ-CHG-7** Once charge current tapers below ~1 A (the CV tail, most of the back half of a
  charge), the load switch may close and the cube may run at a brightness capped to keep total
  input inside the negotiated PD budget. This recovers most of the "lit while docked" behaviour for
  the cost of one comparison in firmware, and is strongly recommended.
- **REQ-CHG-8** Simultaneous charge and discharge through the pack shall be avoided — it defeats
  the charger's termination detection and confuses coulomb counting.

### 5.4 The magnetic dock

A magnetic connector on the cube's edge carrying power and USB, so setting the cube down charges it
and connects the console with no plug to find.

- **REQ-DOCK-1** **Five pins, not four:** VBUS, GND, D+, D− and **one CC**. Four pins leaves no CC
  line, which means no PD negotiation and a hard 5 V ceiling. USB-C needs CC1 *and* CC2 only to
  detect cable flip — a magnetic connector cannot flip, so a single CC is sufficient for a sink.
- **REQ-DOCK-2** Voltage matters more than pin count, because magnetic contacts are typically rated
  2–3 A. Through 2 A per pin, against 45 Wh usable:

  | negotiated | power | charge time |
  |---|---|---|
  | 5 V | 10 W | 5.0 h |
  | 9 V | 18 W | 2.8 h |
  | 20 V | 40 W | **1.2 h** |

  If PD across the dock proves troublesome, the acceptable fallback is to split the roles: the
  USB-C receptacle does fast charging and development, and the dock is a 5 V overnight cradle.
- **REQ-DOCK-3** **The cube side shall be a pure sink** — high-impedance inputs presenting no
  voltage of its own. Exposed contacts on an object that gets set down on desks and metal must be
  inert to a coin or a key. The dock presents power; the cube never does.
- **REQ-DOCK-4** Reverse-polarity protection on the dock input regardless of magnet keying, which
  is a convention rather than a guarantee. ESD protection per REQ-SAF-5.
- **REQ-DOCK-5** USB across the dock is **full-speed only (12 Mbps)**, which is what makes pogo
  pins viable at all — the ESP32-S3's USB peripheral is full-speed, so there is no high-speed
  differential pair to get wrong. Route D+/D− as a pair with a ground return anyway.
- **REQ-DOCK-6** Data and power are simultaneous and independent — that is ordinary USB and needs
  no special provision. What *does* need provision: the master's 3.3 V rail shall stay up while the
  display rails are switched off for charging (REQ-CHG-6), so the console is alive exactly when the
  cube is docked.

---

## 6. CUBE-MASTER

- **REQ-M-1** ESP32-S3-WROOM-1**U** N16R8 with an **external antenna**. ESP-NOW for beaker-mode
  cube chaining lives only on this board; the U variant lets the antenna be placed away from the
  panels. Panel emissions degrading the radio is the surviving risk of putting a radio in this
  object at all, and it is why no display board has one.
- **REQ-M-2** LSM6DSOX 6-DOF IMU on I2C, configured ±8 g / ±500 dps at 208 Hz. **±8 g is not
  negotiable** — a hand-shaken cube clips a ±2 g part, and it clips exactly during the interaction
  the object exists for.
- **REQ-M-3** The IMU shall sit at the cube's **geometric centre**, on a satellite board reaching in
  from the main board, with the cells mounted off-centre to free that space. Rev 0.1 assumed the
  cells would occupy the centre and specified firmware compensation instead; putting the IMU there
  is both simpler and strictly more correct — see §6.1.
- **REQ-M-3a** The IMU bracket's **first resonant mode shall be above 200 Hz.** This is the real
  cost of a satellite board and it is easy to get wrong. The LSM6DSOX at 208 Hz ODR has roughly
  100 Hz of analogue bandwidth, and `MotionSource`'s shake path is a 5 Hz *high*-pass — it passes
  everything above 5 Hz. A bracket resonance anywhere below ~100 Hz is therefore injected straight
  into the container-acceleration signal as phantom shake, and the fluid sloshes in response to the
  bracket ringing rather than to the hand. Fix it mechanically: a short thick post or a triangulated
  bracket, not a long thin standoff. A firmware low-pass would also work, at the cost of shake
  latency.
- **REQ-M-3b** I²C to the satellite board shall be routed away from the HUB75 ribbons. 400 kHz is
  slow, but ribbon switching at 16 MHz is an effective aggressor.
- **REQ-M-4** SPI host: SCK, MOSI, MISO and **three individual CS lines**. Broadcast asserts all
  three CS at once, so one payload reaches all nodes; individual CS is for polling status on MISO.
- **REQ-M-5** Drives no panels. Its spare SRAM and its whole CPU budget are why the radio and the
  physics can coexist here — measured **112.6 KB of ~230 KB**, so ~117 KB spare. **[M]**
  (An earlier revision said 172.6 KB; the report was charging the master a DMA buffer and a staging
  buffer for a HUB75 chain it does not have.)
- **REQ-M-6** USB-C for PD, USB2 console and JTAG on one connector.
- **REQ-M-7** One button (mode/reset) and one RGB status LED, both firmware-defined.
- **REQ-M-8** 3.3 V buck (not LDO) from pack voltage. An LDO dropping 14.8→3.3 V at 150 mA wastes
  1.2 W as heat inside a sealed box.

### 6.1 Why the geometric centre, and not the centre of mass

The simulation applies **one** container acceleration to the whole fluid, so the question is which
single point best represents the volume. Across a rigid body the true acceleration field during
rotation is

```
a(r) = a_ref + ω̇ × r + ω × (ω × r)
```

Integrate that over a volume symmetric about the geometric centre and both correction terms vanish
exactly, because `∫ r dV = 0`. **The acceleration at the geometric centre is the exact volume
average**, which makes it the optimal sample point rather than merely a convenient one. Any other
position needs the compensation Rev 0.1 specified, and even then only recovers what this placement
gives for free.

Centre of mass is the wrong target: what the fluid experiences depends on where the *fluid* is, not
where the mass is. Sensor position also only matters during rotation — gravity is uniform, so
attitude estimation is unaffected either way.

Mounting the cells off-centre costs almost nothing. 228 g of cells and holders 38 mm off-centre in a
1.55 kg object shifts the centre of mass by **5.6 mm**, 8.7 % of the half-width. The geometric
centre then swings about that offset CoM during rotation — 0.06 g at 10 rad/s, 0.23 g at a violent
20 rad/s, against 1–3 g of linear shake. And an IMU *at* the geometric centre measures that
correctly, because it is real motion of the fluid volume, not an artefact.

What off-centre mass does cost: a slightly non-diagonal inertia tensor, so the cube will not spin
perfectly cleanly about its face axes, and impact loading on cantilevered holders. Neither is
expected to matter for a hand-held object, but both are untested.

---

## 7. CUBE-DISPLAY (×3, identical)

- **REQ-D-1** ESP32-S3-WROOM-1 N16R8. **No antenna requirement** — these boards never transmit.
- **REQ-D-2** Two HUB75E connectors, each able to drive a chain of **one to three** panels, so the
  same board serves a 1-, 2- or 3-board cube and the topology is chosen at bring-up rather than at
  layout. Both connectors share all 14 HUB75 signals — panels chain, they do not each need a port.
- **REQ-D-3** 5 V synchronous buck, input **12.0–16.8 V** (4S pack), **≥12 A continuous**, with
  soft-start. Sized for a board driving all six panels; §4.4 has why 8 A is not enough, REQ-PWR-4
  the soft-start. Requires the APL cap enforced from boot (REQ-PWR-1), including during the
  bring-up test pattern.
- **REQ-D-4** 3.3 V buck for the module.
- **REQ-D-5** ≥1000 µF of local bulk capacitance on the 5 V rail per panel, close to the
  connectors. Panel inrush at power-on is substantial and the harness has inductance.
- **REQ-D-6** SPI device: SCK, MOSI, MISO, CS.
- **REQ-D-7** **Two ID strapping pins**, jumper- or resistor-set, giving four codes. All three
  boards are electrically identical and one firmware image serves them; which faces a board drives
  is a fact about the wiring, read at boot. This is the same principle as the mount table: physical
  facts belong in hardware straps and NVS, not in per-board firmware variants.
- **REQ-D-8** Series termination resistors, **22–33 Ω, at the source**, on CLK, LAT, OE and all six
  RGB lines. Footprint a **74AHCT245** buffer per connector even if ribbons are short — retrofitting
  one into a finished cube is miserable, and an unpopulated footprint costs nothing.
- **REQ-D-9** Programming header (UART TX/RX, EN, IO0, 3V3, GND) for initial bring-up and unbrick.
  Routine updates should go over the SPI bus from the master once that is implemented.

---

## 8. Interconnect

### 8.1 SPI bus

- **REQ-SPI-1** 20 MHz, master as host, DMA-backed so the transfer overlaps the next physics step.
- **REQ-SPI-2** Payload ≈ **21 KB/frame worst case, 630 KB/s at 30 fps**: 8 B header, 1280 × 6 B
  quantised int16 positions, 1280 × 4 B velocity and material, plus an RLE heat field. **[M]**
- **REQ-SPI-3** Series resistors on SCK and MOSI at the master. Keep stubs short; route SCK and
  MOSI as a matched pair with a ground return in the harness.
- **REQ-SPI-4** Fall back to 10 MHz if signal integrity disappoints — still 2× the required
  bandwidth. Design the harness so this is a firmware constant, not a rework.
- **REQ-SPI-5** A dropped frame must be survivable: a display node holds its last frame. Each node
  shall report its last received step index so the master can flag a node falling behind. A stale
  face is visually obvious and otherwise silent in logs.

### 8.2 Power harness

- **REQ-H-1** Pack voltage to each display board on ≥20 AWG (4.05 A at the 60 W cap), fused or eFused per board.
- **REQ-H-2** **Never** power a panel through another panel's pigtail. Each panel gets 5 V
  injection direct from its display board.
- **REQ-H-3** Star ground at the master. Panel return currents are large and switch at 16 MHz;
  daisy-chained grounds will put that on the SPI reference.

---

## 9. Pin allocation

Both boards must respect the N16R8 exclusion list. **IO33–37 are consumed by the octal PSRAM** —
using them appears to work until the first PSRAM access, then corrupts memory. Also unavailable:
IO26–32 (module SPI flash), IO19/20 (native USB), IO43/44 (UART0), IO0/3/45/46 (strapping, and
IO46 is input-only).

That leaves IO1–18, IO21, IO38–42, IO47, IO48 — 26 pins, against the 20 a display board needs.

> The comment in `platform/esp32/src/Pins.h` currently names only IO35/36/37 for PSRAM. That is the
> subset a WROOM-1 exposes and happens to be safe today, but **IO33–37** is the correct range and
> the comment should be widened before anyone picks a pin from it.

### CUBE-DISPLAY — 20 pins

| function | pins |
|---|---|
| HUB75 RGB | R1=4 G1=5 B1=6 R2=7 G2=15 B2=16 |
| HUB75 address | A=17 B=18 C=8 D=9 **E=21** |
| HUB75 control | LAT=10 OE=11 CLK=12 |
| SPI device | SCK=13 MOSI=14 MISO=47 CS=48 |
| board ID straps | 38, 39 |

**E=21 is a change from the current firmware**, which has `kE = -1` for 1/16-scan 32×32 panels.
64-row panels are 1/32 scan; leaving E unwired shows the top half of each panel duplicated.

### CUBE-MASTER — ~12 pins

| function | pins |
|---|---|
| SPI host | SCK=12 MOSI=11 MISO=13 CS0=14 CS1=15 CS2=16 |
| I2C (IMU, charger, AFE) | SDA=8 SCL=9 |
| IMU interrupt | 10 |
| Charger / PD alert | 17, 18 |
| Button, status LED | 4, 5 |
| USB | 19, 20 (native) |

---

## 10. Mechanical and thermal

- **REQ-MEC-1** Outside ~128 mm per face. With ~15 mm of panel and frame depth the usable interior
  is roughly a **98 mm cube (~0.94 L)**. Four 18650 cells are 65 mm long and sit comfortably in a
  row — one reason 18650 beats 21700 here despite near-identical energy density. Verify the holder
  footprint against the real frame: four holders plus a 12 A buck plus two HUB75 headers puts the
  board near 80 x 80 mm, most of one internal face.
- **REQ-MEC-1a** Cells shall be mounted **off-centre**, leaving the geometric centre free for the
  IMU satellite board (REQ-M-3). The resulting 5.6 mm shift in centre of mass is acceptable; see
  §6.1.
- **REQ-MEC-2** Target all-in mass **≤1.6 kg**: ~780 g panels, ~188 g cells + ~40 g holders, ~300 g frame and
  boards. Above roughly 2 kg the object stops being shakeable and the IMU becomes a tilt sensor,
  which undercuts the primary interaction.
- **REQ-MEC-3** Display boards mount at the shared edge of their face pair (§2.1).
- **REQ-THM-1** **Ventilation is required and is an open problem.** Panels dissipate a large share
  of their heat from the driver ICs on their *inward* face, so roughly 8–10 W lands inside a sealed
  ~1 L box at the design case. Perforated corners or edge vents are preferred to a fan; a fan adds
  noise and a moving part to a handheld object.
- **REQ-THM-2** Thermal validation shall run the **kettle** scene — the measured worst case at
  31 % duty — at maximum permitted brightness for one hour, logging pack, charger and panel
  temperatures. Not the campfire, which is only 13 %.
- **REQ-THM-3** Cells shall be thermally separated from the panel backs and from the 5 V bucks.

---

## 11. Protection and safety

- **REQ-SAF-1** Pack fuse sized per REQ-PWR-2 (15 A), plus per-board fusing or eFuse.
- **REQ-SAF-2** Reverse-polarity protection on the pack connector.
- **REQ-SAF-3** Charge inhibit outside 0–45 °C (REQ-CHG-5).
- **REQ-SAF-4** Over-temperature shutdown independent of the main firmware — a hung `simTask` must
  not be able to hold the panels at full brightness indefinitely.
- **REQ-SAF-5** ESD protection on USB-C and on any externally exposed contact.
- **REQ-SAF-6** No exposed conductor may reach pack potential. If a charging dock is ever added,
  its pads need reverse-polarity and short-circuit protection.

---

## 12. Bring-up requirements

The PCB must make the following possible **without reflashing**, because each is a fact about the
built object that cannot be known before it exists:

- **REQ-BU-1** Per-face mount rotation and mirror, adjustable over serial (`m` command) against the
  orientation test pattern (`t`). Already implemented; `ChainMap` guarantees the mapping stays a
  bijection so a typo cannot silently blank a face.
- **REQ-BU-2** The calibrated mount table and the IMU axis map shall persist in **NVS**. They are
  currently in-memory only, which means recalibrating after every reboot.
- **REQ-BU-3** Brightness, and the APL cap, adjustable over serial.
- **REQ-BU-4** Pack voltage, current, SoC, temperature and charge state readable over serial.
- **REQ-BU-5** Buy **one** panel first. 64×64 panels at the same pitch ship with different scan
  rates and driver ICs, and many need an FM6126A/ICN2038S init sequence. Bring one up on a display
  board, then buy the remaining five from the same batch.

---

## 13. Candidate parts

Not a BOM — starting points, all to be confirmed for availability and second sources.

| function | candidate | note |
|---|---|---|
| MCU | ESP32-S3-WROOM-1U-N16R8 | U = external antenna, master only |
| IMU | LSM6DSOX | ±8 g / ±500 dps at 208 Hz |
| PD sink | STUSB4500 | autonomous, NVM-configured, works with no firmware |
| Charger | BQ25792 | 1–4S buck-boost, I2C, integrated ADC |
| Pack AFE | BQ76920 | 3–5S protection, balancing, current sense |
| 5 V buck | ≥12 A synchronous, 20 V input, soft-start | one per display board; §4.4 |
| Magnetic dock | 5-pin, ≥2 A/pin | VBUS, GND, D+, D−, **one CC** — §5.4 |
| 3 V3 buck | 500 mA synchronous | per board; not an LDO |
| HUB75 buffer | 74AHCT245 | footprint per connector, populate if needed |
| Load switch | high-side eFuse with enable | REQ-CHG-6 |

---

## 14. Open items

Ordered by how much of the design they could invalidate.

1. **Panel power rating [A].** The 20 W/panel figure drives the pack size, the converter rating and
   the runtime requirement. Measure a real panel at full white and at the kettle scene before
   committing to cells. **If panels measure 30 W, REQ-BAT-2 fails and the pack must grow.**
2. **Scan-logic baseline [A].** Assumed 1.5 W/panel. At the design case this is 9 W of 21.9 W — 41 %
   of panel power — so it matters more than its size suggests. Measure with all LEDs off.
3. **Thermal.** REQ-THM-1 is a genuine unknown; ~10 W inside 1 L may need more than passive vents.
4. **Sustained octal PSRAM bandwidth** under concurrent CPU load. Decides whether one display
   board can drive all six faces. Requires the row-walking blit first — the per-pixel path does
   147 456 scattered PSRAM accesses per frame at six faces and cannot work. RESOURCES.md §2.
5. **ESP-NOW versus panel EMF.** Putting a radio in this object was a reopened decision. The
   master drives no panels so there is no DMA timing to disturb, but "a radio on a non-display node
   cannot disturb a display node" is reasonable and **unverified**. Measure display refresh with
   the radio transmitting.
6. **Refresh rate at 128×64.** `S3_LCD_DIV_NUM=10` with `HZ_16M` measured 141 Hz on a 192×32 chain
   (6144 px). A 128×64 chain is 8192 px, so expect ~106 Hz and possibly a bump to `HZ_20M`.
7. **Blit cost.** ~8192 `drawPixelRGB888` calls per node per frame at ~130 cycles each ≈ 4.4 ms,
   about 13 % of a 30 fps frame. Affordable; `ChainMap::row` exists so a direct DMA-buffer row
   write can replace it without disturbing the mapping.
8. **Cell format.** 18650 at 65 mm fits the 98 mm interior with margin. Confirm holder footprint
   against the real frame, and that a single 80x80 mm board carrying four holders, a 12 A buck and
   two HUB75 headers still leaves room for the panels' own connectors.
