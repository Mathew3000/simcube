# SPI link bring-up: session brief

**You are picking this up cold.** Everything needed is here or linked from here.

**Other agents are working in the same repository at the same time**, on Milestone 4 in `core/`.
Your work is confined to `platform/esp32/` and the boundary is clean — see §7.

---

## 1. Why this matters more than its size

`platform/esp32/src/SpiFrameLink.cpp` is **141 lines that have never executed.** It compiles,
links, fits the budget, and has never moved a byte between two boards. With
`core/src/SimFrame.cpp` behind it that is ~385 lines of never-run code, and it carries the entire
multi-node architecture:

- the `cube` build is one board driving six 32x32 faces, which works;
- **64x64 does not fit one board at all** — 288 KB of DMA against 320 KB of DRAM — so the six-face
  cube is one master computing physics and three display nodes drawing, connected by this link;
- `docs/CUBE-PCB.md` specifies four boards, connectors, a shared SPI bus and series resistors on
  the strength of it.

A PCB is about to be laid out around a link nobody has seen work. **That is the risk this closes.**
If the transport is wrong, better to know before copper.

---

## 2. What this project is

`partsim` — a particle fluid simulation for an LED cube. One C++17 core compiles to host tests,
WASM and ESP32 firmware, all running bit-identical physics.

| document | what it gives you |
|---|---|
| [`SIMULATION.md`](SIMULATION.md) §10 | multi-node: the seam, the wire format, render sets |
| [`RESOURCES.md`](RESOURCES.md) | per-role budget; why the split exists |
| [`DECISIONS.md`](DECISIONS.md) §9 | the seven reversals. **Read these.** |
| [`CUBE-PCB.md`](CUBE-PCB.md) §8 | the interconnect this validates |

House rule: **this project has been wrong seven times by asserting a number instead of measuring
one.** A negative result, honestly reported, is worth more than a confident one. If the link does
not work, saying so clearly *is* the deliverable.

---

## 3. The wiring — five jumpers, and the user may have already done it

Between two devkits:

| signal | master | display |
|---|---|---|
| SCK | IO13 | IO13 |
| MOSI | IO14 | IO14 |
| MISO | IO47 | IO47 |
| CS | IO4 (`kSpiCsOut[0]`) | IO48 (`kSpiCsIn`) |
| **GND** | GND | GND |

**GND is not optional** and is the classic omission — two separately-USB-powered boards have no
common reference without it, and the bus will look alive and decode garbage.

Role comes from **strap pins**, not the build: `kRoleA` = IO38, `kRoleB` = IO39, both pulled up,
jumper to ground to pull low.

| straps (B,A) | role |
|---|---|
| unstrapped `0b11` | **Master** — physics, IMU, no panels |
| `0b00` | Display0 — faces 0 and 2 |
| `0b01` | Display1 — faces 1 and 5 |
| `0b10` | Display2 — faces 3 and 4 |

So: flash `master` to one board and leave its straps open; flash `display` to the other and ground
IO38 **and** IO39 for Display0. Note a `display` build defaults to Display0 when unstrapped, so a
missing strap jumper looks like success — set them deliberately.

Confirm with the user that the wiring exists before assuming it.

---

## 4. The task

**Make one frame travel from a master to a display node and be decoded.** In order:

1. **Does anything arrive at all?** The smallest possible check first — bytes on MISO/MOSI, a
   `pollDirect` returning non-zero. Do not start by debugging the protocol.
2. **Does a frame decode?** `decodeFrame` validates magic, version, a geometry hash and a
   Fletcher-16 before touching state, so a failure tells you *which* of those. The geometry hash
   rejecting a mismatched build is a real and likely outcome — both ends must agree on the panel
   table exactly.
3. **Does the display node's step index track the master's?** That is the staleness signal the
   whole architecture is designed around: with one authoritative simulation, divergence is
   impossible and **staleness is the only failure mode**.
4. **Does it survive?** Run it for minutes, not seconds. Report the dropped-frame rate.

Broadcast asserts all three chip selects at once — every node needs every particle, since a
particle can light any face. With one display node wired, only `kSpiCsOut[0]` matters, but do not
"fix" the broadcast to suit a two-board bench.

### Numbers to check against

| | expected |
|---|---|
| frame size | ~21 KB worst case, 10 B/particle + RLE'd heat |
| rate | 30 fps → ~630 KB/s |
| clock | 20 MHz configured; **10 MHz is a legitimate fallback** and still 2x the requirement |

If it only works at 10 MHz or below, that is a finding for `CUBE-PCB.md` §8.1 and REQ-SPI-6 (series
resistors), not a defeat.

---

## 5. State of the tree

On `main`, clean, `ctest` 7/7. Golden hashes, which **must not move** — this is a transport
bring-up, not a physics change:

```
scripts/golden_hash.txt        f0021217 8e143d3b
scripts/golden_hash_esp32.txt  f0021217 8e143d3b
```

The firmware was restructured recently: application logic is `partsim::app::App` in `platform/app`
behind `Console`/`Clock`/`Display`/`MotionSensor`/`FrameLink`, and `main.cpp` is drivers and tasks.
`FrameLink` is the interface you are implementing against; `NullFrameLink` is what a board without
a transport gets.

---

## 6. Verification

```bash
ctest --test-dir build --output-on-failure
cd platform/esp32
for e in cube cube-fast panel lite qemu beaker master display; do
  ~/.platformio/penv/bin/pio run -e $e || echo "FAILED $e"
done
~/.platformio/penv/bin/pio run -e master  -t upload --upload-port <port A>
~/.platformio/penv/bin/pio run -e display -t upload --upload-port <port B>
```

`scripts/console.py` drives the console non-interactively — it reads until the port goes quiet,
because the interesting commands block for tens of seconds without printing a terminator. `r`
reports frame timing, and on a display node the last accepted step index.

`PARTSIM_PORT` overrides the port it picks; with two boards you will need it.

---

## 7. Working alongside the other agents

| yours | not yours |
|---|---|
| `platform/esp32/src/SpiFrameLink.{h,cpp}` | anything in `core/` |
| `platform/esp32/src/main.cpp` — task wiring | `core/src/SimFrame.cpp` |
| `platform/esp32/src/Pins.h` | `platform/app/` |

If you conclude the **wire format** is wrong, that is a finding to report, not a change to make —
`SimFrame.cpp` is shared with the browser preview and the host tests, and another agent is in
`core/`.

1. **Commit only your own paths.** `git add <paths>`, never `git add -A`.
2. **`git status` before every commit.** Files you did not touch are not yours.
3. Append `DECISIONS.md` entries at the end of the numbered run and expect to rebase.

---

## 8. Traps

1. **Flashing socket.** A DevKitC-1 has two USB-C ports. **COM** is a CH343 bridge with hardware
   auto-reset and always works; **USB** is native and only works if the running firmware
   cooperates — esptool reports "No serial data received" while the board is perfectly alive.
2. **An upload can leave a board in ROM download mode** (`waiting for download`). Recover by pulsing
   RTS with DTR held high.
3. **MISO contention.** Three SPI devices with CS low all drive MISO — hence REQ-SPI-6's 100-330 Ω
   series resistors. With one display node this cannot bite; with three on a bench it will.
4. **A `display` build defaults to Display0 when unstrapped.** A missing strap jumper therefore
   looks like success.
5. **A new PlatformIO environment fails with `HTTPClientError`**; seed `.pio/libdeps/<new>` from a
   working env, then delete `core.pio-link`, `app.pio-link` and `integrity.dat` from it —
   `symlink://` records an absolute path and you will otherwise compile another checkout's `core/`
   (`DECISIONS.md` D47).
6. **`core/` will be mid-edit under you, and your firmware build compiles it.** Another agent is
   working in `core/` throughout, so `pio run` can fail on a file you never touched — a declaration
   saved before its definition, a half-renamed symbol. Do not debug it and do not fix it; it is
   someone else's file in flight. Either wait a minute and retry, or build in isolation:

   ```bash
   git worktree add /tmp/spi HEAD          # HEAD only: no uncommitted work from anyone
   cp <your modified files> /tmp/spi/<same paths>
   mkdir -p /tmp/spi/platform/esp32/.pio/libdeps
   cp -R platform/esp32/.pio/libdeps/cube /tmp/spi/platform/esp32/.pio/libdeps/cube
   rm -f /tmp/spi/platform/esp32/.pio/libdeps/cube/{core.pio-link,app.pio-link,integrity.dat}
   cd /tmp/spi/platform/esp32 && pio run -e master
   ```

   The `rm` is trap 5 and is not optional — without it the worktree compiles the *main* checkout's
   `core/`, which is the thing you were trying to escape. Clean up with
   `git worktree remove /tmp/spi --force`.

7. **No panels are attached.** A display node blits into a DMA buffer that clocks out into nothing,
   which is fine and still measures correctly. You cannot *see* a frame arrive; you have to report
   it.

---

## 9. Definition of done

Either:

- **It works.** A frame travels, decodes, and the display node's step index tracks the master's over
  minutes. Report the achieved clock, the frame rate, the drop rate, and update `CUBE-PCB.md` §8.1
  with what the bus actually did.

or:

- **It does not, and you have said exactly where it stops** — bytes, framing, checksum, geometry
  hash, timing — with the measurement. That is a genuinely valuable outcome, because the PCB has
  not been laid out yet.

Either way: both goldens unchanged, `ctest` 7/7, all 8 environments build, and a `DECISIONS.md`
entry for what was learned about the transport.

### Out of scope

Milestone 4, the solver, the renderer, and a third display node — two boards prove the link; the
third proves the fan-out, and that is a later question.
