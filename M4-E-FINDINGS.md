# M4-E findings — beakers in the browser

What was built, what was measured, and the five things the brief asserted that turned out to be
wrong. Everything below is a number this session produced; where a claim is inferred rather than
measured it says so.

---

## 1. What landed

| | |
|---|---|
| `platform/wasm/bindings.cpp` | `ps_beaker_*`, a third set of state alongside `g_sim` and `g_nodes`. `JsSpillTransport` implements `SpillTransport` with JS on the other side of it. |
| `platform/wasm/web/beakers.html` | N beakers, chained, with fault injection and the counters on screen. |
| `platform/wasm/web/beakerview.js` | N cubes in one three.js scene. `cubeview.js` untouched. |
| `scripts/build_wasm.sh --beaker` | the second artifact, `public/partsim_beaker.{mjs,wasm}`, its own build directory. |
| `scripts/check_wasm.sh` | staleness guard extended to watch **both** artifacts. |

`ctest` 7/7. Both goldens `f0021217 8e143d3b`, unmoved. The default artifact's determinism check
still passes. Nothing in `core/` was changed — §4 below is a list of things that probably should be,
reported rather than patched, per §3 of the brief.

---

## 2. Verification, as asked for in §6

Three beakers, 1270 particles each, a deterministic tilt sweep, run headlessly against the shipped
beaker artifact through the same calls the page makes. Phases run back to back on the same beakers,
so each starts where the last ended.

| phase | frames | total fill | drift | dye drift | unsent | shortfall | madeUp | bad | rejected |
|---|---|---|---|---|---|---|---|---|---|
| perfect ring | 600 | 3810 → 3810 | **0.0%** | −0.9% | **0** | 0 | 0 | 0 | 0 |
| 30% packet loss | 600 | 3810 → 3800 | −0.3% | +1.3% | **0** | 1394 | **1394** | 0 | 0 |
| clean link again | 600 | 3800 → 3801 | 0.0% | +0.4% | **0** | +10 | +10 | 0 | 0 |
| 6-frame delay | 400 | 3801 → 3809 | +0.2% | +0.5% | **0** | +0 | +0 | 0 | 0 |
| 10% corrupted byte | 400 | 3809 → 3806 | −0.1% | 0.0% | **0** | +10 | +10 | 5 | 0 |
| **ring broken** | 600 | 3806 → 2002 | **−47.4%** | −47.3% | **0** | +0 | +0 | 0 | 0 |

- **Ring conserves.** Residual drift is the packets in flight at the instant of measurement, not a
  leak: in a run instrumented for it, `particlesOut − particlesIn` was 4441 − 4432 = 9, and the fill
  deficit was 3810 − 3801 = 9. Exactly.
- **A single beaker strictly decreases** and never recovers: −47.4% over 600 frames, monotone, and
  its dye falls with it (−47.3%) because dye leaves with the fluid.
- **Shortfall is made up one for one**, 1394 and 1394, and the ring did not drain.
- **`unsent` is zero in every phase**, because `ps_beaker_step` calls `advance()` — which clears the
  spill queue once per *frame* and accumulates across every substep — and pumps once immediately
  after. Trap 5 avoided by construction rather than by discipline.
- **Dye is conserved to within ±1.3%** while the ring is closed, and the drift does not accumulate
  in either direction across 2600 frames (−0.9, +1.3, +0.4, +0.5, 0.0), i.e. it plateaus.
  Comfortably inside the ~5% A measured.
- **Fill is 60% of capacity** — see §4.2, which is not what the stated rule produces here.

Fault injection uses an unseeded `Math.random()`, so the loss counts move between runs (5 of 15
corrupted packets were caught here; an earlier run caught 3 of 12). The invariants do not: across
every run, `shortfall == madeUp`, `unsent == 0`, and the closed ring held to within a few particles.
One of those earlier corruption runs is what exposed §5.2, and it is reproducible on demand rather
than by luck — see the one-byte repro there.

Module cost per displayed frame (advance + pump + render, all beakers, steady pour):

| | res 32 | res 64 |
|---|---|---|
| 1 beaker | 3.21 ms | 3.84 ms |
| 3 beakers | 9.29 ms | 11.23 ms |
| 4 beakers | 12.18 ms | 14.87 ms |
| 6 beakers | — | 32.36 ms |

---

## 3. The two open questions in `M4-PLAN.md` §6

Settled on renders from `platform/host/spill_ppm` built at the beaker tier — the host net rather
than the browser, because it has no bloom pass and no superimposed far faces between the answer and
the pixels.

**Does the split kernel read as colour in the fluid, or as colour floating on it?**
**In the fluid.** Mid-pour, the receiving cube shows a magenta body with a distinct red layer
*above* it — the arriving dye sitting on top of the blue it has not mixed into yet — and a
continuous violet gradient between the two that follows the shape of the liquid, including down the
bottom face. The colour is where the fluid is, and it moves with it. Nothing reads as a decal.

**Does ~10 distinguishable colour regions at `cube`'s spacing read as mixing?**
**Yes.** Rendered the identical fixture twice, chroma on, at d=2.5 (the beaker tier, ~13 regions)
and at d=3.0 (`cube`, ~10 regions), and cropped the receiving cube's four side faces from each. The
red-over-magenta stratification, the gradient, and the converging body are all present at d=3.0. It
is softer — the interface between the arriving red and the mixed body is blurrier, and the falling
stream is visibly beadier, individual blobs rather than a continuous column — but it is a
difference of sharpness, not of legibility. **The coarser spacing survives the colour question**, so
that lever is available: this is a vote for 26 fps, not 8.5.

Caveat worth stating: this is one fixture (red into blue, a steady 115° pour). A mix of two dyes
that are closer together in hue would be a harder test and was not run.

---

## 4. What the brief asserted that measurement contradicted

### 4.1 `scripts/build_wasm.sh -DPARTSIM_ENABLE_CHROMA=1` does nothing at all

§4 of the brief gives this as the command for the second artifact. It silently produces a
**chroma-free** build.

`core/CMakeLists.txt` forwards exactly six cache variables to the compiler — the `PARTSIM_MAX_*`
capacities and `PARTSIM_INTERNAL_PIXELS`. `PARTSIM_ENABLE_CHROMA` is not among them, so that `-D`
sets a CMake cache entry nothing reads, and `Config.h` falls through to its default of `0`. The tier
is the only thing that sets it, and a tier arrives as a *compiler* flag
(`-DCMAKE_CXX_FLAGS=-DPARTSIM_TIER_BEAKER=1`), which is how `build-beaker/` in the tree was
configured.

The failure is silent and it looks exactly like the bug the page exists to find: a ring of
identical blue cubes reads as dye that will not diffuse. So `beakers.html` now calls
`ps_beaker_has_chroma()` and refuses to run against a module without it, naming the right command.
`--beaker` is a flag on the script rather than a `-D` the caller supplies, for the same reason.

### 4.2 The 60%-fill rule does not produce 60% anywhere except on the device

D63 states the rule as `(kMaxParticles * 3) / 5`, and `App.cpp` implements it that way. That is 60%
of the **particle pool**. On the device the pool is 512 and the volume's capacity is larger, so the
pool binds and a beaker comes up at 307 — 60%, as intended.

In a host or WASM build the pool is 16384, so the rule asks for 9830, and `Simulation::init` clamps
it to `capacity * 9 / 10`. Measured: capacity 2117 at the beaker tier, fill **1905 — 90% of
capacity**. That is precisely the condition D63 exists to prevent, reached by following D63.

The bindings therefore take the fraction of whichever is binding,
`min(pool * 3/5, capacity * 3/5)`, which gives 1270 (60% of capacity) here and leaves the device
unchanged at 307. **This is worth pushing back into `App.cpp`**: the firmware is correct today only
because of a coincidence between two unrelated numbers, and the `beaker` tier at a larger pool would
inherit the bug.

### 4.3 Six beakers fit. So do seven.

§7 says "Four beakers fits; six does not." Measured by building the module at successive counts and
loading each:

| beakers | result |
|---|---|
| 4, 5, 6, 7 | links, instantiates, `ps_beaker_init(n, 64)` succeeds, runs |
| 8 | `wasm-ld: error: initial memory too small, 34740704 bytes needed` |

Against `INITIAL_MEMORY` of 33554432, so a beaker costs 3.41 MB and the hard ceiling is **seven**.
The brief's arithmetic charged three display nodes at a `Simulation` each; a `DisplayNode` carries
`RenderState`'s draw-only containers, not a solver, which is where the missing room was.

`kMaxBeakers` is now **6** — one beaker of slack under a ceiling whose failure mode is a *link*
error rather than a runtime one, so there is nothing to degrade gracefully into. A 6-beaker ring at
res 64 conserves (7620 → 7611, `unsent` 0) at 32.4 ms/frame of physics. The page still defaults to
3, which is the size the verification in §2 is written against.

### 4.4 `nodes.html` cannot run at all, and has not been able to since it was written

Not mine to fix (§3 lists it), so reporting only.

`nodes.html` imports `./cubeview.js`, which does `import * as THREE from 'three'`. A bare specifier
needs an import map, and `nodes.html` has none — `index.html` and `orient.html` both carry
`<script type="importmap">{ "imports": { "three": "./vendor/three.module.js" } }</script>`.
`panels.html` has none either but does not touch three.js, so it is fine.

Without the map the module graph fails to resolve and **the entire module script never executes**.
The static HTML panel still renders, so the page looks like it loaded and did nothing — which is
what happened to `beakers.html` on its first run and cost about twenty minutes. The fix is three
lines copied from `index.html`.

### 4.5 `rejected` does not fire when a beaker is overfull

D63 treats `injectSpill` returning false as the evidence that a chain is losing volume to a full
vessel. It is not, on a host or WASM build. `injectSpill` fails only when the **particle pool**
is full — 16384 here — and `capacity` (2117) is the solver's rest-density figure, not a hard cap.

Measured in the broadcast/unaddressed run of §6, with beakers stuffed to 3540 particles each, 167%
of capacity: `rejected` **0**. What actually fires is `SpillQueue::dropped`, 6796 packets' worth,
because more than `kMaxSpill` crossed in a single step. On the device the two coincide, since the
pool (512) is smaller than the capacity; off the device they do not, and `rejected` is silent
through the whole failure.

That is why the page's fill readout turns amber above 90% of capacity rather than waiting for a
rejection: at host capacities nothing on the chain says a beaker is overfull.

---

## 5. Two defects in the wire format

Both are in `core/` and neither was touched. They are reported because the browser is where they
became visible, and because the second one is a *permanent* fault from a single transient event.

### 5.1 Fletcher-16 is computed mod 255, so 0x00 and 0xFF are congruent

`fletcher16` in `core/src/SpillFrame.cpp` (and the identical one in `SimFrame.cpp`) is the textbook
mod-255 form. Under mod 255, `0x00 ≡ 0xFF`. Substituting one for the other **anywhere in the header
is invisible to the checksum**.

Exhaustive single-byte test against the real `encodeSpill`/`decodeSpill`, every offset, every
substitution:

```
off  orig  ->0xFF  ->0x00   xor0xFF
  0  0x50  reject  reject  reject  magic
  2  0x01  reject  reject  reject  version
  3  0x00  ACCEPT   same   ACCEPT  from        <-- zero in normal traffic
  6  0x00  ACCEPT   same   ACCEPT  seq         <-- zero in normal traffic
  7  0x00  ACCEPT   same   ACCEPT  seq         <-- zero in normal traffic
 10  0x00  ACCEPT   same   ACCEPT  totalOut    <-- zero in normal traffic
 11  0x00  ACCEPT   same   ACCEPT  totalOut    <-- zero in normal traffic
 13  0x00  reject   same   reject  count       <-- caught by a RANGE check, not the checksum
 16  0x00  ACCEPT   same   ACCEPT  payload
```

10 of 36 header corruptions got through, and **every one of them is a byte that is zero in ordinary
traffic** — the high bytes of `seq` and `totalOut`, and `from` on cube 0. The one zero header byte
that is caught, `count`'s high byte, is caught by `h.count > kSpillMaxPerPacket`, not by the
checksum. This is not a rare coincidence; it is the most likely corruption the format can suffer.

### 5.2 `SpillReceiver::note` has no plausibility bound, so one bad byte is a permanent fault

`note()` computes `missing = h.totalOut - seen_` in `uint32_t` and adds it to `shortfall_`. The pump
then pays that debt off at `kSpillMaxPerPacket` particles per frame, forever.

Minimal repro: two beakers, a clean link, **one byte flipped once** at offset 10 (`totalOut` byte 2,
which is `0x00` in any real chain), 900 frames.

| | fill | bad | shortfall | owed | madeUp |
|---|---|---|---|---|---|
| clean link | 2540 → 2313 | 0 | 0 | 0 | 0 |
| one flipped byte | 2540 → 2484 | **0** | **16711680** | 16696596 | 15084 and climbing |

`0xFF0000` exactly. `bad` is **zero** — nothing reports it. `madeUp` rises linearly at 18/frame
(4284 → 9684 → 15084 at frames 300/600/900) and would take roughly ten days of wall-clock at 30 fps
to work off the debt. Meanwhile the fill stays plausible, because the beaker is pouring out of its
own open top at about the rate it is manufacturing clones. Every symptom of a healthy chain.

The two defects are independent. Fixing the checksum (Adler-32, or Fletcher-16 mod 256, or simply
also covering the payload) makes this corruption detectable; bounding the shortfall — refuse or clamp
a `missing` larger than the pool can hold — makes it *survivable*, which matters because a peer that
reboots or is re-addressed can hand over a wild cumulative count without any corruption at all.

`beakers.html` flags a shortfall larger than the ring's capacity in red and names the cause, because
the number is the only symptom there is.

### 5.3 The payload is not checksummed at all, and `cr`/`cg` are not validated

`fletcher16(in, 14)` covers the header only. Everything from byte 16 is unprotected, and
`decodeSpill` reads `cr` and `cg` straight out of it with no range check.

`Particles.h` states the invariant the two-component encoding rests on: "mixing is a convex lerp,
which preserves `cr + cg <= kChromaOne` — the implied one can never go negative". A packet can break
it. Injecting 40 arrivals with `cr = cg = 0xFFFF` (both above `kChromaOne` = 65280), 600 frames,
against an otherwise identical run:

| | fill | mean r | mean g | dye weight (r+g) |
|---|---|---|---|---|
| clean | 1983 | 0.3686 | 0.0000 | 731.0 |
| 40 corrupted arrivals | 1983 | 0.3687 | 0.0171 | **765.1** |

Identical fill and identical physics — only the dye differs. **34.1 particle-units of dye created
from nothing**, `bad` = 0, and green appeared in a chain that contains no green. A less contrived
test (flip one random payload byte on 25% of packets) lost 4.1% of the total dye with nothing
reporting it.

Conservation of dye is the property §6 of the brief asks to be verified. It holds across a clean
link and it is not enforced against a dirty one.

---

## 6. D66 in the browser: a runaway, not a 33% overshoot

The brief's new §E3 was adopted — every beaker is addressed at `ps_beaker_init`, and
`ps_beaker_set_chain_position` is exposed so the page can express a *wrong* order too.

This page delivers point-to-point by default, so as §E3 anticipates, addressing costs nothing and
changes nothing there. Saying so is not the whole answer, though: the hardware has no such option,
and the browser is supposed to be where its behaviour is debugged. So `beakers.html` has a
**broadcast** carrier mode that hands each packet to every other beaker, which is what the radio
does. Three beakers, only beaker 0 tilted, 600 frames:

| carrier | addressing | fill | particlesIn | foreign |
|---|---|---|---|---|
| point-to-point | addressed | 3810 → 3810 (0.0%) | 1007 | 0 |
| point-to-point | unaddressed | 3810 → 3810 (0.0%) | 1007 | 0 |
| **broadcast** | **unaddressed** | **3810 → 10231 (+168.5%)** | **234390** | 0 |
| broadcast | addressed | 3810 → 3810 (0.0%) | 1007 | 429 |

D66 measured 150 → 200 on a host fixture, +33%. Sustained, it is **+168.5% and 233× the arrivals**,
because it compounds: a beaker that has been overfilled pours out of its own open top, that spill is
broadcast, and every other beaker takes it too. It is a feedback loop, not a one-off duplication,
and it stops only when the pools saturate. Worth knowing, because "the ring fills up out of nothing"
undersells how fast.

---

## 7. Judgement calls someone could reasonably have made differently

- **`beakerview.js` is new rather than an option on `cubeview.js`.** That module sizes itself from
  `innerWidth/innerHeight`, appends its own canvas and owns one bloom composer; N of it is N WebGL
  contexts. Bending it would put a second topology into the file `index.html` and `orient.html`
  share — and that sharing is the whole reason it exists.
- **The chain order lives in JS, not in the module.** `downstream[]` is a JS array; the module is
  told only each beaker's ring *position*. Chain order is user configuration on a real cube, so a
  page that hard-coded it would be proving something the firmware does not do. The positions handed
  to `setChainPosition` are derived by *walking* the configured order, not by assuming `0→1→2`; a
  configuration that is not a single closed ring has no positions to derive and goes unaddressed,
  and the page says so.
- **`ps_beaker_reset` does not reset the chain counters.** A refill on hardware does not reboot the
  radio, and D60 says a receiver should treat `totalOut` restarting at zero the way it treats a peer
  that rebooted. Resetting them here would hide the one place that can be watched.
- **The beaker artifact is built at the whole beaker tier**, not chroma bolted onto the default
  spacing. A tier is a named, buildable point on the ladder (`Config.h`); chroma-at-d-3.0 is a
  configuration nobody ships. The d=3.0 render in §3 was a throwaway measurement build, not an
  artifact.
- **Bloom was turned down**, from `(0.85, 0.55, 0.12)` to `(0.35, 0.4, 0.62)`, and made toggleable.
  At `cubeview.js`'s settings a pouring beaker blows out to white; the same frame from the host
  renderer, which has no bloom, is saturated red over magenta with no white in it. The white was the
  bloom, not the resolve. A page whose subject is colour should not ship a post-process that removes
  it.
- **`?warm=`, `?tilt=`, `?bloom=`, `?cast=`, `?addr=`.** The states worth looking at are a few
  seconds into a pour, and a debugging instrument whose starting point depends on how fast the
  machine is would be a poor one. `warm` steps at a fixed 1/60 so a URL lands on the same state
  every time.

## 8. Not done

- The `App.cpp` fill rule (§4.2) — `platform/app/` is out of scope, and it is a firmware change.
- The `nodes.html` import map (§4.4) — listed as not mine.
- Both wire-format defects (§5) — `core/`, and the brief asks for a finding rather than a patch.
- NVS persistence of chain order and colour: explicitly out of scope until the browser proves what
  the configuration is. It now has: an order per beaker, a dye per beaker, and a ring length.
