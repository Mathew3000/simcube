// Runs the SAME scripted golden sequence in the WASM build that the host runs natively, and
// compares the resulting hashes bit-for-bit.
//
// This is the only thing that can actually verify the project's central promise -- that the
// browser shows you what the ESP32 will do. Fixed timestep, a seeded RNG, -ffp-contract=off
// and the ban on libm transcendentals all exist to make this pass; without the check they are
// just good intentions.
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, '..');

const createPartsim = (await import(join(root, 'platform/wasm/web/public/partsim.mjs'))).default;
const mod = await createPartsim();

const STEPS = 500, SEED = 0xc0ffee;  // must match kGoldenSteps / kGoldenSeed in core

const state = mod._ps_golden_hash(STEPS, SEED) >>> 0;
mod._ps_render();
const pixels = mod._ps_pixel_hash() >>> 0;
const hex = (n) => n.toString(16).padStart(8, '0');

const expected = readFileSync(join(root, 'scripts/golden_hash.txt'), 'utf8')
  .split('\n').map((l) => l.trim()).filter((l) => l && !l.startsWith('#'))[0].split(/\s+/);

console.log(`wasm  state ${hex(state)}  pixels ${hex(pixels)}`);
console.log(`host  state ${expected[0]}  pixels ${expected[1]}`);

let bad = 0;
if (hex(state) !== expected[0]) { console.error('MISMATCH: particle state diverged'); bad = 1; }
if (hex(pixels) !== expected[1]) { console.error('MISMATCH: rendered pixels differ'); bad = 1; }

// Also exercise the zero-copy view contract the frontend depends on.
const panels = mod._ps_panel_count();
if (panels !== 6) { console.error(`expected 6 panels, got ${panels}`); bad = 1; }
let litPanels = 0;
for (let i = 0; i < panels; i++) {
  const ptr = mod._ps_panel_ptr(i);
  const w = mod._ps_panel_w(i), h = mod._ps_panel_h(i);
  const view = new Uint8Array(mod.HEAPU8.buffer, ptr, w * h * 4);
  if (view.length !== w * h * 4) { console.error(`panel ${i} view detached`); bad = 1; }
  if (view.some((v, k) => k % 4 !== 3 && v > 0)) litPanels++;
  // Alpha must be opaque everywhere, including black texels.
  for (let k = 3; k < view.length; k += 4)
    if (view[k] !== 255) { console.error(`panel ${i} alpha not opaque`); bad = 1; k = view.length; }
}
if (litPanels === 0) { console.error('no panel rendered anything'); bad = 1; }
console.log(`views ok, ${litPanels}/${panels} panels lit`);

// The basis table the frontend places its quads from must match a unit cube.
for (let i = 0; i < panels; i++) {
  const bp = mod._ps_panel_basis(i);
  const b = new Float32Array(mod.HEAPF32.buffer, bp, 12);
  const n = [b[9], b[10], b[11]];
  const len = Math.hypot(...n);
  if (Math.abs(len - 1) > 1e-4) { console.error(`panel ${i} normal not unit: ${len}`); bad = 1; }
}
console.log('basis ok');

process.exit(bad);
