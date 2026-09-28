// Run after scripts/build_wasm.sh --surface. Exercises the actual browser ABI.
import assert from 'node:assert/strict';
import createSurfaceWater from '../platform/wasm/web/public/partsim_surface.mjs';
const m = await createSurfaceWater();
assert.equal(m._sw_init(17), 0);
for (const res of [8,32,64]) {
  assert.equal(m._sw_init(res), 1);
  assert.equal(m._sw_resolution(), res);
  assert.equal(m._sw_pixels(-1, 0), 0);
  assert.equal(m._sw_pixels(0, 6), 0);
  const initial = m._sw_volume(0)+m._sw_volume(1);
  for (let i = 0; i < 60; ++i) m._sw_step(1/120, 1);
  assert.ok(Math.abs(m._sw_volume(0)-0.65) < 1e-6, 'upright cube must not leak');
  m._sw_orient(0, 1, 0, 0, 0); // 180 degrees around X
  for (let i = 0; i < 1200; ++i) m._sw_step(1/120, 1);
  assert.ok(m._sw_volume(0) < 0.1, 'inverted cube should drain');
  assert.ok(Math.abs(m._sw_volume(0)+m._sw_volume(1)-initial) < 1e-5, 'conserved total');
  m._sw_render();
  for (let cube = 0; cube < 2; ++cube) for (let face = 0; face < 6; ++face) {
    const ptr = m._sw_pixels(cube, face);
    assert.ok(ptr > 0);
    for (let i = 0; i < res*res; ++i) assert.equal(m.HEAPU8[ptr+i*4+3], 255);
  }
  console.log(`ok surface WASM: ${res}x${res}, fill + inversion + transfer + six-face output`);
}
