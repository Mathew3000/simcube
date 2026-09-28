import * as THREE from 'three';
import { createBeakerView } from './beakerview.js';

const $ = id => document.getElementById(id);
let mod;
try {
  const { default: createSurfaceWater } = await import('./public/partsim_surface.mjs');
  mod = await createSurfaceWater();
} catch (error) {
  $('error').style.display = 'block';
  $('error').textContent = `Unable to load the surface-water module.\n\nRun scripts/build_wasm.sh --surface, then scripts/serve.sh.\nOpen /surface.html over HTTP.\n\n${error.message}`;
  throw error;
}
const query = new URLSearchParams(location.search);
const requested = Number(query.get('res') || 32);
const res = [8, 32, 64].includes(requested) ? requested : 32;
if (!mod._sw_init(res)) throw new Error('Surface-water geometry initialization failed');
mod._sw_render();
$('resolution').value = String(res);
$('resolution').onchange = () => { query.set('res', $('resolution').value); location.search = query; };

// Adapter reuses the existing six-face geometry viewer, without linking or
// allocating any particle simulation. The viewer's historical openFace=3
// convention means +Y for its rim only; our actual geometry +Y face is 5.
const adapter = {
  HEAPU8: mod.HEAPU8, HEAPF32: mod.HEAPF32,
  _ps_beaker_panel_count: () => 6,
  _ps_beaker_panel_w: () => res,
  _ps_beaker_panel_h: () => res,
  _ps_beaker_panel_basis: face => mod._sw_basis(face),
  _ps_beaker_panel_ptr: (cube, face) => mod._sw_pixels(cube, face),
};
const view = createBeakerView(adapter, 2, { openFace: 3 });
view.bloomPass.enabled = false;
view.setChain([1, 0]);
const poses = [new THREE.Euler(0, 0, 0, 'XYZ'), new THREE.Euler(0, 0, 0, 'XYZ')];
let paused = false, selected = 0;
function poseChanged() {
  view.beakers[selected].body.quaternion.setFromEuler(poses[selected]);
  syncSliders();
}
function syncSliders() {
  const e = poses[selected];
  $('pitch').value = String(Math.round(THREE.MathUtils.radToDeg(e.x)));
  $('roll').value = String(Math.round(THREE.MathUtils.radToDeg(e.z)));
  $('pitchValue').value = `${$('pitch').value}°`;
  $('rollValue').value = `${$('roll').value}°`;
}
$('selected').onchange = () => { selected = Number($('selected').value); syncSliders(); };
for (const id of ['pitch', 'roll']) $(id).oninput = () => {
  poses[selected].set(THREE.MathUtils.degToRad(Number($('pitch').value)), 0,
                      THREE.MathUtils.degToRad(Number($('roll').value)), 'XYZ');
  poseChanged();
};
$('upright').onclick = () => { poses[selected].set(0, 0, 0); poseChanged(); };
$('tip').onclick = () => { poses[selected].set(0, 0, -2.15); poseChanged(); };
$('shake').onclick = () => mod._sw_impulse(selected, 2.0, 0.2, 0.8);
$('bloom').onchange = () => { view.bloomPass.enabled = $('bloom').checked; };
$('pause').onclick = () => { paused = !paused; $('pause').textContent = paused ? 'Resume' : 'Pause'; };
$('reset').onclick = () => {
  mod._sw_init(res);
  for (let i = 0; i < 2; ++i) { poses[i].set(0,0,0); view.beakers[i].body.quaternion.identity(); }
  syncSliders();
  mod._sw_render();
};
$('swap').onclick = () => {
  const a = view.beakers[0].slot.position.x;
  view.beakers[0].slot.position.x = view.beakers[1].slot.position.x;
  view.beakers[1].slot.position.x = a;
  view.setChain([1, 0]);
};

// Permanent number labels follow identity when display positions are swapped.
for (let i = 0; i < 2; ++i) {
  const c = document.createElement('canvas'); c.width = 256; c.height = 64;
  const ctx = c.getContext('2d');
  ctx.fillStyle = '#bce8ff'; ctx.font = 'bold 38px sans-serif'; ctx.textAlign = 'center';
  ctx.fillText(`CUBE ${i+1}`, 128, 46);
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthTest: false }));
  sprite.scale.set(24, 6, 1); sprite.position.set(0, 27, 0);
  view.beakers[i].slot.add(sprite);
}

let drag = null;
const canvas = view.renderer.domElement;
canvas.addEventListener('pointerdown', event => {
  if (event.button !== 0) return;
  drag = { x: event.clientX, y: event.clientY };
  canvas.setPointerCapture(event.pointerId);
});
canvas.addEventListener('pointermove', event => {
  if (!drag) return;
  const dx = event.clientX-drag.x, dy = event.clientY-drag.y;
  poses[selected].x = THREE.MathUtils.clamp(poses[selected].x+dy*0.008, -Math.PI, Math.PI);
  poses[selected].z = THREE.MathUtils.clamp(poses[selected].z-dx*0.008, -Math.PI, Math.PI);
  drag = { x: event.clientX, y: event.clientY };
  poseChanged();
});
canvas.addEventListener('pointerup', () => { drag = null; });
canvas.addEventListener('pointercancel', () => { drag = null; });
canvas.addEventListener('contextmenu', event => event.preventDefault());

let previous = performance.now(), accumulator = 0, lastHud = 0;
let simMs = 0, shadeMs = 0;
const fixedDt = 1/120;
function animate(now) {
  const elapsed = Math.min((now-previous)/1000, 0.05); previous = now;
  for (let i = 0; i < 2; ++i) {
    const q = view.beakers[i].body.quaternion;
    mod._sw_orient(i, q.x, q.y, q.z, q.w);
  }
  const start = performance.now();
  if (!paused) {
    accumulator += elapsed;
    while (accumulator >= fixedDt) {
      mod._sw_step(fixedDt, $('pouring').checked ? 1 : 0);
      accumulator -= fixedDt;
    }
  } else accumulator = 0;
  const afterSim = performance.now();
  mod._sw_render();
  const afterShade = performance.now();
  simMs = simMs*0.9+(afterSim-start)*0.1;
  shadeMs = shadeMs*0.9+(afterShade-afterSim)*0.1;
  for (const b of view.beakers) for (const panel of b.panels) panel.tex.needsUpdate = true;
  view.controls.update(); view.render();
  if (now-lastHud > 200) {
    const v0 = mod._sw_volume(0), v1 = mod._sw_volume(1);
    $('volumes').textContent = `Cube 1: ${(v0*100).toFixed(1)}% · Cube 2: ${(v1*100).toFixed(1)}% · Total: ${((v0+v1)*100).toFixed(2)}% of one cube`;
    $('flow').textContent = `Flow 1→2: ${(mod._sw_flow(0)*100).toFixed(1)}%/s · 2→1: ${(mod._sw_flow(1)*100).toFixed(1)}%/s`;
    $('stats').textContent = `${res}×${res} × 6 faces/cube · ${mod._sw_state_bytes()} B water state/cube · simulation ${simMs.toFixed(2)} ms · shading ${shadeMs.toFixed(2)} ms (both cubes; excludes WebGL)`;
    lastHud = now;
  }
  requestAnimationFrame(animate);
}
requestAnimationFrame(animate);
