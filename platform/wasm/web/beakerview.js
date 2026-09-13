// Several cubes in ONE three.js scene, for the beaker chain page.
//
// Deliberately not an option on cubeview.js. That module is built around a single fullscreen cube:
// it sizes itself from innerWidth/innerHeight, appends its own canvas and owns one bloom composer.
// N instances of it would be N WebGL contexts and N composers, and bending it into shape would put
// a second topology into the file index.html and orient.html depend on -- an orientation checker
// that shares a scene builder with the live simulation is the entire reason cubeview.js exists, so
// it is the last file to make conditional. This builds the same quads from the same basis and
// leaves that one alone.
import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { EffectComposer } from './vendor/jsm/postprocessing/EffectComposer.js';
import { RenderPass } from './vendor/jsm/postprocessing/RenderPass.js';
import { UnrealBloomPass } from './vendor/jsm/postprocessing/UnrealBloomPass.js';

// `mod` supplies the geometry through the ps_beaker_* entry points; every quad is placed from
// ps_beaker_panel_basis rather than from a hardcoded cube, so the physics and the picture cannot
// disagree about which face is which -- which matters here more than anywhere, because the whole
// page is about what leaves through face 3.
export function createBeakerView(mod, count, { openFace = 3 } = {}) {
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(45, innerWidth / innerHeight, 1, 1200);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.setSize(innerWidth, innerHeight);
  document.body.append(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  // The left button tilts the beakers -- that is the interaction the page exists for. Camera
  // orbit goes to the right button, the same split cubeview.js makes and for the same reason.
  controls.mouseButtons = { LEFT: null, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.ROTATE };

  const faces = mod._ps_beaker_panel_count();
  const b0 = new Float32Array(mod.HEAPF32.buffer, mod._ps_beaker_panel_basis(0), 12);
  const side = Math.hypot(b0[3], b0[4], b0[5]) * mod._ps_beaker_panel_w(0);
  const pitchX = side * 1.45;

  const beakers = [];
  for (let i = 0; i < count; i++) {
    // Two groups: the outer one places the beaker on the shelf and never rotates, the inner one
    // carries the tilt. Rotating a single group would swing the cube along the row as it poured.
    const slot = new THREE.Group();
    slot.position.x = (i - (count - 1) / 2) * pitchX;
    scene.add(slot);
    const body = new THREE.Group();
    slot.add(body);

    const panels = [];
    for (let f = 0; f < faces; f++) {
      const b = new Float32Array(mod.HEAPF32.buffer, mod._ps_beaker_panel_basis(f), 12);
      const origin = new THREE.Vector3(b[0], b[1], b[2]);
      const u = new THREE.Vector3(b[3], b[4], b[5]);
      const v = new THREE.Vector3(b[6], b[7], b[8]);
      const n = new THREE.Vector3(b[9], b[10], b[11]);
      const w = mod._ps_beaker_panel_w(f), h = mod._ps_beaker_panel_h(f);

      // One view per (beaker, face), constructed once. ALLOW_MEMORY_GROWTH is off precisely so
      // these stay valid: a grown heap swaps in a new ArrayBuffer and every one of these goes
      // zero-length, which shows up as black panels and no error at all.
      const view = new Uint8Array(mod.HEAPU8.buffer, mod._ps_beaker_panel_ptr(i, f), w * h * 4);
      const tex = new THREE.DataTexture(view, w, h, THREE.RGBAFormat);
      tex.needsUpdate = true;

      const mesh = new THREE.Mesh(
        new THREE.PlaneGeometry(u.length() * w, v.length() * h),
        // BackSide: (u, v, n) is right-handed with n pointing INWARD, so the plane's front face
        // looks into the volume while we stand outside it.
        new THREE.MeshBasicMaterial({ map: tex, side: THREE.BackSide, toneMapped: false }),
      );
      mesh.quaternion.setFromRotationMatrix(
        new THREE.Matrix4().makeBasis(u.clone().normalize(), v.clone().normalize(), n));
      mesh.position.copy(origin)
          .add(u.clone().multiplyScalar(w / 2))
          .add(v.clone().multiplyScalar(h / 2));
      body.add(mesh);
      panels.push({ face: f, tex, view, mesh });
    }

    // The open face gets a lit rim and the other eight edges stay dark, so which way a beaker has
    // to be tipped to pour is readable from the picture rather than from the face index.
    const box = new THREE.BoxGeometry(side, side, side);
    body.add(new THREE.LineSegments(new THREE.EdgesGeometry(box),
                                    new THREE.LineBasicMaterial({ color: 0x1b2634 })));
    const rim = new THREE.EdgesGeometry(new THREE.PlaneGeometry(side, side));
    const rimLines = new THREE.LineSegments(rim, new THREE.LineBasicMaterial({ color: 0x4bbf73 }));
    rimLines.rotation.x = -Math.PI / 2;
    rimLines.position.y = openFace === 3 ? side / 2 : -side / 2;
    body.add(rimLines);

    beakers.push({ index: i, slot, body, panels, rimLines });
  }

  const span = pitchX * Math.max(1, count - 1) + side;
  // Panned right, not centred: the control panel owns the left of the window, and a row centred in
  // the viewport puts beaker 0 behind it. Camera and target move together so this is a pan rather
  // than a rotation -- orbiting still turns about the row.
  const shift = -side * 0.75;
  controls.target.set(shift, 0, 0);
  camera.position.set(shift, side * 0.75, span * 1.15 + side * 1.2);
  controls.minDistance = side;
  controls.maxDistance = span * 4 + side * 4;
  controls.update();

  // Arrows along the shelf showing the chain order. Rebuilt rather than reoriented when the order
  // changes, because the order is what a user configures on a real cube and a stale arrow would be
  // a lie about the one thing this page is for.
  const arrowGroup = new THREE.Group();
  scene.add(arrowGroup);
  function setChain(next) {
    arrowGroup.clear();
    if (!next) return;
    for (let i = 0; i < count; i++) {
      const j = next[i];
      if (j == null || j === i) continue;
      const from = beakers[i].slot.position.clone();
      const to = beakers[j].slot.position.clone();
      const dir = to.clone().sub(from);
      const wrap = Math.abs(j - i) > 1;   // the ring's closing hop, drawn lower and in front
      from.y = to.y = wrap ? -side * 0.95 : -side * 0.66;
      // Toward the viewer, so the hops do not vanish behind the cubes they run between.
      from.z = to.z = side * (wrap ? 0.95 : 0.7);
      const len = dir.length() - side * 0.55;
      if (len <= 0) continue;
      const start = from.clone().add(dir.clone().normalize().multiplyScalar(side * 0.3));
      arrowGroup.add(new THREE.ArrowHelper(dir.clone().normalize(), start, len,
                                           wrap ? 0x9d6bd6 : 0x3f8fd0, side * 0.12, side * 0.07));
    }
  }

  // Much weaker than cubeview.js's (0.85, 0.55, 0.12), and MEASURED against the host renderer
  // rather than chosen by eye. Those settings make the cubes read as LEDs, which is what index.html
  // wants; here they blow a pouring beaker out to white, and the colour is the entire subject. The
  // same frame rendered by platform/host/spill_ppm (no bloom at all) is saturated red over magenta
  // with no white in it, so the white was the bloom pass, not the resolve. Threshold raised above
  // where the fluid sits, so only the brightest texels glow.
  const composer = new EffectComposer(renderer);
  composer.addPass(new RenderPass(scene, camera));
  const bloomPass = new UnrealBloomPass(new THREE.Vector2(innerWidth, innerHeight), 0.35, 0.4, 0.62);
  composer.addPass(bloomPass);

  addEventListener('resize', () => {
    camera.aspect = innerWidth / innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(innerWidth, innerHeight);
    composer.setSize(innerWidth, innerHeight);
  });

  const camRight = () => new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld, 0);
  const camUp = () => new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld, 1);

  return { THREE, scene, camera, renderer, controls, beakers, side, pitchX, setChain,
           camRight, camUp, bloomPass, render: () => composer.render() };
}
