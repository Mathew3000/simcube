// Shared three.js scene for the cube. Used by index.html (the live simulation) and by
// orient.html (the orientation checker), so both are guaranteed to place panels identically --
// an orientation check that built its own scene would prove nothing about the real one.
import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { EffectComposer } from './vendor/jsm/postprocessing/EffectComposer.js';
import { RenderPass } from './vendor/jsm/postprocessing/RenderPass.js';
import { UnrealBloomPass } from './vendor/jsm/postprocessing/UnrealBloomPass.js';

// Builds the scene, one textured quad per panel, and an orbit camera.
//
// Every quad is placed from ps_panel_basis, never from a hardcoded cube layout, so the physics
// and the visuals cannot disagree about where a panel is or which way it faces.
export function createCubeView(mod, { canvasParent = document.body, frame = true,
                                      bloom = true } = {}) {
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(45, innerWidth / innerHeight, 1, 500);
  camera.position.set(46, 34, 62);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.setSize(innerWidth, innerHeight);
  canvasParent.append(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 40;
  controls.maxDistance = 200;
  // The left button is the app's -- it rotates the OBJECT, which is the whole point of the
  // interaction. Camera orbit moves to the right button.
  controls.mouseButtons = { LEFT: null, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.ROTATE };

  const cube = new THREE.Group();
  scene.add(cube);

  const panels = [];
  for (let i = 0; i < mod._ps_panel_count(); i++) {
    const b = new Float32Array(mod.HEAPF32.buffer, mod._ps_panel_basis(i), 12);
    const origin = new THREE.Vector3(b[0], b[1], b[2]);
    const u = new THREE.Vector3(b[3], b[4], b[5]);    // world step per +1 texel in x
    const v = new THREE.Vector3(b[6], b[7], b[8]);    // world step per +1 texel in y
    const n = new THREE.Vector3(b[9], b[10], b[11]);  // unit INWARD normal
    const w = mod._ps_panel_w(i), h = mod._ps_panel_h(i);

    // DataTexture defaults to flipY = false, unpackAlignment = 1 and NearestFilter, which is
    // exactly right here: panel row 0 is the BOTTOM row (matching WebGL's bottom-left texture
    // origin) so no flip is needed, and nearest keeps LED pixels crisp instead of smearing.
    const view = new Uint8Array(mod.HEAPU8.buffer, mod._ps_panel_ptr(i), w * h * 4);
    const tex = new THREE.DataTexture(view, w, h, THREE.RGBAFormat);
    tex.needsUpdate = true;

    const mesh = new THREE.Mesh(
      new THREE.PlaneGeometry(u.length() * w, v.length() * h),
      // BackSide, deliberately: (u, v, n) is right-handed with n pointing INWARD, so the
      // plane's front face looks into the volume while we view it from outside. Building the
      // basis as (-u, v, -n) to face outward would mirror the texture horizontally instead.
      new THREE.MeshBasicMaterial({ map: tex, side: THREE.BackSide, toneMapped: false }),
    );

    const basis = new THREE.Matrix4().makeBasis(u.clone().normalize(), v.clone().normalize(), n);
    mesh.quaternion.setFromRotationMatrix(basis);
    mesh.position.copy(origin)
        .add(u.clone().multiplyScalar(w / 2))
        .add(v.clone().multiplyScalar(h / 2));

    cube.add(mesh);
    panels.push({ index: i, tex, view, w, h, mesh });
  }

  if (frame && panels.length === 6) {
    // A dark frame, which is also honest: real HUB75 panels have a physical bezel at the seams.
    //
    // The size must come from the basis, not the texel count. `panels[0].w * 1.0` was a texel
    // count standing in for a world size -- correct only while the pitch happened to be 1.0, and
    // at 64 texels/pitch 0.5 it would have drawn a 64-unit box around a 32-unit cube. Every other
    // placement in this file already derives from u/v; this was the one that did not.
    const size = panels[0].mesh.geometry.parameters.width;
    cube.add(new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(size, size, size)),
      new THREE.LineBasicMaterial({ color: 0x1b2634 }),
    ));
  }

  // Bloom, so the panels read as emitting light rather than as coloured squares. The threshold
  // is low because every lit texel is supposed to glow -- these are LEDs, not lit surfaces.
  let composer = null;
  let bloomPass = null;
  if (bloom) {
    composer = new EffectComposer(renderer);
    composer.addPass(new RenderPass(scene, camera));
    bloomPass = new UnrealBloomPass(new THREE.Vector2(innerWidth, innerHeight), 0.85, 0.55, 0.12);
    composer.addPass(bloomPass);
  }

  addEventListener('resize', () => {
    camera.aspect = innerWidth / innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(innerWidth, innerHeight);
    if (composer) composer.setSize(innerWidth, innerHeight);
  });

  // One entry point, so callers do not have to know whether bloom is in play.
  const render = () => (composer ? composer.render() : renderer.render(scene, camera));
  const setBloom = (on) => { if (bloomPass) bloomPass.enabled = on; };

  const camRight = () => new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld, 0);
  const camUp = () => new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld, 1);

  return { THREE, scene, camera, renderer, controls, cube, panels, camRight, camUp,
           render, setBloom, bloomPass };
}
