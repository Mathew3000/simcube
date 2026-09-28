# Lightweight surface water — first prototype

Branch: `feature/lightweight-water`.

This backend targets six-faced LED cubes that tilt, slosh, fill and pour into
the next cube by identity. It emulates the visible liquid boundary instead of
solving particle density throughout the volume. It is separate from the PBF
backend: existing scenes, golden hashes and firmware selection are unchanged.

## Try it

With Emscripten and CMake installed (the same prerequisites as the other previews):

```sh
scripts/build_wasm.sh --surface
scripts/serve.sh
```

Open <http://localhost:8080/surface.html>. Use `?res=64` for six 64×64 faces per
cube, or choose 8, 32 or 64 in the page. Pick a cube and drag, use the pitch/roll
sliders, or click **Tip to pour**. **Shake** adds a slosh impulse. Turn transfer
off to inspect a closed container at any angle. **Swap positions** demonstrates
that routing follows cube identity, not world position. Both cubes are displayed
with all six LED faces; the green +Y rim marks a *virtual* opening.

The preview is a two-cube ring (1 → 2 → 1). The receiver's orientation controls
its own water. A full receiver applies backpressure: the source retains any
volume that cannot be accepted. Reset initializes 65% and 15%, so the conserved
total is 80% of one cube's capacity. Pause freezes simulation; camera controls
and shading remain available. Timings exclude WebGL and are browser timings,
not measurements or predictions for ESP32-S3.

The particle links require their respective existing WASM builds:
`scripts/build_wasm.sh` and `scripts/build_wasm.sh --beaker`.

## Model and shader

`core/include/partsim/SurfaceWater.h` and `core/src/SurfaceWater.cpp` contain the
portable C++ implementation. `platform/wasm/surface_bindings.cpp` exports it as
an independent WASM module; JS handles controls and displays the C++ pixels.
There is no separate JavaScript physics model and no GPU-only water shader.

- **Volume:** one float in [0,1], independent of panel resolution. Coordinates
  are the unit cube [-0.5,0.5]³.
- **Surface:** a shared plane, `dot(up, position) = level`. Its normal follows
  object-space gravity through a damped spring. Acceleration and explicit
  impulses excite slosh. A deterministic antipodal nudge handles exact inversion.
- **Level:** recalculated from volume and the current surface normal. Integrate
  clipped affine column heights over a square analytically, then bisect the
  plane offset 19 times. Integrating along the dominant normal component avoids
  division by nearly zero components in axis-aligned poses. Tilt does not change
  volume. This is bounded work per update, independent of LED count.
- **Pour:** a stylized rate proportional to submerged rim head to the power 3/2.
  The +Y opening's lowest edge controls the head. Only accepted volume is removed
  from the donor. Incoming water excites the receiver's shading ripple oscillator.
- **Shading:** each face uses the existing `Panel` basis. Every LED samples the
  same surface in object coordinates, with an antialiased waterline, blue depth
  ramp and shared polynomial lighting modes. Ripples currently modulate lighting,
  not the surface geometry. There is no per-pixel neighbour search, square root,
  trigonometric call or division. Output is RGBA into caller-owned storage.

The water object is 56 bytes on the tested host/WASM builds. An RGBA scratch
face is 4,096 bytes at 32² or 16,384 bytes at 64² and can be reused face by face.
HUB75 bitplane/DMA storage is additional. The browser holds all faces for both
cubes to supply textures; an MCU does not need that browser allocation.

## Validation and measurement

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
build/tests/partsim_tests surface_water
build/platform/host/partsim_surface_bench
node scripts/check_surface.mjs
```

Native tests cover known analytic cube cuts, monotonicity and symmetry,
near-axis poses, fill preservation through arbitrary rotation, settling and
exact inversion, transfer conservation, full-receiver backpressure and renderer
buffer bounds at 8²/32²/64². The Node check exercises the actual WASM API,
inversion/draining, conserved total and every face buffer at all three sizes.
`partsim_surface_bench` separates one simulation update from six-face shading.
Neither host nor WASM timing establishes the hardware frame rate.

## Integration boundary and remaining work

This is a working host/browser prototype, **not yet the active ESP32 firmware
backend**. The device application still instantiates its existing particle
simulation. The next device work should be:

1. Add a dedicated surface role/environment that does not instantiate the PBF
   pools. Feed `MotionSource::down()` into `step()`; convert container acceleration
   into g before passing it. Use a fixed update cadence (start at 120 Hz, measure
   whether 60 Hz suffices) and render independently at the selected display rate.
2. Feed one face scratch buffer into panel mapping and the HUB75 blitter. Measure
   step, shading, colour conversion and blit separately on S3; also measure actual
   panel refresh. Tune colour depth and node count from those measurements.
3. Define a versioned surface snapshot for display nodes. Transmit volume, normal,
   level and shading phase/state, rather than particles or full images. Synchronize
   snapshot application at frame boundaries. Do not send the native C++ object
   layout as a wire format.
4. Implement aggregate-volume radio transfers with stable source/session IDs,
   sequence numbers, cumulative counters, duplicate rejection and periodic final
   counter announcements. Define reset/reboot recovery and acknowledgements for
   receiver capacity before removing source water. `pourTo()` is an in-process
   reference, **not a reliable distributed protocol**.
5. Tune the visible result on real panels before adding complexity: decorative
   ballistic droplets, local receiving splashes, then a coarse dye field if needed.

Known simplifications: planar slosh (no breaking waves), stylized flow, no
separate droplets, no volumetric dye/mixing and no transport delay. The two-cube
preview applies transfers sequentially each fixed tick. Long browser stalls are
discarded instead of catching up a large physical time interval. Float volume
conservation has rounding error; a deployed network ledger should use integer
volume units with fractional rate accumulation. Surface snapshots have not yet
been checked for bit-identical host/WASM/S3 serialization.
