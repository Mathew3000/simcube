#include "partsim/Simulation.h"

namespace partsim {
namespace {

constexpr int kPanelRes = 32;
// A shake dies away over a handful of frames rather than instantly, so a flick reads as an
// impulse with follow-through instead of a single-frame jolt.
constexpr float kJerkDecay = 0.85f;

// Scene transition pacing. The palette crossfade and the drain/refill run concurrently.
constexpr float kFadeSeconds = 1.6f;
// Particles added or removed per physics step. At 60Hz this drains 3000 particles in about a
// second and a half, which reads as the tank emptying rather than as a glitch.
constexpr int kPopulationRate = 32;

}  // namespace

bool Simulation::init(int mode, int particleCount, uint32_t seed) {
  geometry_.clear();
  geometry_ = (mode == kSinglePanel) ? Geometry::slab(kPanelRes, kPanelRes, kPitch)
                                     : Geometry::cube(kPanelRes, kPitch);
  if (!volume_.build(geometry_, kSlabDepth, kCellSize)) return false;

  solver_.init();
  if (!field_.init(volume_)) return false;
  renderer_.setExposure(kSplatExposure);
  // A renderer that cannot serve this geometry is a configuration error, not a soft failure:
  // it means the build's render capacity is smaller than the faces it was asked to drive, and
  // the result would be silently unlit panels.
  const bool rok = (renderSetCount_ < 0) ? renderer_.init(geometry_)
                                         : renderer_.init(geometry_, renderSet_, renderSetCount_);
  if (!rok) return false;

  particles_.clear();
  // A slab tolerates a much smaller share of its nominal capacity than a cube; overfilling
  // leaves the fluid permanently over-compressed and it never settles.
  const int ceiling = (mode == kSinglePanel) ? capacity() / 4 : (capacity() * 9) / 10;
  fill(imin(particleCount, ceiling), kWater, seed);

  gravity_ = Vec3{0.0f, -kGravityMag, 0.0f};
  jerk_ = Vec3{0.0f, 0.0f, 0.0f};
  accumulator_ = 0.0f;
  emitterCount_ = 0;
  // A fresh init is a full reset, so auto-cycle does not survive it. Otherwise it silently
  // leaks between runs and a scene drifts when nobody asked it to.
  autoCycle_ = false;
  cycleClock_ = 0.0f;
  fade_ = 1.0f;
  // The targets must match what was just filled, or the transition logic sees a population it
  // did not ask for and immediately drains the whole tank.
  targetWater_ = particles_.n;
  targetSand_ = 0;
  rng_.reseed(seed ^ 0x5EEDu);
  stats.particles = particles_.n;
  stats.substeps = 0;
  return true;
}

bool Simulation::initScene(int mode, int sceneId, uint32_t seed) {
  if (!init(mode, 0, seed)) return false;
  setScene(sceneId);
  return true;
}

void Simulation::applySceneTargets(int sceneId) {
  sceneId_ = sceneId < 0 ? 0 : (sceneId >= sceneCount() ? sceneCount() - 1 : sceneId);
  const SceneDesc& sc = sceneAt(sceneId_);

  // A slab tolerates a far smaller share of its nominal capacity than a cube does; overfilling
  // leaves the fluid permanently over-compressed and it never settles.
  const int ceiling = (geometry_.count() == 1) ? capacity() / 4 : (capacity() * 9) / 10;
  targetSand_ = imin(sc.sandCount, ceiling);
  targetWater_ = imin(sc.waterCount, ceiling - targetSand_);
  if (targetWater_ < 0) targetWater_ = 0;

  emitterCount_ = imin(sc.emitterCount, kMaxEmitters);
  for (int i = 0; i < emitterCount_; ++i) emitters_[i] = sc.emitters[i];
  cycleClock_ = 0.0f;
}

void Simulation::setScene(int sceneId) {
  const int prevPalette = sceneAt(sceneId_).paletteIndex;
  (void)prevPalette;
  applySceneTargets(sceneId);

  particles_.clear();
  field_.clear();
  // Sand first so it starts at the bottom, which is where it ends up anyway -- starting it on
  // top just means watching it sink for several seconds before the scene looks right.
  if (targetSand_ > 0) fill(targetSand_, kSand, 0xA5A5u + (uint32_t)sceneId_);
  if (targetWater_ > 0)
    fill(particles_.n + targetWater_, kWater, 0xC3C3u + (uint32_t)sceneId_);

  fade_ = 1.0f;
  fadeFrom_ = fadeTo_ = &paletteAt(sceneAt(sceneId_).paletteIndex);
  renderer_.setPalette(fadeTo_);
  stats.particles = particles_.n;
}

void Simulation::transitionToScene(int sceneId) {
  fadeFrom_ = &paletteAt(sceneAt(sceneId_).paletteIndex);
  applySceneTargets(sceneId);
  fadeTo_ = &paletteAt(sceneAt(sceneId_).paletteIndex);
  fade_ = 0.0f;
  renderer_.setPaletteBlend(fadeFrom_, fadeTo_, 0.0f);
  // Emitters switch immediately, so a new fire lights while the old fluid is still draining.
  // Waiting until the drain finished would leave a visibly dead pause mid-transition.
}

void Simulation::countMaterials(int& water, int& sand) const {
  water = 0;
  sand = 0;
  for (int i = 0; i < particles_.n; ++i) (particles_.mat[i] == kSand ? sand : water)++;
}

bool Simulation::populationReached() const {
  int water = 0, sand = 0;
  countMaterials(water, sand);
  return water == targetWater_ && sand == targetSand_;
}

void Simulation::spawnOne(uint8_t material) {
  // Enters near the top so it falls in, rather than materialising inside the settled body where
  // it would be instantly over-dense and get flung out.
  const Aabb& b = volume_.box();
  const Vec3 s = b.size();
  const float m = kRestSpacing;
  particles_.add(Vec3{b.lo.x + m + rng_.nextFloat() * (s.x - 2.0f * m),
                      b.hi.y - m - rng_.nextFloat() * m,
                      b.lo.z + m + rng_.nextFloat() * (s.z - 2.0f * m)},
                 Vec3{0.0f, 0.0f, 0.0f}, material);
}

void Simulation::advanceTransition(float dt) {
  if (fade_ < 1.0f) {
    fade_ = pmin(1.0f, fade_ + dt / kFadeSeconds);
    renderer_.setPaletteBlend(fadeFrom_, fadeTo_, fade_);
    if (fade_ >= 1.0f) renderer_.setPalette(fadeTo_);
  }

  int water = 0, sand = 0;
  countMaterials(water, sand);
  int budget = kPopulationRate;

  // Drain first, so a scene that swaps one material for another does not briefly exceed capacity.
  while (budget > 0 && (water > targetWater_ || sand > targetSand_)) {
    const uint8_t victim = (sand > targetSand_) ? (uint8_t)kSand : (uint8_t)kWater;
    int found = -1;
    for (int i = particles_.n - 1; i >= 0; --i)
      if (particles_.mat[i] == victim) { found = i; break; }
    if (found < 0) break;
    particles_.removeAt(found);
    (victim == kSand ? sand : water)--;
    --budget;
  }
  while (budget > 0 && (water < targetWater_ || sand < targetSand_)) {
    if (sand < targetSand_) { spawnOne(kSand); ++sand; }
    else { spawnOne(kWater); ++water; }
    --budget;
  }
  stats.particles = particles_.n;
}

int Simulation::fill(int count, uint8_t material, uint32_t seed) {
  Rng r(seed);
  const Aabb& b = volume_.box();
  const float d = kRestSpacing;
  const int nx = imax(1, (int)(b.size().x / d));
  const int nz = imax(1, (int)(b.size().z / d));

  // Jittered rest lattice from the floor up. The jitter matters: a perfect lattice is a
  // metastable configuration and hides whether the solver actually holds together.
  for (int layer = 0; particles_.n < count && layer < 256; ++layer)
    for (int iz = 0; iz < nz && particles_.n < count; ++iz)
      for (int ix = 0; ix < nx && particles_.n < count; ++ix)
        particles_.add(Vec3{b.lo.x + (0.5f + (float)ix) * d + r.nextSigned() * 0.1f * d,
                            b.lo.y + (0.5f + (float)layer) * d + r.nextSigned() * 0.1f * d,
                            b.lo.z + (0.5f + (float)iz) * d + r.nextSigned() * 0.1f * d},
                       Vec3{0.0f, 0.0f, 0.0f}, material);
  return particles_.n;
}

void Simulation::setOrientation(Quat q) {
  gravity_ = rotateByConjugate(q, Vec3{0.0f, -kGravityMag, 0.0f});
}

void Simulation::setRenderSet(const int* panels, int count) {
  if (count < 0 || count > kMaxRenderPanels) return;
  for (int i = 0; i < count; ++i) renderSet_[i] = panels[i];
  renderSetCount_ = count;
}

void Simulation::fixedStep(float dt) {
  // Container acceleration is indistinguishable from gravity in the opposite direction, so a
  // shake is one vector subtraction rather than a separate force path.
  if (fade_ < 1.0f || !populationReached()) advanceTransition(dt);

  const Vec3 effective = gravity_ - jerk_;
  solver_.step(particles_, volume_, hash_, scratch_, defaultMaterials(), effective, dt);
  // Heat uses the same object-space gravity, so flames lean when the cube is tilted and get
  // pressed around when it is shaken, with no extra plumbing.
  field_.step(volume_, gravity_, jerk_, emitters_, emitterCount_, dt, rng_);
  jerk_ *= kJerkDecay;
  if (length2(jerk_) < 1e-4f) jerk_ = Vec3{0.0f, 0.0f, 0.0f};
}

int Simulation::advance(float wallDt) {
  // Guard against a tab switch or a debugger pause handing us a huge delta.
  if (wallDt > 0.25f) wallDt = 0.25f;
  if (wallDt < 0.0f) wallDt = 0.0f;
  accumulator_ += wallDt;

  int n = 0;
  while (accumulator_ >= kFixedDt && n < kMaxSubsteps) {
    fixedStep(kFixedDt);
    accumulator_ -= kFixedDt;
    ++n;
  }
  // Drop the backlog rather than accrue debt: catching up would make a slow frame trigger
  // even more work on the next one.
  if (n == kMaxSubsteps) accumulator_ = 0.0f;

  if (autoCycle_ && n > 0) {
    cycleClock_ += (float)n * kFixedDt;
    if (cycleClock_ >= sceneAt(sceneId_).dwellSeconds)
      transitionToScene((sceneId_ + 1) % sceneCount());
  }

  stats.substeps = n;
  stats.particles = particles_.n;
  return n;
}

uint32_t Simulation::stateHash() const {
  const size_t bytes = sizeof(float) * (size_t)particles_.n;
  uint64_t h = fnv1a(particles_.x, bytes);
  h = fnv1a(particles_.y, bytes, h);
  h = fnv1a(particles_.z, bytes, h);
  h = fnv1a(particles_.vx, bytes, h);
  h = fnv1a(particles_.vy, bytes, h);
  h = fnv1a(particles_.vz, bytes, h);
  return (uint32_t)(h ^ (h >> 32));
}

uint32_t Simulation::fieldHash() const {
  uint64_t h = 1469598103934665603ull;
  for (int i = 0; i < field_.cellCount(); ++i) {
    const uint8_t v = field_.at(i);
    h = fnv1a(&v, 1, h);
  }
  return (uint32_t)(h ^ (h >> 32));
}

uint32_t goldenHash(Simulation& sim, int steps, uint32_t seed) {
  uint64_t combined = 1469598103934665603ull;

  for (int pass = 0; pass < 2; ++pass) {
    const int sceneId = (pass == 0) ? kGoldenSceneA : kGoldenSceneB;
    if (!sim.initScene(Simulation::kCube, sceneId, seed + (uint32_t)pass)) return 0u;

    for (int s = 0; s < steps; ++s) {
      // Scripted motion: gravity swings around at fixed magnitude, plus a flick every 97 steps.
      // Derived from the step index through the polynomial trig in Math.h, so it is a pure
      // function of s on every target -- no clock, no RNG, no libm.
      const float t = (float)s * 0.017f;
      const Vec3 dir{fsin(t) * 0.6f, -1.0f, fcos(t * 0.73f) * 0.6f};
      sim.setGravityObject(normalize(dir) * kGravityMag);
      if (s % 97 == 96)
        sim.addContainerAccel(Vec3{fcos((float)s) * 40.0f, 25.0f, fsin((float)s) * 40.0f});
      sim.stepFixed();
    }

    const uint32_t st = sim.stateHash(), fl = sim.fieldHash();
    combined = fnv1a(&st, sizeof(st), combined);
    combined = fnv1a(&fl, sizeof(fl), combined);
  }
  return (uint32_t)(combined ^ (combined >> 32));
}

}  // namespace partsim
