#include "partsim/Simulation.h"

namespace partsim {
namespace {

constexpr int kPanelRes = 32;
// A shake dies away over a handful of frames rather than instantly, so a flick reads as an
// impulse with follow-through instead of a single-frame jolt.
constexpr float kJerkDecay = 0.85f;

}  // namespace

bool Simulation::init(int mode, int particleCount, uint32_t seed) {
  geometry_.clear();
  geometry_ = (mode == kSinglePanel) ? Geometry::slab(kPanelRes, kPanelRes, kPitch)
                                     : Geometry::cube(kPanelRes, kPitch);
  if (!volume_.build(geometry_, kSlabDepth, kCellSize)) return false;

  solver_.init();
  if (!field_.init(volume_)) return false;
  renderer_.setExposure(kSplatExposure);
  renderer_.init(geometry_);

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

void Simulation::setScene(int sceneId) {
  const SceneDesc& sc = sceneAt(sceneId);
  sceneId_ = sceneId < 0 ? 0 : (sceneId >= sceneCount() ? sceneCount() - 1 : sceneId);

  // A slab tolerates a far smaller share of its nominal capacity than a cube does; overfilling
  // leaves the fluid permanently over-compressed and it never settles.
  const int ceiling = (geometry_.count() == 1) ? capacity() / 4 : (capacity() * 9) / 10;

  particles_.clear();
  field_.clear();
  // Sand first so it starts at the bottom, which is where it ends up anyway -- starting it on
  // top just means watching it sink for several seconds before the scene looks right.
  if (sc.sandCount > 0) fill(imin(sc.sandCount, ceiling), kSand, 0xA5A5u + (uint32_t)sceneId_);
  if (sc.waterCount > 0)
    fill(imin(particles_.n + sc.waterCount, ceiling), kWater, 0xC3C3u + (uint32_t)sceneId_);

  emitterCount_ = imin(sc.emitterCount, kMaxEmitters);
  for (int i = 0; i < emitterCount_; ++i) emitters_[i] = sc.emitters[i];

  renderer_.setPalette(&paletteAt(sc.paletteIndex));
  cycleClock_ = 0.0f;
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

void Simulation::fixedStep(float dt) {
  // Container acceleration is indistinguishable from gravity in the opposite direction, so a
  // shake is one vector subtraction rather than a separate force path.
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
      setScene((sceneId_ + 1) % sceneCount());
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
