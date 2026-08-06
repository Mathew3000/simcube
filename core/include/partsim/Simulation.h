#pragma once
#include "partsim/Renderer.h"
#include "partsim/Scene.h"
#include "partsim/Rng.h"
#include "partsim/Solver.h"

namespace partsim {

struct Stats {
  float simMs;      // filled in by the platform layer, which owns the clock
  float renderMs;
  int particles;
  int substeps;
};

// Everything the platform layers talk to. Owns every pool by value, so a Simulation is
// ~1.2MB at host capacity and must live in static storage -- never on the stack.
//
// The core works entirely in OBJECT SPACE: gravity arrives already expressed in the object's
// frame. That one decision is why the browser (rotate the object, gravity stays world-down)
// and the IMU (which natively reports gravity in the device frame) are the same code path.
class Simulation {
 public:
  enum Mode : int { kCube = 0, kSinglePanel = 1 };

  bool init(int mode, int particleCount, uint32_t seed);
  // Loads a scene preset: refills particles, sets the palette, installs heat emitters.
  bool initScene(int mode, int sceneId, uint32_t seed);
  // Immediate: replaces the population in one go. Fine for a deliberate user action.
  void setScene(int sceneId);
  // Gradual: drains the old materials and refills the new ones over a couple of seconds while
  // crossfading the palette. Used by auto-cycle, where a hard reset reads as a glitch rather
  // than as a change of scene.
  void transitionToScene(int sceneId);
  int scene() const { return sceneId_; }
  bool transitioning() const { return fade_ < 1.0f || !populationReached(); }

  // Drift between scenes on a timer, so the cube is an ambient object rather than something
  // that has to be operated.
  void setAutoCycle(bool on) { autoCycle_ = on; cycleClock_ = 0.0f; }
  bool autoCycle() const { return autoCycle_; }

  // World-space down, rotated into object space by the object's orientation.
  void setOrientation(Quat q);
  // Directly set object-space gravity (used by tests and the golden sequence).
  void setGravityObject(Vec3 g) { gravity_ = g; }
  Vec3 gravityObject() const { return gravity_; }

  // Acceleration OF THE CONTAINER, which is how a shake enters the simulation.
  //
  // Mind the sign: inside the container its acceleration is indistinguishable from gravity in
  // the opposite direction, so accelerating the cube upward presses the fluid DOWN (an
  // elevator starting to rise), and shoving it to the right piles the water on the LEFT. That
  // is physically correct and the reason this is named for the container rather than for the
  // fluid -- "addJerk(up)" reads as "throw the water up", which is backwards.
  //
  // Decays over a few frames so a flick reads as an impulse with follow-through.
  void addContainerAccel(Vec3 a) { jerk_ += a; }
  // Overwrite rather than accumulate. The browser feeds discrete flicks, so accumulating with
  // decay is right there; an IMU feeds a continuous signal at 208 Hz, and accumulating that
  // would sum ~3 samples per physics step into a container acceleration several times the
  // measured one. The IMU path therefore sets, and the decay in fixedStep never engages.
  void setContainerAccel(Vec3 a) { jerk_ = a; }
  void addContainerAccelWorld(Quat q, Vec3 a) { jerk_ += rotateByConjugate(q, a); }
  Vec3 containerAccel() const { return jerk_; }

  // Fixed-timestep accumulator. Returns how many physics steps ran.
  int advance(float wallDt);
  // Exactly one physics step, for tests and the golden sequence.
  void stepFixed() { fixedStep(kFixedDt); }

  // Renders with the accumulator's unconsumed time folded into the splat, so the picture keeps
  // moving smoothly between physics steps rather than holding still and then jumping. Costs
  // nothing: it is one multiply-add per particle inside a loop that already reads velocity.
  // Splat everything into the accumulation buffers without resolving colour. The ESP32 has no
  // room for an RGBA copy of all six panels, so the firmware calls this and then resolves each
  // face into one shared staging buffer. Identical splat path to render(), so the device and
  // the browser cannot drift apart visually.
  void accumulate() {
    renderer_.setTimeOffset(interpolate_ ? accumulator_ : 0.0f);
    renderer_.accumulate(particles_, field_, geometry_);
  }

#if PARTSIM_INTERNAL_PIXELS
  void render() {
    renderer_.setTimeOffset(interpolate_ ? accumulator_ : 0.0f);
    renderer_.render(particles_, field_, geometry_);
  }
#endif

  void setInterpolate(bool on) { interpolate_ = on; }
  bool interpolate() const { return interpolate_; }

  const FieldGrid& field() const { return field_; }

  const Geometry& geometry() const { return geometry_; }
  const SimVolume& volume() const { return volume_; }
  const Particles& particles() const { return particles_; }
  const Solver& solver() const { return solver_; }
  Renderer& renderer() { return renderer_; }
  const Renderer& renderer() const { return renderer_; }

  int particleCount() const { return particles_.n; }
  int capacity() const { return solver_.capacity(volume_); }
  void setPalette(const Palette* p) { renderer_.setPalette(p); }

  // FNV-1a over position and velocity, folded to 32 bits. The cross-target determinism
  // check compares this between the host and the WASM build.
  uint32_t stateHash() const;
  // Same, over the heat field, so fire is covered by that check too.
  uint32_t fieldHash() const;

  Stats stats{0.0f, 0.0f, 0, 0};

 private:
  void fixedStep(float dt);
  int fill(int count, uint8_t material, uint32_t seed);
  void applySceneTargets(int sceneId);
  void advanceTransition(float dt);
  void countMaterials(int& water, int& sand) const;
  bool populationReached() const;
  void spawnOne(uint8_t material);

  Geometry geometry_;
  SimVolume volume_;
  Particles particles_;
  SpatialHash hash_;
  Solver solver_;
  FieldGrid field_;
  Renderer renderer_;
  float scratch_[kMaxParticles];

  Emitter emitters_[kMaxEmitters];
  int emitterCount_ = 0;
  int sceneId_ = 0;
  int targetWater_ = 0;
  int targetSand_ = 0;
  const Palette* fadeFrom_ = nullptr;
  const Palette* fadeTo_ = nullptr;
  float fade_ = 1.0f;  // 1 == not fading
  bool autoCycle_ = false;
  float cycleClock_ = 0.0f;
  Rng rng_{0x5EEDu};  // field flicker; seeded, never clock-based

  Vec3 gravity_{0.0f, -kGravityMag, 0.0f};
  Vec3 jerk_{0.0f, 0.0f, 0.0f};  // container acceleration; see addContainerAccel

  float accumulator_ = 0.0f;
  bool interpolate_ = true;
};

// A fixed scripted motion sequence, run identically on every target so the resulting hash can
// be compared bit-for-bit. Lives in core rather than in each test harness precisely so the two
// harnesses cannot drift apart.
//
// Runs two scenes on purpose: water+sand first, then the kettle. A water-only sequence leaves
// the friction pass and the whole heat field uncovered, so a divergence in either would pass
// the check unnoticed.
uint32_t goldenHash(Simulation& sim, int steps, uint32_t seed);
constexpr int kGoldenSteps = 500;   // per scene
constexpr uint32_t kGoldenSeed = 0xC0FFEEu;
constexpr int kGoldenSceneA = 3;    // water and sand
constexpr int kGoldenSceneB = 4;    // kettle: water over a burner

}  // namespace partsim
