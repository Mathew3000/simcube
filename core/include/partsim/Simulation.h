#pragma once
#include "partsim/Renderer.h"
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
  void addContainerAccelWorld(Quat q, Vec3 a) { jerk_ += rotateByConjugate(q, a); }
  Vec3 containerAccel() const { return jerk_; }

  // Fixed-timestep accumulator. Returns how many physics steps ran.
  int advance(float wallDt);
  // Exactly one physics step, for tests and the golden sequence.
  void stepFixed() { fixedStep(kFixedDt); }

  void render() { renderer_.render(particles_, geometry_); }

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

  Stats stats{0.0f, 0.0f, 0, 0};

 private:
  void fixedStep(float dt);
  int fill(int count, uint8_t material, uint32_t seed);

  Geometry geometry_;
  SimVolume volume_;
  Particles particles_;
  SpatialHash hash_;
  Solver solver_;
  Renderer renderer_;
  float scratch_[kMaxParticles];

  Vec3 gravity_{0.0f, -kGravityMag, 0.0f};
  Vec3 jerk_{0.0f, 0.0f, 0.0f};  // container acceleration; see addContainerAccel

  float accumulator_ = 0.0f;
};

// A fixed scripted motion sequence, run identically on every target so the resulting state
// hash can be compared bit-for-bit. Lives in core rather than in each test harness precisely
// so the two harnesses cannot drift apart.
uint32_t goldenHash(Simulation& sim, int steps, int particleCount, uint32_t seed);
constexpr int kGoldenSteps = 600;
constexpr int kGoldenParticles = 1200;
constexpr uint32_t kGoldenSeed = 0xC0FFEEu;

}  // namespace partsim
