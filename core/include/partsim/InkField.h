#pragma once
#include "partsim/Config.h"
#include "partsim/Rng.h"
#include "partsim/SimVolume.h"

namespace partsim {

// Coloured dye dispersing through a clear carrier liquid, as a small fixed-point scalar field.
//
// The carrier liquid is not simulated. An empty cell means clear water, not empty space, and that
// is the entire saving: PBF spends its time on neighbour gathering, incompressibility and wall
// compensation -- properties of the water the viewer never sees -- and its cost grows as n^1.77
// with how full the vessel is. A full 16^3 field and an empty one cost the same.
//
// Measured, not assumed: the advection step below runs 5.9-14.4 ms on an ESP32-S3 against the
// 732 ms/step a physically full beaker would need at d=2.5. See docs/DESIGN-SUGGESTIONS.md.
//
// This sits BESIDE the PBF solver rather than replacing it. Sand has a surface and grains that
// touch, so it keeps its particles; dye has neither.
class InkField {
 public:
  // A flow primitive: a decaying ball of rotation. No neighbours, no collision, no pressure.
  //
  // Inside its radius it contributes a tangential velocity proportional to
  // cross(axis, position - centre), falling off to exactly zero at the radius so the support
  // stays compact and a cell outside it costs one compare.
  struct Vorton {
    int32_t cx, cy, cz;  // centre, Q8.8 in GRID coordinates (not world)
    int32_t r2max;       // radius^2, Q8.8 in cells -- the support test, one compare per cell
    // (1<<24)/r2max, so the falloff index is (r2 * invR2) >> 16 with no divide in the cell loop.
    // Stored rather than derived because r2max is fixed for the vorton's whole life.
    uint32_t invR2;
    int8_t ax, ay, az;  // axis, Q0.7
    int16_t strength;   // Q8.8, signed so the two halves of a pair counter-rotate
    uint16_t life;      // remaining updates; 0 means the slot is free
  };

  bool init(const SimVolume& v);
  void clear();

  // Seeds the vorton axes once. Their phase then evolves continuously -- re-randomising a
  // direction every step makes the lateral displacement white noise, which averages to a
  // perfectly straight column. FieldGrid.cpp learned this the same way.
  void seedFlow(Rng& rng);

  // Adds dye mass to `channel` in a ball around an object-space point. Masses, not hue ratios:
  // opacity is the channel sum and colour is their ratio, so red into blue becomes magenta
  // without storing RGB per cell.
  void inject(Vec3 objectPos, float radius, int channel, uint8_t amount);

  // Creates a counter-rotating pair, which is what produces the curled head at the front of a
  // plume rather than a spreading blob. `strength` is Q8.8 and may be negative.
  void spawnVortonPair(Vec3 objectPos, Vec3 axisObject, float radius, int strength, int life);

  // One field update. Integer-only in the cell loop: no divide, no square root, no transcendental.
  //
  // `gravityObject` sets the settling direction and `containerAccel` adds a bulk impulse; both are
  // object-space, exactly as the solver takes them. dtMillis is integer so that a fixed cadence is
  // bit-reproducible -- the field is part of the deterministic path.
  void step(Vec3 gravityObject, Vec3 containerAccel, int dtMillis);

  IVec3 dim() const { return IVec3{kInkDim, kInkDim, kInkDim}; }
  int cellCount() const { return kInkCells; }
  float cellSize() const { return cell_; }
  Vec3 origin() const { return lo_; }
  bool empty() const { return peak_ == 0; }
  uint8_t peak() const { return peak_; }

  // Read-only access to one dye channel's current buffer. Exactly cellCount() bytes.
  const uint8_t* channel(int c) const { return dye_[cur_][c]; }
  uint8_t at(int c, int x, int y, int z) const {
    return dye_[cur_][c][(z * kInkDim + y) * kInkDim + x];
  }

  // The active box, in cells, that step() actually touched. Early in a pour most of the volume is
  // clear and costs nothing.
  IVec3 activeLo() const { return IVec3{loX_, loY_, loZ_}; }
  IVec3 activeHi() const { return IVec3{hiX_, hiY_, hiZ_}; }

  int vortonCount() const;

  // Read-only view for the renderer, so the projection is identical whether the dye was advected
  // here or arrived over a wire. Defined in RenderState.cpp, as FieldGrid::view() is.
  struct InkView view() const;

  // Forces a full-grid update instead of the tracked active box. Exists for the test that asserts
  // the two agree bit for bit -- an active-bounds bug is otherwise invisible until a tendril is
  // silently clipped.
  void setForceFullUpdate(bool on) { forceFull_ = on; }

 private:
  void rebuildBounds();
  // Velocity of the flow at one Q8.8 grid point, in Q8.8 cells per second.
  void velocityAt(int px, int py, int pz, int32_t& vx, int32_t& vy, int32_t& vz) const;

  uint8_t dye_[2][kInkChannels][kInkCells];
  int cur_ = 0;

  Vorton vort_[kMaxVortons];
  // Falloff indexed by normalised squared distance, and a sine for the curl modes. Both are built
  // at init from integer arithmetic and partsim::fsin, so nothing here reaches for libm.
  int16_t falloff_[256];
  int16_t sin_[256];

  Vec3 lo_{0.0f, 0.0f, 0.0f};
  float cell_ = 1.0f;
  float invCell_ = 1.0f;

  // Settling and bulk drift, Q8.8 cells per second, recomputed once per step outside the loop.
  int32_t driftX_ = 0, driftY_ = 0, driftZ_ = 0;
  uint32_t phase_ = 0;  // curl-mode phase; advances with dt, so it is a pure function of the tick

  int loX_ = 0, loY_ = 0, loZ_ = 0;
  int hiX_ = -1, hiY_ = -1, hiZ_ = -1;  // empty box
  uint8_t peak_ = 0;
  bool forceFull_ = false;
};

}  // namespace partsim
