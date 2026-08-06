#pragma once
#include "partsim/Config.h"
#include "partsim/Geometry.h"

namespace partsim {

// How one physical panel is mounted on the object.
//
// The renderer emits every panel in the same convention (+x right, +y up, seen from OUTSIDE
// -- see Geometry.h). The physical panels cannot all be glued that way: on a cube, four of
// them are rotated relative to whatever direction the ribbon cable happens to leave from, and
// which rotation each one got is a fact about the built object, not about the code.
//
// So it is a table. Getting it wrong shows up as a face whose contents are rotated or
// mirrored, which is obvious on sight and fixable without a rebuild (see the `mount` serial
// command in the firmware).
struct FaceMount {
  uint8_t slot;    // position in the daisy chain; panel `slot` owns chain x in [slot*w, +w)
  uint8_t rotate;  // quarter turns COUNTER-CLOCKWISE from renderer orientation to physical
  uint8_t mirror;  // 1 = mirror in x after rotating (panel glued facing the other way round)
};

// One horizontal run of a renderer row, expressed in chain pixels.
//
// Exists so the driver can push a whole row with a stride instead of calling drawPixel per
// texel (~130 cycles each on the S3, because every bitplane is a separate read-modify-write).
// It also makes the cost of the mount table visible: at rotate 0 or 2 a row maps to a
// contiguous horizontal run and can be blitted; at 1 or 3 it maps to a column and cannot.
struct ChainRun {
  int cx, cy;    // chain pixel for renderer texel (0, j) of this row
  int dx, dy;    // chain step per +1 texel in x; exactly one of these is 0
  int count;     // texels in the run
};

// Maps renderer panel texels onto the single wide canvas the HUB75 driver exposes.
//
// Two coordinate flips live here and nowhere else:
//   * the mount rotation/mirror above, and
//   * renderer row 0 is the BOTTOM row, while a HUB75 panel scans from the TOP.
// Both are pure integer arithmetic, so tests/test_chainmap.cpp can prove the whole mapping is
// a bijection onto the chain -- which is the check that catches a duplicated slot or a
// transposed rotation, the two ways this table goes wrong.
class ChainMap {
 public:
  // Returns false on a malformed table: a slot out of range, two faces sharing a slot,
  // rotate > 3, or an odd rotation applied to a non-square panel.
  bool init(const Geometry& g, const FaceMount* mounts, int count);

  int chainWidth() const { return chainW_; }
  int chainHeight() const { return chainH_; }
  int count() const { return count_; }
  const FaceMount& mount(int panel) const { return mounts_[panel]; }

  // Live re-mount, for calibrating against the built object over serial. Returns false if the
  // change would make the table invalid, leaving the old one in place.
  bool setMount(int panel, FaceMount m);

  // Renderer texel (i, j) of `panel` -> chain pixel. Bounds are the caller's business; this
  // is called per texel in the blit path.
  void map(int panel, int i, int j, int& cx, int& cy) const;

  ChainRun row(int panel, int j) const;

  // Straight-through mounts: face k in chain slot k, no rotation. The correct starting point
  // only because it is the arrangement to aim for when gluing -- expect to fix it up per face
  // once the real cube exists.
  static void defaultMounts(int count, FaceMount* out);

 private:
  bool validate(const FaceMount* mounts, int count) const;

  FaceMount mounts_[kMaxPanels] = {};
  uint16_t w_[kMaxPanels] = {};
  uint16_t h_[kMaxPanels] = {};
  int count_ = 0;
  int chainW_ = 0;
  int chainH_ = 0;
};

}  // namespace partsim
