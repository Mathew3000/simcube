#include "partsim/ChainMap.h"

namespace partsim {

void ChainMap::defaultMounts(int count, FaceMount* out) {
  for (int k = 0; k < count; ++k) out[k] = FaceMount{(uint8_t)k, 0, 0};
}

bool ChainMap::validate(const FaceMount* mounts, int count) const {
  if (count <= 0 || count > kMaxPanels) return false;
  uint32_t used = 0;
  for (int k = 0; k < count; ++k) {
    const FaceMount& m = mounts[k];
    if (m.slot >= (uint8_t)count) return false;
    if (used & (1u << m.slot)) return false;  // two faces in one chain slot
    used |= 1u << m.slot;
    if (m.rotate > 3) return false;
    if (m.mirror > 1) return false;
    // A quarter turn transposes the panel; on a non-square panel that would not fit the slot.
    if ((m.rotate & 1) && w_[k] != h_[k]) return false;
  }
  return true;
}

bool ChainMap::init(const Geometry& g, const FaceMount* mounts, int count) {
  if (count != g.count()) return false;
  if (count <= 0 || count > kMaxPanels) return false;

  // A daisy chain is one canvas of uniform tiles. Mixed panel sizes would need a per-slot x
  // offset table, and no arrangement we build wants one -- so reject rather than pretend.
  for (int k = 0; k < count; ++k) {
    w_[k] = g.at(k).w;
    h_[k] = g.at(k).h;
    if (w_[k] != g.at(0).w || h_[k] != g.at(0).h) return false;
  }
  if (!validate(mounts, count)) return false;

  for (int k = 0; k < count; ++k) mounts_[k] = mounts[k];
  count_ = count;
  chainW_ = (int)w_[0] * count;
  chainH_ = (int)h_[0];
  return true;
}

bool ChainMap::setMount(int panel, FaceMount m) {
  if (panel < 0 || panel >= count_) return false;
  FaceMount trial[kMaxPanels];
  for (int k = 0; k < count_; ++k) trial[k] = mounts_[k];
  trial[panel] = m;
  if (!validate(trial, count_)) return false;
  mounts_[panel] = m;
  return true;
}

void ChainMap::map(int panel, int i, int j, int& cx, int& cy) const {
  const FaceMount& m = mounts_[panel];
  const int w = (int)w_[panel], h = (int)h_[panel];

  int dx, dy;
  switch (m.rotate) {
    case 0:  dx = i;         dy = j;         break;
    case 1:  dx = h - 1 - j; dy = i;         break;
    case 2:  dx = w - 1 - i; dy = h - 1 - j; break;
    default: dx = j;         dy = w - 1 - i; break;
  }
  if (m.mirror) dx = w - 1 - dx;

  // Renderer row 0 is the BOTTOM row (it shares the convention with a GL texture); a HUB75
  // panel scans from the TOP. Exactly the flip that, when forgotten, puts the water on the
  // ceiling -- which is how it was found in the browser canvas page during Milestone 1.
  cx = (int)m.slot * w + dx;
  cy = h - 1 - dy;
}

ChainRun ChainMap::row(int panel, int j) const {
  ChainRun r{0, 0, 1, 0, (int)w_[panel]};
  map(panel, 0, j, r.cx, r.cy);
  if (r.count >= 2) {
    int x1, y1;
    map(panel, 1, j, x1, y1);
    r.dx = x1 - r.cx;
    r.dy = y1 - r.cy;
  }
  return r;
}

}  // namespace partsim
