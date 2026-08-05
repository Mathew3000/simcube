// Host-side inspector / benchmark. Grows with the project; right now it dumps the baked
// panel table and derived volume so the geometry can be eyeballed, which is the step-2
// verification.
#include <cstdio>

#include "partsim/Geometry.h"
#include "partsim/SimVolume.h"

using namespace partsim;

namespace {

const char* faceName(int i) {
  static const char* names[6] = {"-Z", "+Z", "-X", "+X", "-Y bot", "+Y top"};
  return i < 6 ? names[i] : "?";
}

void dumpGeometry(const char* label, const Geometry& g, float slabDepth) {
  std::printf("\n=== %s: %d panel(s), %d texels ===\n", label, g.count(), g.totalTexels());
  std::printf("%-8s %-20s %-14s %-14s %-14s\n", "face", "origin", "u (right)", "v (up)",
              "n (inward)");
  for (int i = 0; i < g.count(); ++i) {
    const Panel& p = g.at(i);
    std::printf("%-8s (%6.1f %6.1f %6.1f) (%4.1f %4.1f %4.1f) (%4.1f %4.1f %4.1f) "
                "(%4.1f %4.1f %4.1f)  %dx%d\n",
                faceName(i), p.origin.x, p.origin.y, p.origin.z, p.u.x, p.u.y, p.u.z,
                p.v.x, p.v.y, p.v.z, p.n.x, p.n.y, p.n.z, (int)p.w, (int)p.h);
  }

  const Aabb b = g.bounds(slabDepth);
  std::printf("bounds  lo (%.1f %.1f %.1f)  hi (%.1f %.1f %.1f)  size (%.1f %.1f %.1f)\n",
              b.lo.x, b.lo.y, b.lo.z, b.hi.x, b.hi.y, b.hi.z, b.size().x, b.size().y,
              b.size().z);

  SimVolume v;
  if (!v.build(g, slabDepth, kCellSize)) {
    std::printf("volume  BUILD FAILED\n");
    return;
  }
  std::printf("volume  grid %dx%dx%d = %d cells @ cell %.1f (h = %.1f, d = %.1f)\n",
              v.dim().x, v.dim().y, v.dim().z, v.cellCount(), v.cellSize(), kSmoothRadius,
              kRestSpacing);
}

}  // namespace

int main() {
  std::printf("partsim bench -- capacities: %d particles, %d panels, %d grid cells\n",
              kMaxParticles, kMaxPanels, kMaxGridCells);
  dumpGeometry("cube 32", Geometry::cube(32, 1.0f), 2.0f);
  dumpGeometry("single panel 32x32 (thin slab)", Geometry::slab(32, 32, 1.0f), 2.0f);
  return 0;
}
