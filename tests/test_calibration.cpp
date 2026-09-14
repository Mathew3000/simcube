// MINI.md M3: the orientation pattern is derived from Geometry rather than a per-face table, and
// the whole point of that is that it is checkable. Two claims, and they are opposites the same way
// the beaker spill tests are (test_beaker_spill.cpp): a calibration pattern that always agrees is
// as useless as one that never does.
#include "check.h"
#include "partsim/Calibration.h"

using namespace partsim;

namespace {

Geometry cube() { return Geometry::cube(8, 4.0f); }

bool sameColour(Rgb8 a, Rgb8 b) { return a.r == b.r && a.g == b.g && a.b == b.b; }

// The logical corner texel of `panel` nearest world point `target`, and how close it got --
// callers use the distance to decide whether this panel actually touches that corner at all.
struct CornerHit {
  int i, j;
  float dist2;
};

CornerHit nearestCorner(const Panel& p, Vec3 target) {
  const int ci[4] = {0, 0, (int)p.w - 1, (int)p.w - 1};
  const int cj[4] = {0, (int)p.h - 1, 0, (int)p.h - 1};
  CornerHit best{0, 0, 1e30f};
  for (int k = 0; k < 4; ++k) {
    const Vec3 w = texelCenter(p, ci[k], cj[k]);
    const float d2 = length2(w - target);
    if (d2 < best.dist2) best = CornerHit{ci[k], cj[k], d2};
  }
  return best;
}

// Every panel whose corner texel actually sits at this global box corner -- three, for a closed
// cube. `panels`/`is`/`js` are sized for the worst case (every panel qualifying) so a bug that
// breaks the "only three" property is visible rather than silently truncated.
int panelsAtCorner(const Geometry& g, Vec3 sign, int* panels, int* is, int* js) {
  const Aabb box = g.bounds(kSlabDepth);
  const Vec3 center = box.center();
  const Vec3 half = box.size() * 0.5f;
  const Vec3 target = center + Vec3{sign.x * half.x, sign.y * half.y, sign.z * half.z};
  // A panel actually at this corner lands within half a texel-diagonal of it; one that merely
  // shares the axis (e.g. the opposite face along one axis) is a whole box-width away, so this
  // threshold cannot mistake one for the other.
  int n = 0;
  for (int p = 0; p < g.count(); ++p) {
    const Panel& pan = g.at(p);
    const CornerHit hit = nearestCorner(pan, target);
    const float threshold = length2(pan.u) + length2(pan.v);  // (pitch*sqrt2)^2, generous
    if (hit.dist2 < threshold) {
      panels[n] = p;
      is[n] = hit.i;
      js[n] = hit.j;
      ++n;
    }
  }
  return n;
}

// Same transform ChainMap::map applies for a square panel's `rotate`, kept local rather than
// depending on ChainMap: this test is about calibrationTexel's own contract, not the chain.
void rotateCorner(int w, int h, int i, int j, int rotate, int& oi, int& oj) {
  switch (rotate & 3) {
    case 0: oi = i; oj = j; break;
    case 1: oi = h - 1 - j; oj = i; break;
    case 2: oi = w - 1 - i; oj = h - 1 - j; break;
    default: oi = j; oj = w - 1 - i; break;
  }
}

}  // namespace

TEST(calibration_three_faces_at_every_corner_agree) {
  const Geometry g = cube();
  const float s[2] = {-1.0f, 1.0f};
  int found = 0;
  for (int xi = 0; xi < 2; ++xi)
    for (int yi = 0; yi < 2; ++yi)
      for (int zi = 0; zi < 2; ++zi) {
        int panels[8], is[8], js[8];
        const int n = panelsAtCorner(g, Vec3{s[xi], s[yi], s[zi]}, panels, is, js);
        CHECK(n == 3);  // exactly three faces meet at every corner of a closed cube
        ++found;
        const Rgb8 c0 = calibrationTexel(g, panels[0], is[0], js[0]);
        for (int k = 1; k < n; ++k) {
          const Rgb8 ck = calibrationTexel(g, panels[k], is[k], js[k]);
          CHECK(sameColour(c0, ck));
        }
      }
  CHECK(found == 8);
}

TEST(calibration_detects_a_mis_rotated_face) {
  // Not a proof for every corner/rotation combination -- one concrete case, checked directly,
  // which is the standard this project holds a calibration pattern to: it must be able to FAIL.
  const Geometry g = cube();
  int panels[8], is[8], js[8];
  const int n = panelsAtCorner(g, Vec3{1.0f, 1.0f, 1.0f}, panels, is, js);
  CHECK(n == 3);

  const int panel = panels[0];
  const Panel& p = g.at(panel);
  const Rgb8 correct = calibrationTexel(g, panel, is[0], js[0]);

  for (int rotate = 1; rotate <= 3; ++rotate) {
    int oi, oj;
    // The texel that would occupy this physical corner if `panel` were mounted with this
    // rotation instead of the identity: see rotateCorner's inverse-rotation comment above.
    rotateCorner((int)p.w, (int)p.h, is[0], js[0], (4 - rotate) & 3, oi, oj);
    const Rgb8 wrong = calibrationTexel(g, panel, oi, oj);
    CHECK(!sameColour(correct, wrong));
  }
}
