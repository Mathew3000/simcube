#include "check.h"
#include "partsim/ChainMap.h"

using namespace partsim;

namespace {

constexpr int kRes = 32;

Geometry cube() { return Geometry::cube(kRes, 1.0f); }

// The check that actually matters: every texel of every face must land on its own chain
// pixel, and together they must cover the chain exactly. Any duplicated slot, transposed
// rotation or off-by-one in the vertical flip breaks this.
void checkBijection(const ChainMap& cm) {
  static uint8_t hits[kRes * kRes * 8];
  const int total = cm.chainWidth() * cm.chainHeight();
  for (int i = 0; i < total; ++i) hits[i] = 0;

  for (int p = 0; p < cm.count(); ++p) {
    for (int j = 0; j < kRes; ++j) {
      for (int i = 0; i < kRes; ++i) {
        int cx = -1, cy = -1;
        cm.map(p, i, j, cx, cy);
        CHECK(cx >= 0 && cx < cm.chainWidth());
        CHECK(cy >= 0 && cy < cm.chainHeight());
        const int flat = cy * cm.chainWidth() + cx;
        CHECK(hits[flat] == 0);  // nothing may be written twice
        hits[flat] = 1;
      }
    }
  }
  for (int i = 0; i < total; ++i) CHECK(hits[i] == 1);  // nothing left dark
}

}  // namespace

TEST(chain_map_default_mounts_are_a_bijection) {
  FaceMount mounts[8];
  ChainMap::defaultMounts(6, mounts);
  ChainMap cm;
  CHECK(cm.init(cube(), mounts, 6));
  CHECK(cm.chainWidth() == 6 * kRes);
  CHECK(cm.chainHeight() == kRes);
  checkBijection(cm);
}

TEST(chain_map_stays_a_bijection_under_every_mount_combination) {
  // The realistic worst case: each face glued at a different angle, some mirrored, and the
  // chain order not matching the geometry order. This is what a real cube looks like.
  const FaceMount mounts[6] = {
      {3, 0, 0}, {0, 1, 0}, {5, 2, 1}, {1, 3, 0}, {4, 1, 1}, {2, 2, 0},
  };
  ChainMap cm;
  CHECK(cm.init(cube(), mounts, 6));
  checkBijection(cm);
}

TEST(chain_map_flips_rows_for_hub75_scan_order) {
  FaceMount mounts[8];
  ChainMap::defaultMounts(6, mounts);
  ChainMap cm;
  CHECK(cm.init(cube(), mounts, 6));

  // Renderer row 0 is the BOTTOM of the picture; a HUB75 panel scans from the top, so it must
  // come out as the LAST physical row. Forgetting this is the ceiling-water bug.
  int cx = 0, cy = 0;
  cm.map(0, 0, 0, cx, cy);
  CHECK(cx == 0);
  CHECK(cy == kRes - 1);

  cm.map(0, 5, kRes - 1, cx, cy);
  CHECK(cx == 5);
  CHECK(cy == 0);
}

TEST(chain_map_offsets_by_slot) {
  const FaceMount mounts[6] = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}};
  ChainMap cm;
  CHECK(cm.init(cube(), mounts, 6));
  for (int p = 0; p < 6; ++p) {
    int cx = 0, cy = 0;
    cm.map(p, 3, 4, cx, cy);
    CHECK(cx == p * kRes + 3);
    CHECK(cy == kRes - 1 - 4);
  }
}

TEST(chain_map_mirror_reflects_in_x_only) {
  const FaceMount m0[6] = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}};
  const FaceMount m1[6] = {{0, 0, 1}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}};
  ChainMap a, b;
  CHECK(a.init(cube(), m0, 6));
  CHECK(b.init(cube(), m1, 6));

  int ax = 0, ay = 0, bx = 0, by = 0;
  a.map(0, 2, 7, ax, ay);
  b.map(0, 2, 7, bx, by);
  CHECK(ay == by);                     // rows untouched
  CHECK(bx == kRes - 1 - ax);          // columns reflected within the slot
}

TEST(chain_map_rows_are_contiguous_only_for_even_rotations) {
  const FaceMount mounts[6] = {{0, 0, 0}, {1, 1, 0}, {2, 2, 0}, {3, 3, 0}, {4, 0, 1}, {5, 2, 1}};
  ChainMap cm;
  CHECK(cm.init(cube(), mounts, 6));

  // Rotations 0 and 2 map a renderer row to a horizontal run, so the driver can blit it with
  // a stride. Rotations 1 and 3 map it to a column and it has to go pixel by pixel -- the
  // cost of the mount table, made visible rather than buried in the blit loop.
  for (int p = 0; p < 6; ++p) {
    const ChainRun r = cm.row(p, 5);
    CHECK(r.count == kRes);
    const bool horizontal = (r.dy == 0);
    CHECK(horizontal == ((cm.mount(p).rotate & 1) == 0));
    CHECK((r.dx == 0) != horizontal);
    CHECK(pabs((float)(r.dx + r.dy)) == 1.0f);  // unit step either way

    // Whatever the direction, walking the run must agree with map() texel for texel.
    for (int i = 0; i < r.count; ++i) {
      int cx = 0, cy = 0;
      cm.map(p, i, 5, cx, cy);
      CHECK(cx == r.cx + r.dx * i);
      CHECK(cy == r.cy + r.dy * i);
    }
  }
}

TEST(chain_map_rejects_malformed_tables) {
  ChainMap cm;
  const Geometry g = cube();

  const FaceMount dupSlot[6] = {{0, 0, 0}, {0, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}};
  CHECK(!cm.init(g, dupSlot, 6));

  const FaceMount outOfRange[6] = {{9, 0, 0}, {1, 0, 0}, {2, 0, 0},
                                   {3, 0, 0}, {4, 0, 0}, {5, 0, 0}};
  CHECK(!cm.init(g, outOfRange, 6));

  const FaceMount badRot[6] = {{0, 7, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}};
  CHECK(!cm.init(g, badRot, 6));

  // Count must match the geometry, or faces would silently go unlit.
  FaceMount ok[8];
  ChainMap::defaultMounts(6, ok);
  CHECK(!cm.init(g, ok, 5));
}

TEST(chain_map_rejects_quarter_turns_on_non_square_panels) {
  // A slab is 32x32 here, so build something explicitly oblong to test the rule.
  Geometry g;
  PanelSpec s{Vec3{0.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, 1.0f}, Vec3{0.0f, 1.0f, 0.0f},
              64, 32, 1.0f};
  CHECK(g.addPanel(s));

  ChainMap cm;
  const FaceMount straight[1] = {{0, 0, 0}};
  CHECK(cm.init(g, straight, 1));
  CHECK(cm.chainWidth() == 64);

  // A quarter turn would transpose it to 32x64 and no longer fit its slot.
  const FaceMount turned[1] = {{0, 1, 0}};
  CHECK(!cm.init(g, turned, 1));
}

TEST(chain_map_live_remount_keeps_the_table_valid) {
  FaceMount mounts[8];
  ChainMap::defaultMounts(6, mounts);
  ChainMap cm;
  CHECK(cm.init(cube(), mounts, 6));

  // Calibrating against the built object: rotating one face is fine...
  CHECK(cm.setMount(2, FaceMount{2, 1, 0}));
  CHECK(cm.mount(2).rotate == 1);
  checkBijection(cm);

  // ...but moving it onto another face's slot would blank a face, so it must be refused and
  // leave the working table alone.
  CHECK(!cm.setMount(2, FaceMount{4, 1, 0}));
  CHECK(cm.mount(2).slot == 2);
  checkBijection(cm);

  // Swapping two slots is legal, and has to go through a spare slot the way the serial
  // console does it -- but the end state must still be a bijection.
  CHECK(cm.setMount(2, FaceMount{2, 3, 1}));
  checkBijection(cm);
}

// --- driving a subset of the faces ---------------------------------------------------------------
// A display node in a multi-node cube holds the whole six-panel table -- Geometry::bounds() derives
// the container from it and every node must agree on that container exactly -- but drives only its
// own faces. So the chain it owns is two tiles wide, not six.

TEST(chain_map_drives_a_subset_of_faces) {
  const Geometry g = cube();
  // This node owns geometry panels 2 and 3, in chain slots 0 and 1.
  const int panels[2] = {2, 3};
  const FaceMount mounts[2] = {{0, 0, 0}, {1, 2, 0}};

  ChainMap cm;
  CHECK(cm.init(g, mounts, panels, 2));
  CHECK(cm.count() == 2);
  CHECK(cm.chainWidth() == 2 * kRes);   // two tiles, not six
  CHECK(cm.chainHeight() == kRes);

  // Driven-face index -> geometry panel. The two only coincide when everything is driven, and
  // conflating them would blit the wrong face's pixels to the wrong physical panel.
  CHECK(cm.panelAt(0) == 2);
  CHECK(cm.panelAt(1) == 3);
  CHECK(cm.panelAt(2) == -1);

  // Still a bijection, over this node's own chain.
  static uint8_t hits[2 * kRes * kRes];
  for (int i = 0; i < 2 * kRes * kRes; ++i) hits[i] = 0;
  for (int f = 0; f < 2; ++f)
    for (int j = 0; j < kRes; ++j)
      for (int i = 0; i < kRes; ++i) {
        int cx = -1, cy = -1;
        cm.map(f, i, j, cx, cy);
        CHECK(cx >= 0 && cx < cm.chainWidth());
        CHECK(cy >= 0 && cy < cm.chainHeight());
        const int flat = cy * cm.chainWidth() + cx;
        CHECK(hits[flat] == 0);
        hits[flat] = 1;
      }
  for (int i = 0; i < 2 * kRes * kRes; ++i) CHECK(hits[i] == 1);
}

TEST(chain_map_rejects_a_bad_subset) {
  const Geometry g = cube();
  ChainMap cm;
  const FaceMount m2[2] = {{0, 0, 0}, {1, 0, 0}};

  const int twice[2] = {3, 3};      // the same face driven twice would be blitted twice
  CHECK(!cm.init(g, m2, twice, 2));
  const int outOfRange[2] = {0, 9};  // no such panel
  CHECK(!cm.init(g, m2, outOfRange, 2));
  const int tooMany[7] = {0, 1, 2, 3, 4, 5, 0};
  CHECK(!cm.init(g, m2, tooMany, 7));

  // ...and the all-faces form still refuses a count that would leave faces unlit. That check was
  // deliberate before the subset form existed; it keeps its meaning for the all-faces overload,
  // where naming fewer faces than exist really is a mistake rather than a subset.
  FaceMount ok[8];
  ChainMap::defaultMounts(6, ok);
  CHECK(!cm.init(g, ok, 5));
}
