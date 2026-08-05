#include "check.h"
#include "partsim/Geometry.h"
#include "partsim/SimVolume.h"

using namespace partsim;

TEST(geometry_cube_has_six_panels) {
  const Geometry g = Geometry::cube(32, 1.0f);
  CHECK(g.count() == 6);
  CHECK(g.totalTexels() == 6 * 32 * 32);
}

TEST(geometry_cube_normals_all_point_inward) {
  const Geometry g = Geometry::cube(32, 1.0f);
  for (int i = 0; i < g.count(); ++i) {
    const Panel& p = g.at(i);
    // The panel's face centre is origin + w/2*u + h/2*v; the inward normal must point
    // from there back toward the cube centre (which is the world origin).
    const Vec3 faceCenter =
        p.origin + p.u * (0.5f * (float)p.w) + p.v * (0.5f * (float)p.h);
    CHECK(dot(p.n, Vec3{0, 0, 0} - faceCenter) > 0.0f);
    CHECK_NEAR(length(p.n), 1.0f, 1e-6);
  }
}

TEST(geometry_basis_is_right_handed_and_orthogonal) {
  const Geometry g = Geometry::cube(32, 1.0f);
  for (int i = 0; i < g.count(); ++i) {
    const Panel& p = g.at(i);
    CHECK_NEAR(dot(p.u, p.v), 0.0f, 1e-5);
    CHECK_NEAR(length(p.u), 1.0f, 1e-6);
    CHECK_NEAR(length(p.v), 1.0f, 1e-6);
    // cross(u, v) == n is the invariant that makes `project`'s dist sign meaningful.
    const Vec3 c = normalize(cross(p.u, p.v));
    CHECK_NEAR(c.x, p.n.x, 1e-5);
    CHECK_NEAR(c.y, p.n.y, 1e-5);
    CHECK_NEAR(c.z, p.n.z, 1e-5);
  }
}

TEST(geometry_outside_observer_convention_on_minus_z_face) {
  // Panel 0 of the cube is the -Z face (plane z = -16, inward normal +Z).
  // An observer OUTSIDE at -Z looking toward +Z with up = +Y has their right hand at +X.
  // So the panel's +x step must be +X and its +y step must be +Y.
  const Geometry g = Geometry::cube(32, 1.0f);
  const Panel& p = g.at(0);
  CHECK_NEAR(p.n.z, 1.0f, 1e-6);
  CHECK_NEAR(p.u.x, 1.0f, 1e-6);
  CHECK_NEAR(p.u.y, 0.0f, 1e-6);
  CHECK_NEAR(p.v.y, 1.0f, 1e-6);
  CHECK_NEAR(p.v.x, 0.0f, 1e-6);
  // Texel (0,0) is the bottom-left corner as seen from outside.
  CHECK_NEAR(p.origin.x, -16.0f, 1e-6);
  CHECK_NEAR(p.origin.y, -16.0f, 1e-6);
  CHECK_NEAR(p.origin.z, -16.0f, 1e-6);
}

TEST(geometry_texel_world_roundtrip) {
  const Geometry g = Geometry::cube(32, 1.0f);
  for (int k = 0; k < g.count(); ++k) {
    const Panel& p = g.at(k);
    for (int j = 0; j < p.h; j += 7) {
      for (int i = 0; i < p.w; i += 5) {
        const Proj r = project(p, texelCenter(p, i, j));
        CHECK_NEAR(r.s, (float)i + 0.5f, 1e-4);
        CHECK_NEAR(r.t, (float)j + 0.5f, 1e-4);
        CHECK_NEAR(r.dist, 0.0f, 1e-4);
      }
    }
  }
}

TEST(geometry_project_dist_is_positive_inside) {
  const Geometry g = Geometry::cube(32, 1.0f);
  for (int k = 0; k < g.count(); ++k) {
    const Panel& p = g.at(k);
    // The cube centre is 16 units inward from every face.
    const Proj r = project(p, Vec3{0, 0, 0});
    CHECK_NEAR(r.dist, 16.0f, 1e-4);
    CHECK_NEAR(r.s, 16.0f, 1e-4);
    CHECK_NEAR(r.t, 16.0f, 1e-4);
  }
}

TEST(geometry_non_unit_pitch_scales_correctly) {
  const Geometry g = Geometry::cube(16, 2.0f);  // 16 texels at pitch 2 == 32 units
  const Panel& p = g.at(0);
  CHECK_NEAR(length(p.u), 2.0f, 1e-6);
  CHECK_NEAR(p.origin.x, -16.0f, 1e-6);
  const Proj r = project(p, texelCenter(p, 3, 4));
  CHECK_NEAR(r.s, 3.5f, 1e-4);
  CHECK_NEAR(r.t, 4.5f, 1e-4);
}

TEST(geometry_rejects_up_parallel_to_normal) {
  Geometry g;
  CHECK(!g.addPanel(PanelSpec{Vec3{0, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 1, 0}, 32, 32, 1.0f}));
  CHECK(g.count() == 0);
  // A slightly-off `up` is silently orthogonalised rather than producing a skewed basis.
  CHECK(g.addPanel(PanelSpec{Vec3{0, 0, 0}, Vec3{0, 0, 1}, Vec3{0.1f, 1, 0}, 32, 32, 1.0f}));
  CHECK_NEAR(dot(g.at(0).u, g.at(0).v), 0.0f, 1e-6);
}

TEST(geometry_rejects_overfull_table) {
  Geometry g;
  int added = 0;
  for (int i = 0; i < kMaxPanels + 4; ++i) {
    if (g.addPanel(PanelSpec{Vec3{0, 0, (float)i}, Vec3{0, 0, 1}, Vec3{0, 1, 0}, 8, 8, 1.0f}))
      ++added;
  }
  CHECK(added == kMaxPanels);
  CHECK(g.count() == kMaxPanels);
}

// --- SimVolume -------------------------------------------------------------

TEST(volume_cube_bounds_are_exactly_res_cubed) {
  const Geometry g = Geometry::cube(32, 1.0f);
  const Aabb b = g.bounds(kSlabDepth);
  CHECK_NEAR(b.size().x, 32.0f, 1e-4);
  CHECK_NEAR(b.size().y, 32.0f, 1e-4);
  CHECK_NEAR(b.size().z, 32.0f, 1e-4);
  CHECK_NEAR(b.center().x, 0.0f, 1e-4);
}

TEST(volume_slab_bounds_get_depth_from_the_inward_push) {
  const Geometry g = Geometry::slab(32, 32, 1.0f);
  const Aabb b = g.bounds(kSlabDepth);
  CHECK_NEAR(b.size().x, 32.0f, 1e-4);
  CHECK_NEAR(b.size().y, 32.0f, 1e-4);
  CHECK_NEAR(b.size().z, kSlabDepth, 1e-4);  // zero-thickness quad + inward push
}

TEST(volume_grid_dims_for_cube) {
  SimVolume v;
  CHECK(v.build(Geometry::cube(32, 1.0f), kSlabDepth, kCellSize));
  CHECK(v.dim().x == 11);  // ceil(32 / 3)
  CHECK(v.dim().y == 11);
  CHECK(v.dim().z == 11);
  CHECK(v.cellCount() == 11 * 11 * 11);
}

TEST(volume_grid_dims_for_slab) {
  SimVolume v;
  CHECK(v.build(Geometry::slab(32, 32, 1.0f), kSlabDepth, kCellSize));
  CHECK(v.dim().x == 11);
  CHECK(v.dim().y == 11);
  CHECK(v.dim().z == 2);  // 4.5 units of depth spans two 3-unit cells
  CHECK(v.cellCount() == 11 * 11 * 2);
}

TEST(volume_rejects_cell_smaller_than_smoothing_radius) {
  // This is the invariant whose violation looks like a solver bug rather than a search
  // bug: a too-small cell makes the 27-cell gather silently miss neighbours.
  SimVolume v;
  CHECK(!v.build(Geometry::cube(32, 1.0f), kSlabDepth, kSmoothRadius * 0.5f));
  CHECK(v.build(Geometry::cube(32, 1.0f), kSlabDepth, kSmoothRadius));
}

TEST(volume_coord_clamps_and_flattens_uniquely) {
  SimVolume v;
  CHECK(v.build(Geometry::cube(32, 1.0f), kSlabDepth, kCellSize));

  // Corners clamp into range rather than indexing out of bounds.
  CHECK(v.cellIndexOf(Vec3{-1000, -1000, -1000}) == 0);
  CHECK(v.cellIndexOf(Vec3{1000, 1000, 1000}) == v.cellCount() - 1);

  // flatten is injective over the whole grid.
  const IVec3 d = v.dim();
  bool seen[11 * 11 * 11] = {false};
  for (int z = 0; z < d.z; ++z)
    for (int y = 0; y < d.y; ++y)
      for (int x = 0; x < d.x; ++x) {
        const int f = v.flatten(IVec3{x, y, z});
        CHECK(f >= 0 && f < v.cellCount());
        CHECK(!seen[f]);
        seen[f] = true;
      }
}

TEST(volume_point_in_cell_maps_back_to_that_cell) {
  SimVolume v;
  CHECK(v.build(Geometry::cube(32, 1.0f), kSlabDepth, kCellSize));
  const Aabb b = v.box();
  const IVec3 d = v.dim();
  for (int z = 0; z < d.z; ++z)
    for (int y = 0; y < d.y; ++y)
      for (int x = 0; x < d.x; ++x) {
        // Nudge just inside the cell's low corner, then clamp to the box for the
        // partial cells at the far edge.
        const Vec3 p{pmin(b.lo.x + ((float)x + 0.1f) * v.cellSize(), b.hi.x - 1e-3f),
                     pmin(b.lo.y + ((float)y + 0.1f) * v.cellSize(), b.hi.y - 1e-3f),
                     pmin(b.lo.z + ((float)z + 0.1f) * v.cellSize(), b.hi.z - 1e-3f)};
        const IVec3 c = v.coordOf(p);
        CHECK(c.x <= x && c.y <= y && c.z <= z);
      }
}
