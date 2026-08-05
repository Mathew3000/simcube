#include "check.h"
#include "partsim/Simulation.h"

using namespace partsim;

namespace {
Simulation g_sim;  // ~1.2MB

// Centre of heat along one axis, weighted by cell value. Returns -1 if nothing is burning.
float heatCentreY(const FieldGrid& f) {
  double sum = 0.0, wsum = 0.0;
  const IVec3 d = f.dim();
  for (int z = 0; z < d.z; ++z)
    for (int y = 0; y < d.y; ++y)
      for (int x = 0; x < d.x; ++x) {
        const double w = f.atCoord(x, y, z);
        sum += w * f.cellCentre(x, y, z).y;
        wsum += w;
      }
  return wsum > 0.0 ? (float)(sum / wsum) : -1.0f;
}

float heatCentreX(const FieldGrid& f) {
  double sum = 0.0, wsum = 0.0;
  const IVec3 d = f.dim();
  for (int z = 0; z < d.z; ++z)
    for (int y = 0; y < d.y; ++y)
      for (int x = 0; x < d.x; ++x) {
        const double w = f.atCoord(x, y, z);
        sum += w * f.cellCentre(x, y, z).x;
        wsum += w;
      }
  return wsum > 0.0 ? (float)(sum / wsum) : -1e30f;
}

int totalHeat(const FieldGrid& f) {
  int t = 0;
  for (int i = 0; i < f.cellCount(); ++i) t += f.at(i);
  return t;
}

// Scene 1 is the campfire: one emitter on the floor, no particles.
void campfire(int steps) {
  g_sim.initScene(Simulation::kCube, 1, 7);
  for (int s = 0; s < steps; ++s) g_sim.stepFixed();
}
}  // namespace

TEST(field_stays_empty_without_emitters) {
  // Water-only scenes must pay nothing for fire existing: the advection pass exits immediately.
  g_sim.initScene(Simulation::kCube, 0, 1);  // water tank
  for (int s = 0; s < 60; ++s) g_sim.stepFixed();
  CHECK(g_sim.field().empty());
  CHECK(totalHeat(g_sim.field()) == 0);
}

TEST(field_emitter_lights_and_rises) {
  campfire(120);
  const FieldGrid& f = g_sim.field();
  CHECK(!f.empty());
  CHECK(totalHeat(f) > 0);

  // The emitter sits at 6% of the box height; heat must end up well above it.
  const float floorY = g_sim.volume().box().lo.y;
  const float emitterY = floorY + 0.06f * g_sim.volume().box().size().y;
  const float centre = heatCentreY(f);
  std::printf("       emitter y %.1f, heat centre y %.1f\n", emitterY, centre);
  CHECK(centre > emitterY + 2.0f);
}

TEST(field_cools_so_heat_does_not_fill_the_box) {
  // Cooling is the only sink in a closed box. Without enough of it, heat piles against the
  // ceiling and the top face ends up as bright as the fire itself.
  campfire(400);
  const FieldGrid& f = g_sim.field();
  const IVec3 d = f.dim();

  // Compare the bottom fifth against the top fifth.
  int lowSum = 0, highSum = 0;
  for (int z = 0; z < d.z; ++z)
    for (int x = 0; x < d.x; ++x) {
      for (int y = 0; y < d.y / 5; ++y) lowSum += f.atCoord(x, y, z);
      for (int y = d.y - d.y / 5; y < d.y; ++y) highSum += f.atCoord(x, y, z);
    }
  std::printf("       heat low %d vs high %d\n", lowSum, highSum);
  CHECK(lowSum > highSum);  // brightest at the base, as a flame should be
}

TEST(field_rises_opposite_gravity_not_merely_upward) {
  // Buoyancy is defined against the object-space gravity vector, so a tilted cube must make the
  // flame lean. This is what makes "fire reacts to motion" work with no extra plumbing.
  g_sim.initScene(Simulation::kCube, 1, 7);
  // Gravity pulled toward -x as well as -y: the plume should drift toward +x.
  g_sim.setGravityObject(normalize(Vec3{-1.0f, -1.0f, 0.0f}) * kGravityMag);
  for (int s = 0; s < 200; ++s) g_sim.stepFixed();

  const float cx = heatCentreX(g_sim.field());
  const float emitterX = g_sim.volume().box().lo.x + 0.5f * g_sim.volume().box().size().x;
  std::printf("       emitter x %.1f, heat centre x %.1f\n", emitterX, cx);
  CHECK(cx > emitterX + 0.5f);  // leaned away from the gravity direction
}

TEST(field_container_shake_pushes_the_flame) {
  g_sim.initScene(Simulation::kCube, 1, 7);
  for (int s = 0; s < 150; ++s) g_sim.stepFixed();
  const float before = heatCentreX(g_sim.field());

  // Accelerate the container toward +x; the flame is pressed toward -x, the same inversion the
  // fluid gets.
  for (int s = 0; s < 40; ++s) {
    g_sim.addContainerAccel(Vec3{120.0f, 0.0f, 0.0f});
    g_sim.stepFixed();
  }
  const float after = heatCentreX(g_sim.field());
  std::printf("       heat centre x %.2f -> %.2f under +x shake\n", before, after);
  CHECK(after < before);
}

TEST(field_never_exceeds_a_byte_or_goes_negative) {
  campfire(500);
  const FieldGrid& f = g_sim.field();
  for (int i = 0; i < f.cellCount(); ++i) CHECK(f.at(i) <= 255);
  // sample() is normalised to 0..1 and must not overshoot at the hottest cell.
  const IVec3 d = f.dim();
  for (int y = 0; y < d.y; ++y) {
    const float s = f.sample(f.cellCentre(d.x / 2, y, d.z / 2));
    CHECK(s >= 0.0f && s <= 1.001f);
  }
}

TEST(field_is_deterministic) {
  campfire(200);
  const int a = totalHeat(g_sim.field());
  campfire(200);
  const int b = totalHeat(g_sim.field());
  CHECK(a == b);
  CHECK(a > 0);
}

TEST(field_dies_out_when_the_emitter_is_removed) {
  campfire(200);
  CHECK(!g_sim.field().empty());
  const int burning = totalHeat(g_sim.field());

  // Switching to a water scene clears the emitters; the heat must decay away rather than linger.
  g_sim.setScene(0);
  for (int s = 0; s < 120; ++s) g_sim.stepFixed();
  std::printf("       heat %d -> %d after switching scenes\n", burning, totalHeat(g_sim.field()));
  CHECK(totalHeat(g_sim.field()) == 0);
}

// --- scenes ----------------------------------------------------------------

TEST(scene_table_is_sane) {
  CHECK(sceneCount() >= 5);
  for (int i = 0; i < sceneCount(); ++i) {
    const SceneDesc& s = sceneAt(i);
    CHECK(s.name != nullptr);
    CHECK(s.emitterCount >= 0 && s.emitterCount <= kMaxEmitters);
    CHECK(s.dwellSeconds > 1.0f);
    CHECK(s.paletteIndex >= 0 && s.paletteIndex < paletteCount());
    CHECK(s.waterCount + s.sandCount + s.emitterCount > 0);  // no empty scenes
    for (int e = 0; e < s.emitterCount; ++e) {
      CHECK(s.emitters[e].nx >= 0.0f && s.emitters[e].nx <= 1.0f);
      CHECK(s.emitters[e].ny >= 0.0f && s.emitters[e].ny <= 1.0f);
      CHECK(s.emitters[e].radius > 0.0f);
      CHECK(s.emitters[e].rate > 0.0f);
    }
  }
  // Out-of-range indices clamp rather than reading past the table.
  CHECK(&sceneAt(-5) == &sceneAt(0));
  CHECK(&sceneAt(9999) == &sceneAt(sceneCount() - 1));
}

TEST(scene_loads_the_materials_it_declares) {
  for (int i = 0; i < sceneCount(); ++i) {
    CHECK(g_sim.initScene(Simulation::kCube, i, 3));
    const SceneDesc& sc = sceneAt(i);

    int water = 0, sand = 0;
    for (int k = 0; k < g_sim.particles().n; ++k)
      (g_sim.particles().mat[k] == kSand ? sand : water)++;

    if (sc.sandCount == 0) CHECK(sand == 0);
    if (sc.waterCount == 0) CHECK(water == 0);
    if (sc.sandCount > 0) CHECK(sand > 0);
    if (sc.waterCount > 0) CHECK(water > 0);
    CHECK(g_sim.particleCount() <= g_sim.capacity());
  }
}

TEST(scene_switch_replaces_rather_than_accumulates) {
  CHECK(g_sim.initScene(Simulation::kCube, 3, 5));  // water and sand
  const int mixed = g_sim.particleCount();
  CHECK(mixed > 0);

  g_sim.setScene(1);  // campfire: no particles at all
  CHECK(g_sim.particleCount() == 0);

  g_sim.setScene(3);
  CHECK(g_sim.particleCount() == mixed);  // and back, with no drift
}

TEST(scene_auto_cycle_advances_and_wraps) {
  CHECK(g_sim.initScene(Simulation::kCube, 0, 9));
  g_sim.setAutoCycle(true);
  CHECK(g_sim.scene() == 0);

  // Long enough for the first scene's dwell to elapse, driven through advance() so the cycle
  // clock is fed by real substeps.
  const float dwell = sceneAt(0).dwellSeconds;
  int guard = 0;
  while (g_sim.scene() == 0 && guard++ < 4000) g_sim.advance(kFixedDt);
  CHECK(g_sim.scene() == 1);
  std::printf("       advanced after ~%.1f s (dwell %.1f)\n", (float)guard * kFixedDt, dwell);

  // Walk all the way round and confirm it wraps to 0 rather than running off the table.
  int seen = 1;
  guard = 0;
  while (seen < sceneCount() && guard++ < 60000) {
    const int before = g_sim.scene();
    g_sim.advance(kFixedDt);
    if (g_sim.scene() != before) ++seen;
  }
  guard = 0;
  while (g_sim.scene() != 0 && guard++ < 60000) g_sim.advance(kFixedDt);
  CHECK(g_sim.scene() == 0);
}

TEST(scene_auto_cycle_off_by_default) {
  CHECK(g_sim.initScene(Simulation::kCube, 0, 9));
  CHECK(!g_sim.autoCycle());
  for (int s = 0; s < 2000; ++s) g_sim.advance(kFixedDt);
  CHECK(g_sim.scene() == 0);  // stays put
}
