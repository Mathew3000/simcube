#include "check.h"
#include "partsim/WaterEffects.h"
using namespace partsim;

TEST(dye_injection_is_local_and_uniform_advection_is_stable) {
  DyeField dye;
  SurfaceWater surface;
  surface.reset(0.6f);
  dye.reset({0.8f,0});
  for (int i = 0; i < 30; ++i) dye.step(1.0f/30.0f,surface,{0.2f,0,0});
  CHECK_NEAR(dye.sample({0,-0.2f,0}).red,0.8f,2e-4f);
  dye.inject({0,0,0},{0,1},0.02f);
  CHECK(dye.sample({0,0,0}).blue > 0.2f);
  CHECK_NEAR(dye.sample({-0.4f,-0.4f,-0.4f}).blue,0,1e-6f);
  dye.stir({0,0,0},{0,-1,0},4.0f);
  for (int i = 0; i < 30; ++i) dye.step(1.0f/30.0f,surface,{0,0,0});
  // The curling head need not pass through one particular point. Verify that
  // dye has travelled below its original injection sphere and stays localized.
  float belowPeak = 0;
  for (int z = 0; z < DyeField::kSize; ++z)
    for (int y = 0; y < 5; ++y)
      for (int x = 0; x < DyeField::kSize; ++x)
        belowPeak = pmax(belowPeak,dye.cell(x,y,z).blue);
  CHECK(belowPeak > 0.02f);
  CHECK(dye.sample({-0.4f,-0.4f,-0.4f}).blue < 0.005f);
  for (int z = 0; z < DyeField::kSize; ++z)
    for (int y = 0; y < DyeField::kSize; ++y)
      for (int x = 0; x < DyeField::kSize; ++x) {
        const auto d = dye.cell(x,y,z);
        CHECK(d.red >= 0 && d.blue >= 0 && d.red+d.blue <= 1.001f);
      }
}

TEST(effects_droplets_detach_from_motion_and_rejoin_without_volume_loss) {
  WaterEffects w;
  w.reset(0.55f,{0,0.9f});
  for (int i = 0; i < 30; ++i) w.step(1.0f/120.0f,{0,-1,0});
  CHECK(w.dropletCount() == 0);
  w.step(1.0f/120.0f,normalize({0.18f,-1,0}));
  CHECK(w.dropletCount() > 0);
  CHECK(w.surface().volume() < 0.55f);
  CHECK_NEAR(w.volume(),0.55f,2e-6f);
  for (int i = 0; i < 480; ++i) w.step(1.0f/120.0f,{0,-1,0});
  CHECK(w.dropletCount() == 0);
  CHECK_NEAR(w.volume(),0.55f,2e-6f);
  w.impulse({2,0,1});
  CHECK(w.dropletCount() > 0);
  CHECK_NEAR(w.volume(),0.55f,2e-6f);
}

TEST(effects_gentle_tilt_does_not_splash) {
  WaterEffects w;
  w.reset(0.5f,{0,0.9f});
  for (int i = 0; i < 120; ++i) {
    const float angle = (float)i*0.002f;
    w.step(1.0f/120.0f,{fsin(angle),-fcos(angle),0});
    CHECK(w.dropletCount() == 0);
  }
}

TEST(effects_pour_transfers_colour_locally_and_reserves_droplet_capacity) {
  WaterEffects a,b;
  a.reset(0.65f,{0,0.9f}); b.reset(0.35f,{0.9f,0});
  for (int i = 0; i < 100; ++i) {
    a.step(1.0f/120.0f,{1,0,0});
    b.step(1.0f/120.0f,{0,-1,0});
    a.pourTo(b,1.0f/120.0f);
    CHECK_NEAR(a.volume()+b.volume(),1.0f,5e-6f);
    CHECK(a.volume() <= 1.000001f && b.volume() <= 1.000001f);
  }
  CHECK(b.volume() > 0.35f);
  CHECK(b.dye().sample({-0.4f,-0.4f,-0.4f}).blue < 0.01f);
  float maxBlue = 0;
  for (int z = 0; z < DyeField::kSize; ++z)
    for (int y = 0; y < DyeField::kSize; ++y)
      for (int x = 0; x < DyeField::kSize; ++x)
        maxBlue = pmax(maxBlue,b.dye().cell(x,y,z).blue);
  CHECK(maxBlue > 0.1f);
  b.reset(0.9f,{0.9f,0});
  b.impulse({2,0,1});
  CHECK(b.dropletCount() > 0);
  const float total = a.volume()+b.volume();
  for (int i = 0; i < 200; ++i) a.pourTo(b,1.0f/120.0f);
  CHECK(b.volume() <= 1.000001f);
  CHECK_NEAR(a.volume()+b.volume(),total,5e-6f);
  for (int i = 0; i < 480; ++i) b.step(1.0f/120.0f,{0,-1,0});
  CHECK(b.dropletCount() == 0);
  CHECK_NEAR(a.volume()+b.volume(),total,5e-6f);
}

TEST(effects_renderer_keeps_bounds_and_draws_drops_in_air) {
  WaterEffects w;
  w.reset(0.45f,{0,0.9f});
  w.impulse({2,0,1});
  for (int i = 0; i < 12; ++i) w.step(1.0f/120.0f,{0,-1,0});
  CHECK(w.dropletCount() > 0);
  const int res = kMaxPanelTexels >= 4096 ? 64 : 32;
  const Geometry g = Geometry::cube(res,32.0f/(float)res);
  uint8_t pixels[64*64*4+8];
  int airPixels = 0;
  for (int face = 0; face < 6; ++face) {
    for (auto& p : pixels) p = 123;
    w.renderPanel(g.at(face),32.0f,pixels+4);
    for (int i = 0; i < 4; ++i) {
      CHECK(pixels[i] == 123); CHECK(pixels[res*res*4+4+i] == 123);
    }
    for (int y = 0; y < res; ++y) for (int x = 0; x < res; ++x) {
      const Vec3 p = texelCenter(g.at(face),x,y)*(1.0f/32.0f);
      if (w.surface().depth(p) < -0.04f && pixels[4+(y*res+x)*4+2] > 35) ++airPixels;
    }
  }
  CHECK(airPixels > 0);
}

namespace {
struct DyeStats { float redRange, blueRange, meanRed, meanBlue; };
DyeStats dyeStats(const DyeField& dye, const SurfaceWater& water) {
  float loR = 1, hiR = 0, loB = 1, hiB = 0, sumR = 0, sumB = 0;
  int count = 0;
  for (int z = 0; z < DyeField::kSize; ++z)
    for (int y = 0; y < DyeField::kSize; ++y)
      for (int x = 0; x < DyeField::kSize; ++x) {
        const Vec3 p{((float)x+0.5f)/DyeField::kSize-0.5f,
                     ((float)y+0.5f)/DyeField::kSize-0.5f,
                     ((float)z+0.5f)/DyeField::kSize-0.5f};
        if (water.depth(p) < 0) continue;
        const auto c = dye.cell(x,y,z);
        loR = pmin(loR,c.red); hiR = pmax(hiR,c.red);
        loB = pmin(loB,c.blue); hiB = pmax(hiB,c.blue);
        sumR += c.red; sumB += c.blue; ++count;
      }
  return {hiR-loR,hiB-loB,sumR/(float)count,sumB/(float)count};
}
}

TEST(dye_diffusion_finishes_both_channels_without_motion) {
  SurfaceWater water;
  water.reset(0.6f);
  DyeField dye;
  dye.reset({0.8f,0});
  dye.inject({-0.2f,-0.2f,-0.2f},{0,1},0.08f,0.3f);
  for (int i = 0; i < 30; ++i) dye.step(1.0f/30.0f,water,{0,0,0});
  auto stats = dyeStats(dye,water);
  CHECK(stats.blueRange > 0.1f); // A visible plume survives the initial second.
  // Neither stir() nor any velocity: diffusion must outlive vortex decay and
  // pass through the Q0.16 precision floor to a completely uniform field.
  for (int i = 0; i < 27000; ++i) {
    dye.step(1.0f/30.0f,water,{0,0,0});
    if (i%30 == 0) {
      stats = dyeStats(dye,water);
      if (stats.redRange == 0 && stats.blueRange == 0) break;
    }
  }
  CHECK(stats.redRange == 0 && stats.blueRange == 0);
  CHECK(stats.meanBlue > 0.005f && stats.meanRed > 0.5f);
  const DyeSample mixed = dye.cell(4,4,4);
  for (int i = 0; i < 60; ++i) dye.step(1.0f/30.0f,water,{0,0,0});
  CHECK_NEAR(dye.cell(4,4,4).red,mixed.red,1e-6f);
  CHECK_NEAR(dye.cell(4,4,4).blue,mixed.blue,1e-6f);
  // New ink must remain local even after a completed mix.
  dye.inject({0,-0.2f,0},{0,1},0.02f);
  CHECK(dyeStats(dye,water).blueRange > 0.1f);
}

TEST(effects_sloshing_accelerates_mixing_without_requiring_splashes) {
  WaterEffects still, moving;
  still.reset(0.7f,{0.8f,0}); moving.reset(0.7f,{0.8f,0});
  still.addInk({0,1}); moving.addInk({0,1});
  for (int i = 0; i < 1200; ++i) {
    const float angle = 0.3f*fsin((float)i*(2.0f/120.0f));
    still.step(1.0f/120.0f,{0,-1,0});
    moving.step(1.0f/120.0f,{fsin(angle),-fcos(angle),0});
    CHECK(moving.dropletCount() == 0);
  }
  const auto a = dyeStats(still.dye(),still.surface());
  const auto b = dyeStats(moving.dye(),moving.surface());
  CHECK(b.blueRange < a.blueRange*0.5f);
  CHECK(b.meanBlue > 0.0001f); // Mixing must not just erase the ink.
  CHECK_NEAR(still.volume(),0.7f,1e-6f);
  CHECK_NEAR(moving.volume(),0.7f,1e-6f);
}

TEST(dye_rgb_channels_follow_identical_transport_and_diffusion) {
  SurfaceWater water; water.reset(0.7f);
  DyeField dye; dye.reset({0,0,0});
  dye.inject({0,-0.1f,0},{0.6f,0.6f,0.6f},0.02f);
  dye.stir({0,-0.1f,0},{0,-1,0},3);
  for (int i = 0; i < 90; ++i) dye.step(1.0f/30.0f,water,{0.1f,0,0},0.5f);
  float peak = 0;
  for (int z = 0; z < 16; ++z) for (int y = 0; y < 16; ++y) for (int x = 0; x < 16; ++x) {
    const auto d = dye.cell(x,y,z);
    CHECK_NEAR(d.red,d.green,1e-6f);
    CHECK_NEAR(d.blue,d.green,1e-6f);
    peak = pmax(peak,d.green);
  }
  CHECK(peak > 0.001f);
}
