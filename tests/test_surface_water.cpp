#include "check.h"
#include "partsim/SurfaceWater.h"

using namespace partsim;

TEST(surface_water_analytic_volume) {
  // Known cross-sections: axis, square diagonal triangle, cube-corner tetrahedron.
  CHECK_NEAR(SurfaceWater::volumeBelow({0,1,0}, -0.2f), 0.3f, 1e-6f);
  CHECK_NEAR(SurfaceWater::volumeBelow({1,1,0}, -0.5f), 0.125f, 1e-6f);
  CHECK_NEAR(SurfaceWater::volumeBelow({1,1,1}, -1.0f), 1.0f/48.0f, 1e-6f);
  for (int x = 0; x <= 5; ++x) for (int z = 0; z <= 5; ++z) {
    const Vec3 n = normalize({(float)x*0.2f, 1.0f, (float)z*0.2f});
    float last = 0.0f;
    for (int i = 0; i <= 100; ++i) {
      const float h = -1.0f + (float)i*0.02f;
      const float v = SurfaceWater::volumeBelow(n, h);
      CHECK(v >= last-1e-6f);
      CHECK_NEAR(v+SurfaceWater::volumeBelow(n, -h), 1.0f, 2e-6f);
      last = v;
    }
  }
  CHECK_NEAR(SurfaceWater::volumeBelow(normalize({1e-7f,1,1e-6f}), 0.1f), 0.6f, 2e-6f);
}

TEST(surface_water_fill_survives_orientation_and_slosh) {
  SurfaceWater w;
  w.reset(0.27f);
  for (int i = 0; i < 1800; ++i) {
    const float t = (float)i/60.0f;
    w.step(1.0f/60.0f, {fsin(t), -fcos(t), fsin(t*0.7f)});
    CHECK_NEAR(w.volume(), 0.27f, 1e-7f);
    CHECK_NEAR(SurfaceWater::volumeBelow(w.up(), w.level()), w.volume(), 4e-6f);
    CHECK_NEAR(length2(w.up()), 1.0f, 1e-5f);
  }
}

TEST(surface_water_settles_and_handles_exact_inversion) {
  SurfaceWater w;
  w.reset();
  w.impulse({2, 0, 1});
  for (int i = 0; i < 900; ++i) w.step(1.0f/60.0f, {0,1,0});
  CHECK_NEAR(w.up().y, -1.0f, 1e-5f);
  for (int i = 0; i < 900; ++i) w.step(1.0f/60.0f, {0,-1,0});
  CHECK_NEAR(w.up().y, 1.0f, 1e-5f);
  w.reset();
  for (int i = 0; i < 900; ++i) w.step(1.0f/60.0f, {0,1,0});
  CHECK_NEAR(w.up().y, -1.0f, 1e-5f);
}

TEST(surface_water_pour_conserves_and_backpressures) {
  SurfaceWater a, b;
  a.reset(0.7f); b.reset(0.1f);
  CHECK_NEAR(a.pourTo(b, 0.1f), 0, 1e-8f);
  CHECK_NEAR(a.pourTo(a, 0.1f), 0, 1e-8f);
  for (int i = 0; i < 600; ++i) {
    a.step(1.0f/60.0f, {0,1,0});
    b.step(1.0f/60.0f, {0,-1,0});
    a.pourTo(b, 1.0f/60.0f);
    CHECK_NEAR(a.volume()+b.volume(), 0.8f, 5e-6f);
  }
  CHECK(a.volume() < 0.1f);
  CHECK(b.volume() > 0.7f);
  b.setVolume(1.0f);
  const float before = a.volume();
  CHECK_NEAR(a.pourTo(b, 0.1f), 0, 1e-8f);
  CHECK_NEAR(a.volume(), before, 1e-8f);
}

TEST(surface_water_shader_bounds_and_resolution) {
  SurfaceWater w;
  w.reset(0.5f);
  uint8_t pixels[64*64*4+8];
  const int resolutions[] = {8, 32, 64};
  for (int res : resolutions) {
    // Profiles with a smaller panel capacity simply skip the larger geometry.
    if (res*res > kMaxPanelTexels) continue;
    const Geometry g = Geometry::cube(res, 32.0f/(float)res);
    CHECK(g.count() == 6);
    for (int face = 0; face < 6; ++face) {
      for (auto& p : pixels) p = 123;
      w.renderPanel(g.at(face), 32.0f, pixels+4);
      for (int j = 0; j < 4; ++j) {
        CHECK(pixels[j] == 123);
        CHECK(pixels[res*res*4+4+j] == 123);
      }
      for (int j = 0; j < res*res; ++j) CHECK(pixels[4+j*4+3] == 255);
    }
    // At rest, +Z has bright wet bottom rows and dark dry top rows.
    w.renderPanel(g.at(1), 32.0f, pixels);
    CHECK(pixels[2] > 100);
    CHECK(pixels[((res-1)*res)*4+2] == 6);
    w.setVolume(0);
    w.renderPanel(g.at(4), 32.0f, pixels);
    CHECK(pixels[2] == 6);
    w.setVolume(1);
    w.renderPanel(g.at(5), 32.0f, pixels);
    CHECK(pixels[2] > 100);
    w.setVolume(0.5f);
  }
}
