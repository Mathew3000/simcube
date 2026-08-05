// The test binary is allowed to use libm -- core is not. That asymmetry is the point:
// here we check the polynomial approximations against the real thing.
#include <cmath>

#include "check.h"
#include "partsim/Rng.h"
#include "partsim/Types.h"

using namespace partsim;

TEST(math_fsin_matches_libm) {
  double worst = 0.0;
  for (int i = -20000; i <= 20000; ++i) {
    const float x = (float)i * 0.002f;  // +/- 40 radians, well past one period
    const double d = std::fabs((double)fsin(x) - std::sin((double)x));
    if (d > worst) worst = d;
  }
  CHECK(worst < 3e-6);
  std::printf("       fsin worst abs err %.3g\n", worst);
}

TEST(math_fcos_matches_libm) {
  double worst = 0.0;
  for (int i = -20000; i <= 20000; ++i) {
    const float x = (float)i * 0.002f;
    const double d = std::fabs((double)fcos(x) - std::cos((double)x));
    if (d > worst) worst = d;
  }
  CHECK(worst < 3e-6);
}

TEST(math_fsin_exact_at_landmarks) {
  CHECK_NEAR(fsin(0.0f), 0.0f, 1e-6);
  CHECK_NEAR(fsin(kPi * 0.5f), 1.0f, 1e-6);
  CHECK_NEAR(fsin(kPi), 0.0f, 1e-6);
  CHECK_NEAR(fsin(-kPi * 0.5f), -1.0f, 1e-6);
  CHECK_NEAR(fsin(kTwoPi * 3.0f), 0.0f, 1e-5);
}

TEST(math_fexp_matches_libm) {
  double worstRel = 0.0;
  for (int i = -2000; i <= 2000; ++i) {
    const float x = (float)i * 0.01f;  // [-20, 20]
    const double want = std::exp((double)x);
    const double rel = std::fabs((double)fexp(x) - want) / want;
    if (rel > worstRel) worstRel = rel;
  }
  CHECK(worstRel < 2e-6);
  std::printf("       fexp worst rel err %.3g\n", worstRel);
}

TEST(math_fexp_saturates) {
  CHECK(fexp(-1000.0f) == 0.0f);
  CHECK(fexp(1000.0f) > 1e37f);
  CHECK_NEAR(fexp(0.0f), 1.0f, 1e-7);
}

TEST(math_prsqrt) {
  for (int i = 1; i <= 1000; ++i) {
    const float x = (float)i * 0.37f;
    CHECK_NEAR(prsqrt(x) * std::sqrt(x), 1.0f, 1e-5);
  }
}

TEST(rng_is_reproducible_and_bounded) {
  Rng a(12345), b(12345);
  for (int i = 0; i < 1000; ++i) CHECK(a.next() == b.next());

  Rng r(7);
  double sum = 0.0;
  for (int i = 0; i < 200000; ++i) {
    const float f = r.nextFloat();
    CHECK(f >= 0.0f && f < 1.0f);
    sum += f;
  }
  CHECK_NEAR(sum / 200000.0, 0.5, 0.01);  // uniform enough for our purposes
}

TEST(rng_never_degenerates_to_zero_state) {
  // An all-zero xoshiro state is a fixed point; reseed must never produce one.
  for (uint32_t seed = 0; seed < 64; ++seed) {
    Rng r(seed);
    bool nonzero = false;
    for (int i = 0; i < 8; ++i) nonzero = nonzero || (r.next() != 0u);
    CHECK(nonzero);
  }
}

TEST(rng_nextInt_in_range) {
  Rng r(99);
  int hist[5] = {0, 0, 0, 0, 0};
  for (int i = 0; i < 50000; ++i) {
    const int v = r.nextInt(5);
    CHECK(v >= 0 && v < 5);
    ++hist[v];
  }
  for (int i = 0; i < 5; ++i) CHECK(hist[i] > 9000);
}

TEST(vec3_basics) {
  const Vec3 a{1, 2, 3}, b{4, 5, 6};
  CHECK_NEAR(dot(a, b), 32.0f, 1e-6);
  const Vec3 c = cross(a, b);
  CHECK_NEAR(c.x, -3.0f, 1e-6);
  CHECK_NEAR(c.y, 6.0f, 1e-6);
  CHECK_NEAR(c.z, -3.0f, 1e-6);
  CHECK_NEAR(dot(c, a), 0.0f, 1e-5);
  CHECK_NEAR(dot(c, b), 0.0f, 1e-5);
  CHECK_NEAR(length(normalize(b)), 1.0f, 1e-6);
  // Degenerate normalize must not produce NaN.
  const Vec3 z = normalize(Vec3{0, 0, 0});
  CHECK(z.x == 0.0f && z.y == 0.0f && z.z == 0.0f);
}

TEST(quat_conjugate_rotation_maps_world_to_object) {
  // 90 deg about +Z. Object rotated by q; a world -Y gravity should read as +X (or -X)
  // in object space -- check it against the explicit expectation.
  const float s = std::sin(kPi * 0.25f), c = std::cos(kPi * 0.25f);
  const Quat q{0, 0, s, c};
  const Vec3 g = rotateByConjugate(q, Vec3{0, -1, 0});
  CHECK_NEAR(g.x, -1.0f, 1e-5);
  CHECK_NEAR(g.y, 0.0f, 1e-5);
  CHECK_NEAR(g.z, 0.0f, 1e-5);
  CHECK_NEAR(length(g), 1.0f, 1e-5);
}

TEST(aabb_expand_and_contains) {
  Aabb b = Aabb::empty();
  b.expand(Vec3{-1, -2, -3});
  b.expand(Vec3{4, 5, 6});
  CHECK_NEAR(b.size().x, 5.0f, 1e-6);
  CHECK_NEAR(b.size().y, 7.0f, 1e-6);
  CHECK_NEAR(b.size().z, 9.0f, 1e-6);
  CHECK(b.contains(Vec3{0, 0, 0}));
  CHECK(!b.contains(Vec3{10, 0, 0}));
  CHECK_NEAR(b.center().x, 1.5f, 1e-6);
}
