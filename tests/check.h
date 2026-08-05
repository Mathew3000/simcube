#pragma once
// Minimal test harness. Deliberately hand-rolled rather than vendoring doctest: the whole
// project must build offline on three toolchains, and this is ~80 lines against a 300KB
// header we would use maybe 5% of.
#include <cstdio>

namespace tst {

using Fn = void (*)();

void registerCase(const char* name, Fn fn);
void fail(const char* file, int line, const char* expr);
void failNear(const char* file, int line, const char* expr, double a, double b, double eps);
int run(int argc, char** argv);

struct Reg {
  Reg(const char* name, Fn fn) { registerCase(name, fn); }
};

}  // namespace tst

#define TEST(name)                                  \
  static void name();                               \
  static ::tst::Reg tst_reg_##name(#name, name);    \
  static void name()

#define CHECK(expr)                                  \
  do {                                               \
    if (!(expr)) ::tst::fail(__FILE__, __LINE__, #expr); \
  } while (0)

#define CHECK_NEAR(a, b, eps)                                                    \
  do {                                                                           \
    const double tst_a = (double)(a), tst_b = (double)(b), tst_e = (double)(eps); \
    const double tst_d = tst_a - tst_b;                                          \
    if (!(tst_d <= tst_e && tst_d >= -tst_e))                                     \
      ::tst::failNear(__FILE__, __LINE__, #a " ~= " #b, tst_a, tst_b, tst_e);     \
  } while (0)
