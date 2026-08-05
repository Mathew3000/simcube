#include "check.h"

#include <cstring>

namespace tst {
namespace {

constexpr int kMaxCases = 256;
struct Case {
  const char* name;
  Fn fn;
};
Case g_cases[kMaxCases];
int g_count = 0;
int g_caseFailures = 0;
const char* g_current = "";

}  // namespace

void registerCase(const char* name, Fn fn) {
  if (g_count < kMaxCases) g_cases[g_count++] = Case{name, fn};
}

void fail(const char* file, int line, const char* expr) {
  ++g_caseFailures;
  std::printf("    FAIL %s:%d  %s\n", file, line, expr);
}

void failNear(const char* file, int line, const char* expr, double a, double b, double eps) {
  ++g_caseFailures;
  std::printf("    FAIL %s:%d  %s   (%.9g vs %.9g, tol %.3g, delta %.3g)\n", file, line,
              expr, a, b, eps, a - b);
}

int run(int argc, char** argv) {
  const char* filter = (argc > 1) ? argv[1] : nullptr;
  int ran = 0, failed = 0;

  for (int i = 0; i < g_count; ++i) {
    if (filter && !std::strstr(g_cases[i].name, filter)) continue;
    g_current = g_cases[i].name;
    g_caseFailures = 0;
    ++ran;
    g_cases[i].fn();
    if (g_caseFailures == 0) {
      std::printf("  ok   %s\n", g_cases[i].name);
    } else {
      std::printf("  FAIL %s  (%d assertion%s)\n", g_cases[i].name, g_caseFailures,
                  g_caseFailures == 1 ? "" : "s");
      ++failed;
    }
  }

  std::printf("\n%d/%d cases passed\n", ran - failed, ran);
  return failed == 0 ? 0 : 1;
}

}  // namespace tst

int main(int argc, char** argv) { return tst::run(argc, argv); }
