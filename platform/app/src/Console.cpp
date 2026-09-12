#include "partsim/app/Console.h"

#include <cstdio>

namespace partsim {
namespace app {

// -Wdouble-promotion is an error for the whole firmware because a stray double in the solver
// silently halves the framerate on a chip that emulates them in software. printf is the one place
// it cannot be obeyed: varargs promote float to double by definition, so there is nothing to fix.
// Suppressed here and at the call sites in App.cpp rather than globally, so the warning keeps
// working everywhere it can find a real bug.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"

void Console::printf(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
}

void Console::vprintf(const char* fmt, va_list ap) {
  char buf[kLineMax];
  const int n = ::vsnprintf(buf, sizeof(buf), fmt, ap);
  if (n < 0) return;
  // vsnprintf always terminates within the buffer, so an over-long line is truncated rather than
  // dropped. Nothing the console prints is close to kLineMax; if something ever is, a clipped
  // line is a far better failure than a heap allocation in a steady-state path.
  write(buf);
}

#pragma GCC diagnostic pop

}  // namespace app
}  // namespace partsim
