#pragma once
#include <cstdarg>

namespace partsim {
namespace app {

// Line-oriented text in and out, which is the whole of what the firmware console needs.
//
// It replaced 72 direct `Serial` calls. The point is not that Serial is bad -- it is that those
// 72 calls were the single largest reason the application logic could not be compiled anywhere
// but Xtensa, and therefore could not be run by a test. A UART, a USB CDC endpoint, a TCP socket
// and stdin are all the same shape once you stop naming one of them.
//
// Implementations supply two primitives and inherit the rest. `write` is the only output the
// interface actually requires, because printf-formatting into a fixed buffer is identical work
// whatever the carrier -- Serial.printf already does exactly this internally.
class Console {
 public:
  virtual ~Console() = default;

  // Emits a NUL-terminated string. Blocking is the implementation's business.
  virtual void write(const char* s) = 0;

  // Non-blocking. Copies the next complete line, without its terminator, into `buf` and returns
  // its length; returns -1 when no line is pending. Never blocks, because on the device this is
  // polled from a task that must yield.
  virtual int readLine(char* buf, int cap) = 0;

  void println(const char* s) { write(s); write("\n"); }
  void println() { write("\n"); }

  // Formatted output. Truncates rather than allocating: this runs on a device where a heap
  // allocation in a steady-state path is a fragmentation bug waiting to strand the board (see
  // tests/test_noalloc.cpp), and no console line here is close to the buffer size.
  void printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
  void vprintf(const char* fmt, va_list ap);

  static constexpr int kLineMax = 256;
};

}  // namespace app
}  // namespace partsim
