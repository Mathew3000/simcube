#!/usr/bin/env bash
# ctest wrapper: the firmware must still compile and link for the Xtensa target.
#
# Worth being a test rather than something done by hand, because the core is shared. A change that
# is fine on clang and Emscripten can fail on the device toolchain -- a stray double that
# -Wdouble-promotion catches, a header that only newlib lacks, a pool that no longer fits. Without
# this, the first sign would be at flashing time.
#
# Skips loudly when PlatformIO is absent: the host build must not require it.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

PIO="$(command -v pio || true)"
if [ -z "$PIO" ] && [ -x "$HOME/.platformio/penv/bin/pio" ]; then
  PIO="$HOME/.platformio/penv/bin/pio"
fi
if [ -z "$PIO" ]; then
  echo "SKIP: pio not found -- run scripts/build_esp32.sh to enable this check"
  exit 0
fi

cd "$ROOT/platform/esp32" || exit 1
out="$("$PIO" run -e cube 2>&1)"
if [ $? -ne 0 ] || ! printf '%s' "$out" | grep -q "SUCCESS"; then
  printf '%s\n' "$out" | grep -E "error:|Error" | head -20
  echo "FAIL: firmware did not build"
  exit 1
fi
printf '%s\n' "$out" | grep -E "RAM:|Flash:"
echo "ok   esp32 firmware builds"
