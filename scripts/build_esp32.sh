#!/usr/bin/env bash
# Builds the ESP32 firmware. No device required -- compiling and linking is itself a real check:
# it is what proves the core actually works on the Xtensa toolchain at the device capacity
# profile, and the linker's RAM figure is the authority on the memory budget.
#
# Usage: scripts/build_esp32.sh [env...]        (default: cube)
#        scripts/build_esp32.sh --upload cube   (flash it, once hardware exists)
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJ="$ROOT/platform/esp32"

# PlatformIO is commonly installed only inside its own virtualenv (that is what the VS Code
# extension does), so fall back to it rather than insisting on a pio on PATH.
PIO="$(command -v pio || true)"
if [ -z "$PIO" ] && [ -x "$HOME/.platformio/penv/bin/pio" ]; then
  PIO="$HOME/.platformio/penv/bin/pio"
fi
if [ -z "$PIO" ]; then
  echo "error: pio not found. Install with: pipx install platformio" >&2
  exit 1
fi

TARGET=()
if [ "${1:-}" = "--upload" ]; then
  TARGET=(--target upload)
  shift
fi

ENVS=("$@")
if [ ${#ENVS[@]} -eq 0 ]; then ENVS=(cube); fi

cd "$PROJ" || exit 1
status=0
for e in "${ENVS[@]}"; do
  echo "=== $e ==="
  if ! "$PIO" run -e "$e" "${TARGET[@]}" 2>&1 | grep -E "RAM:|Flash:|error:|SUCCESS|FAILED"; then
    status=1
  fi
done
exit $status
