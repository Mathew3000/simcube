#!/usr/bin/env bash
# The third leg of the cross-target determinism check.
#
# The host and WASM builds already agree bit-for-bit. The DEVICE hash in
# scripts/golden_hash_esp32.txt was, until this check existed, a recorded expectation that nothing
# had ever produced -- and a reference nobody has tested is a reference nobody should trust.
#
# QEMU cannot verify the display, the IMU or any timing. It can run the solver on real Xtensa
# codegen with real newlib, which is exactly what the determinism claim is about.
#
# Skips loudly without QEMU, like the WASM and PlatformIO checks.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
QEMU="${PARTSIM_QEMU:-$HOME/esp-qemu/qemu/bin/qemu-system-xtensa}"

if [ ! -x "$QEMU" ]; then
  echo "SKIP: Espressif QEMU not found at $QEMU -- see scripts/run_qemu.sh"
  exit 0
fi
if ! command -v python3 >/dev/null 2>&1; then echo "SKIP: python3 not found"; exit 0; fi

LOG="$(mktemp -t partsim-qemu)"
bash "$ROOT/scripts/run_qemu.sh" qemu "${PARTSIM_QEMU_TIMEOUT:-900}" > "$LOG" 2>&1

want="$(grep -v '^#' "$ROOT/scripts/golden_hash_esp32.txt" | head -1 | awk '{print $1}')"
got="$(grep -oE '^state  [0-9a-f]{8}' "$LOG" | awk '{print $2}' | head -1)"

if [ -z "$got" ]; then
  echo "FAIL: the firmware did not report a state hash under QEMU."
  tail -20 "$LOG" | sed 's/^/  /'
  rm -f "$LOG"; exit 1
fi
if [ "$got" != "$want" ]; then
  echo "FAIL: device-capacity determinism broken on emulated Xtensa."
  echo "  reference (golden_hash_esp32.txt): $want"
  echo "  emulated Xtensa:                   $got"
  echo "  The host and WASM builds agree, so this is Xtensa codegen or newlib diverging."
  rm -f "$LOG"; exit 1
fi
echo "ok   emulated Xtensa reproduces the device hash ($got)"
echo "     (no timing is reported: QEMU drives guest timers from host wall-clock, so any"
echo "      ms/step from it measures the build machine rather than the S3)"
rm -f "$LOG"
