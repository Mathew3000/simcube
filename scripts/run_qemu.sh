#!/usr/bin/env bash
# Boots the firmware under the Espressif QEMU fork and prints its console.
#
# Usage: scripts/run_qemu.sh [env] [seconds]
#
# What this verifies: boot, FreeRTOS task creation and pinning, stack sufficiency, alignment,
# software-float promotions, the console, and the golden determinism sequence on real Xtensa
# codegen. What it cannot: the display (no LCD_CAM/GDMA model), the IMU (no device model), or
# anything about TIMING -- QEMU is not cycle-accurate, so no frame-rate or blit-cost number from
# it means anything.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENVNAME="${1:-qemu}"
SECONDS_LIMIT="${2:-900}"

QEMU="${PARTSIM_QEMU:-$HOME/esp-qemu/qemu/bin/qemu-system-xtensa}"
if [ ! -x "$QEMU" ]; then
  echo "error: qemu-system-xtensa not found at $QEMU" >&2
  echo "  Espressif's fork is required -- upstream QEMU has Xtensa but no esp32s3 machine." >&2
  echo "  https://github.com/espressif/qemu/releases  (needs brew pixman libgcrypt sdl2)" >&2
  exit 1
fi

PIO="$(command -v pio || true)"
[ -z "$PIO" ] && [ -x "$HOME/.platformio/penv/bin/pio" ] && PIO="$HOME/.platformio/penv/bin/pio"
[ -z "$PIO" ] && { echo "error: pio not found" >&2; exit 1; }

cd "$ROOT/platform/esp32" || exit 1
"$PIO" run -e "$ENVNAME" >/dev/null 2>&1 || { echo "FAIL: firmware build ($ENVNAME)"; exit 1; }

# QEMU wants a whole flash image, not the three pieces esptool would write separately.
IMG="$(mktemp -t partsim-flash)"
python3 - "$ENVNAME" "$IMG" <<'PY'
import os, sys
env, out = sys.argv[1], sys.argv[2]
B = os.path.join('.pio', 'build', env)
img = bytearray(b'\xff' * (16 * 1024 * 1024))
for off, name in [(0x0, 'bootloader.bin'), (0x8000, 'partitions.bin'), (0x10000, 'firmware.bin')]:
    with open(os.path.join(B, name), 'rb') as f:
        d = f.read()
    img[off:off + len(d)] = d
with open(out, 'wb') as f:
    f.write(bytes(img))
PY

# Output to a file we can watch. QEMU never exits on its own -- it keeps serving the console -- so
# waiting for the process to end would always burn the whole timeout. Watch for the marker instead.
OUT="$(mktemp -t partsim-qemu-out)"
"$QEMU" -nographic -machine esp32s3 -drive "file=$IMG,if=mtd,format=raw" -serial mon:stdio \
  > "$OUT" 2>&1 &
QPID=$!
DONE="${PARTSIM_QEMU_MARKER:-QEMU: done}"
for _ in $(seq 1 "$SECONDS_LIMIT"); do
  kill -0 $QPID 2>/dev/null || break
  grep -q "$DONE" "$OUT" 2>/dev/null && break
  sleep 1
done
kill $QPID 2>/dev/null
wait $QPID 2>/dev/null
cat "$OUT"
rm -f "$OUT" "$IMG"
