#!/usr/bin/env bash
# Two things the firmware must satisfy, both checkable with no hardware present:
#
#  1. The static pools at the device capacity profile fit internal SRAM.
#  2. The physics at those capacities is reproducible -- the hash in
#     scripts/golden_hash_esp32.txt is the value the device has to print back.
#
# It builds its own tree (build-esp32) with PARTSIM_PROFILE=esp32 rather than reusing build/,
# because the whole point is to measure a configuration the host build does not use.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# A self-imposed ceiling on the simulation plus display footprint, not the hardware limit. The
# actual figure, from the linker on a real build of platform/esp32: 327680 bytes of DRAM, of which
# the firmware's static data is 159.7KB (this report's 127.6KB plus the Arduino core's own .bss)
# and the HUB75 DMA buffers take another ~72KB from the heap at begin(). That leaves roughly 85KB
# for task stacks and the allocator.
#
# 230KB keeps the simulation+display side well inside that, so growing a pool cannot quietly eat
# the margin the DMA buffers need. Raise it deliberately, with the linker number in hand.
BUDGET_KB="${PARTSIM_SRAM_BUDGET_KB:-230}"

cmake -S . -B build-esp32 -DPARTSIM_PROFILE=esp32 -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  >/dev/null || { echo "FAIL: cmake configure (esp32 profile)"; exit 1; }
cmake --build build-esp32 -j --target partsim_memreport partsim_golden >/dev/null 2>&1 \
  || { echo "FAIL: build at esp32 capacities"; exit 1; }

REPORT="$ROOT/build-esp32/platform/host/partsim_memreport"
GOLDEN="$ROOT/build-esp32/platform/host/partsim_golden"

"$REPORT" "$BUDGET_KB" || exit 1

have="$("$GOLDEN" -q)"
want="$(grep -v '^#' "$ROOT/scripts/golden_hash_esp32.txt" | head -1)"
if [ "$have" != "$want" ]; then
  echo "FAIL: esp32-capacity golden hash changed."
  echo "  file:  $want"
  echo "  build: $have"
  echo "  This hash is the reference the DEVICE must reproduce. If the change was intended:"
  echo "    ./build-esp32/platform/host/partsim_golden -q > scripts/golden_hash_esp32.txt"
  exit 1
fi
echo "ok   esp32-capacity golden hash matches ($have)"
