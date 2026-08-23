#!/usr/bin/env bash
# Two things the firmware must satisfy, both checkable with no hardware present:
#
#  1. The static pools at each device capacity profile fit internal SRAM.
#  2. The physics at those capacities is reproducible -- the hash in
#     scripts/golden_hash_esp32.txt is the value the device has to print back.
#
# It builds its own trees rather than reusing build/, because the whole point is to measure
# configurations the host build does not use.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# A self-imposed ceiling on the simulation plus display footprint, not the hardware limit. The
# actual figure, from the linker on a real build of platform/esp32: 327680 bytes of DRAM, of which
# the firmware's static data is 159.7KB (the report's own total plus the Arduino core's .bss) and
# the HUB75 DMA buffers take another ~72KB from the heap at begin(). That leaves roughly 85KB for
# task stacks and the allocator.
#
# 230KB keeps the simulation+display side well inside that, so growing a pool cannot quietly eat
# the margin the DMA buffers need. Raise it deliberately, with the linker number in hand.
BUDGET_KB="${PARTSIM_SRAM_BUDGET_KB:-230}"

status=0

build_profile() {
  local prof="$1" dir="$2"
  cmake -S . -B "$dir" -DPARTSIM_PROFILE="$prof" -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null \
    || { echo "FAIL: cmake configure ($prof)"; return 1; }
  cmake --build "$dir" -j --target partsim_memreport partsim_golden >/dev/null 2>&1 \
    || { echo "FAIL: build at $prof capacities"; return 1; }
}

# A profile that is expected to fit. Over budget is a failure.
must_fit() {
  local prof="$1" dir="build-$1"
  build_profile "$prof" "$dir" || { status=1; return; }
  if "$dir/platform/host/partsim_memreport" "$BUDGET_KB" | tail -3 | sed "s/^/  [$prof] /"; then
    :
  else
    echo "FAIL: $prof does not fit ${BUDGET_KB}KB"
    status=1
  fi
}

# A profile that is KNOWN not to fit yet, with a named reason. Reported loudly but not fatal --
# and if it ever starts fitting, that is announced too, so this cannot silently become stale.
known_over() {
  local prof="$1" reason="$2" dir="build-$1"
  build_profile "$prof" "$dir" || { status=1; return; }
  if "$dir/platform/host/partsim_memreport" "$BUDGET_KB" >/dev/null 2>&1; then
    echo "  [$prof] NOW FITS ${BUDGET_KB}KB -- promote this to must_fit in $0"
  else
    "$dir/platform/host/partsim_memreport" | tail -2 | sed "s/^/  [$prof] /"
    echo "  [$prof] EXPECTED over ${BUDGET_KB}KB: $reason"
  fi
}

must_fit esp32
must_fit esp32-master

# The display node carries a whole Simulation today, and 55.7KB of that is state it never uses:
# the full Particles pool including the solver's predicted positions and lambdas (31.3KB), the
# heat field's ping-pong second buffer (10.3KB), the neighbour grid it never builds (10.5KB) and
# the solver scratch (3.6KB). A render-only state container removes all of it and brings the node
# to ~190KB. Until then this genuinely does not fit, and saying so beats a green tick.
known_over esp32-display "needs the render-only state container; carries ~55.7KB of solver state it never uses"

# The esp32 profile's hash is the reference the DEVICE must reproduce.
have="$("$ROOT/build-esp32/platform/host/partsim_golden" -q)"
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
exit $status
