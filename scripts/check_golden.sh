#!/usr/bin/env bash
# Guards against a stale reference: if the solver changes, scripts/golden_hash.txt must be
# regenerated, or the WASM comparison silently checks against the wrong numbers.
set -uo pipefail
TOOL="$1"; FILE="$2"
have="$("$TOOL" -q)"
want="$(grep -v '^#' "$FILE" | head -1)"
if [ "$have" != "$want" ]; then
  echo "FAIL: golden hashes are stale."
  echo "  file:  $want"
  echo "  build: $have"
  echo "  If the solver or renderer changed intentionally, regenerate with:"
  echo "    ./build/platform/host/partsim_golden -q > scripts/golden_hash.txt"
  exit 1
fi
echo "ok   golden hashes match this build ($have)"
