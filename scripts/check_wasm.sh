#!/usr/bin/env bash
# ctest wrapper for the cross-target determinism check. Skips loudly (never silently) when the
# WASM artifact has not been built, because the host build must not require emsdk.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! command -v node >/dev/null 2>&1; then
  echo "SKIP: node not found"; exit 0
fi
if [ ! -f "$ROOT/platform/wasm/web/public/partsim.mjs" ]; then
  echo "SKIP: WASM not built -- run scripts/build_wasm.sh to enable this check"; exit 0
fi
if [ ! -f "$ROOT/scripts/golden_hash.txt" ]; then
  echo "SKIP: scripts/golden_hash.txt missing"; exit 0
fi
exec node "$ROOT/scripts/check_determinism.mjs"
