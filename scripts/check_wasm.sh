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

# Staleness, checked separately from behaviour -- because behaviour cannot detect it.
#
# The determinism comparison only proves the artifact and the host agree on the golden sequence. A
# deliberately bit-neutral refactor (blocks A and B of the 64x64 work were both) leaves the hashes
# identical, so a WASM build from before the refactor passes the comparison while the browser runs
# code that no longer exists in the tree. That happened: an artifact 18 days behind core sailed
# through this check.
ART="$ROOT/platform/wasm/web/public/partsim.wasm"
NEWER="$(find "$ROOT/core" "$ROOT/platform/wasm/bindings.cpp" -type f \
         \( -name '*.cpp' -o -name '*.h' \) -newer "$ART" -print 2>/dev/null)"
if [ -n "$NEWER" ]; then
  echo "FAIL: the WASM artifact is older than sources it was built from."
  echo "  newer than $ART:"
  printf '    %s\n' $NEWER | sed "s|$ROOT/||"
  echo "  The hash comparison below would still pass -- it cannot see a stale binary."
  echo "  Rebuild:  scripts/build_wasm.sh"
  exit 1
fi

exec node "$ROOT/scripts/check_determinism.mjs"
