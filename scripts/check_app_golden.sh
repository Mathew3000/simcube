#!/usr/bin/env bash
# The platform HAL's own check: platform/app must not merely COMPILE away from Xtensa, it must
# produce the same physics there.
#
# A HAL with one implementation is a rename, not a seam -- the coupling stays and nobody finds out
# until somebody attempts the port. So this drives the host build of the application layer through
# its `g` command, which is the same App::runGolden the firmware's console calls, and compares the
# state hash against the same reference the host and WASM targets are held to.
set -uo pipefail
TOOL="$1"; FILE="$2"
have="$(echo g | "$TOOL" --no-step | sed -n 's/^state  *\([0-9a-f]*\).*/\1/p')"
want="$(grep -v '^#' "$FILE" | head -1 | awk '{print $1}')"
if [ -z "$have" ]; then
  echo "FAIL: the host console printed no state hash. Output was:"
  echo g | "$TOOL" --no-step
  exit 1
fi
if [ "$have" != "$want" ]; then
  echo "FAIL: the application layer's golden hash does not match this build."
  echo "  file:            $want"
  echo "  platform/app:    $have"
  echo "  The host and the firmware share platform/app/src/App.cpp, so a mismatch here means the"
  echo "  extraction reached the physics rather than that a reference is stale."
  exit 1
fi
echo "ok   platform/app reproduces the golden state hash on the host ($have)"
