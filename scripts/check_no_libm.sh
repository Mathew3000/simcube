#!/usr/bin/env bash
# core/ must be free of libm transcendentals: sinf/cosf/expf/powf are not specified to any
# particular accuracy, so they differ in the last bits between newlib (ESP32), musl
# (Emscripten) and libSystem (macOS) -- enough to diverge a PBF trajectory within a few
# hundred steps. Math.h is the single sanctioned <cmath> include, and only for sqrtf.
set -uo pipefail
ROOT="${1:-.}"
CORE="$ROOT/core"
status=0

files=$(find "$CORE" -type f \( -name '*.h' -o -name '*.cpp' \) | sort)

# Comments legitimately mention these names, so scan code only: drop // line comments and
# blank out /* */ block comment bodies before matching.
strip_comments() {
  sed -e 's|//.*||' -e 's|/\*[^*]*\*/||g' "$1"
}

banned='\b(sinf?|cosf?|tanf?|asinf?|acosf?|atanf?|atan2f?|expf?|exp2f?|logf?|log2f?|log10f?|powf?|fmodf?|ldexpf?|cbrtf?|hypotf?)[[:space:]]*\('

for f in $files; do
  rel="${f#$ROOT/}"
  is_math=0
  [ "$rel" = "core/include/partsim/Math.h" ] && is_math=1

  code=$(strip_comments "$f")

  if [ $is_math -eq 0 ]; then
    if hit=$(printf '%s\n' "$code" | grep -nE '#include[[:space:]]*<(cmath|math\.h)>'); then
      echo "FAIL: $rel includes <cmath>/<math.h>; only core/include/partsim/Math.h may:"
      printf '%s\n' "$hit" | sed 's/^/  /'
      status=1
    fi
    # sqrtf is fine (IEEE-754 requires correct rounding) but must go through Math.h.
    if hit=$(printf '%s\n' "$code" | grep -nE '\bsqrtf?[[:space:]]*\('); then
      echo "FAIL: $rel calls sqrt directly; use partsim::psqrt / prsqrt:"
      printf '%s\n' "$hit" | sed 's/^/  /'
      status=1
    fi
  fi

  if hit=$(printf '%s\n' "$code" | grep -nE "$banned"); then
    echo "FAIL: $rel calls a libm transcendental; use partsim::fsin/fcos/fexp:"
    printf '%s\n' "$hit" | sed 's/^/  /'
    status=1
  fi
done

if [ $status -eq 0 ]; then
  echo "ok   core/ is free of nondeterministic libm calls ($(printf '%s\n' "$files" | wc -l | tr -d ' ') files)"
fi
exit $status
