#!/usr/bin/env bash
# Builds the WASM module and copies it next to the web frontend.
#
# Two CONFIGURATIONS of the same module ship side by side:
#
#   scripts/build_wasm.sh            -> public/partsim.mjs         the default tier; every page but
#                                                                  beakers.html loads it, and it is
#                                                                  the one check_determinism.mjs
#                                                                  compares against the host
#   scripts/build_wasm.sh --beaker   -> public/partsim_beaker.mjs  the beaker tier: chroma, water
#                                                                  only, d=2.5. beakers.html only.
#   scripts/build_wasm.sh --ink      -> public/partsim_ink.mjs     the ink tier: a dye field with
#                                                                  no bulk PBF. ink.html only.
#
# --beaker is a FLAG rather than a -D the caller passes, because the switch that matters
# (PARTSIM_ENABLE_CHROMA) is set by the tier inside Config.h and is not a CMake cache variable:
# `scripts/build_wasm.sh -DPARTSIM_ENABLE_CHROMA=1` sets an option nothing reads and silently
# produces a chroma-free artifact that looks exactly like a mix that will not converge. Only
# PARTSIM_MAX_* are forwarded as cache variables (core/CMakeLists.txt), so the tier has to arrive
# as a compiler flag. See DECISIONS.md D67.
#
# emsdk requires Python >= 3.10 and macOS ships 3.9 with Xcode, so EMSDK_PYTHON has to point
# at a newer interpreter or the whole toolchain fails with a confusing error.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

: "${EMSDK_PYTHON:=/opt/homebrew/bin/python3.12}"
export EMSDK_PYTHON
EMSDK_DIR="${EMSDK_DIR:-$HOME/emsdk}"

NAME=partsim
BUILD_DIR=build-wasm
TIER_FLAGS=""
if [ "${1:-}" = "--ink" ]; then
  shift
  NAME=partsim_ink
  BUILD_DIR=build-wasm-ink
  TIER_FLAGS="-DCMAKE_CXX_FLAGS=-DPARTSIM_TIER_INK=1"
elif [ "${1:-}" = "--beaker" ]; then
  shift
  NAME=partsim_beaker
  # Its OWN build directory. Two configurations sharing one is a stale-object bug waiting to
  # happen: CMake reconfigures, but objects whose sources did not change are kept, so a beaker
  # build in build-wasm would link default-tier objects with chroma-tier ones and the struct
  # layouts would disagree.
  BUILD_DIR=build-wasm-beaker
  TIER_FLAGS="-DCMAKE_CXX_FLAGS=-DPARTSIM_TIER_BEAKER=1"
fi

if ! command -v emcmake >/dev/null 2>&1; then
  if [ -f "$EMSDK_DIR/emsdk_env.sh" ]; then
    # shellcheck disable=SC1091
    source "$EMSDK_DIR/emsdk_env.sh" >/dev/null 2>&1
  else
    echo "error: emcmake not found and no emsdk at $EMSDK_DIR" >&2
    echo "  git clone https://github.com/emscripten-core/emsdk.git ~/emsdk" >&2
    echo "  cd ~/emsdk && EMSDK_PYTHON=/opt/homebrew/bin/python3.12 ./emsdk install latest && ./emsdk activate latest" >&2
    exit 1
  fi
fi

emcmake cmake -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release \
  -DPARTSIM_WASM_NAME="$NAME" ${TIER_FLAGS:+"$TIER_FLAGS"} "$@" >/dev/null
cmake --build "$BUILD_DIR" -j

DEST="$ROOT/platform/wasm/web/public"
mkdir -p "$DEST"
cp "$BUILD_DIR/platform/wasm/$NAME.mjs" "$DEST/"
cp "$BUILD_DIR/platform/wasm/$NAME.wasm" "$DEST/"

echo "built -> $DEST/$NAME.mjs ($(wc -c <"$DEST/$NAME.mjs" | tr -d ' ') bytes)"
echo "        $DEST/$NAME.wasm ($(wc -c <"$DEST/$NAME.wasm" | tr -d ' ') bytes)"
