#!/usr/bin/env bash
# Builds the WASM module and copies it next to the web frontend.
#
# emsdk requires Python >= 3.10 and macOS ships 3.9 with Xcode, so EMSDK_PYTHON has to point
# at a newer interpreter or the whole toolchain fails with a confusing error.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

: "${EMSDK_PYTHON:=/opt/homebrew/bin/python3.12}"
export EMSDK_PYTHON
EMSDK_DIR="${EMSDK_DIR:-$HOME/emsdk}"

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

emcmake cmake -B build-wasm -DCMAKE_BUILD_TYPE=Release "$@" >/dev/null
cmake --build build-wasm -j

DEST="$ROOT/platform/wasm/web/public"
mkdir -p "$DEST"
cp build-wasm/platform/wasm/partsim.mjs "$DEST/"
cp build-wasm/platform/wasm/partsim.wasm "$DEST/"

echo "built -> $DEST/partsim.mjs ($(wc -c <"$DEST/partsim.mjs" | tr -d ' ') bytes)"
echo "        $DEST/partsim.wasm ($(wc -c <"$DEST/partsim.wasm" | tr -d ' ') bytes)"
