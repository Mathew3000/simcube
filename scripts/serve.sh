#!/usr/bin/env bash
# Serves the web frontend. A server is required: ES modules and WASM cannot be loaded from
# file:// because of CORS.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR="$ROOT/platform/wasm/web"
PORT="${1:-8080}"

if [ ! -f "$DIR/public/partsim.wasm" ] && [ ! -f "$DIR/public/partsim_surface.wasm" ]; then
  echo "error: WASM missing -- run scripts/build_wasm.sh (or --surface) first" >&2
  exit 1
fi

echo "serving $DIR on http://localhost:$PORT/"
echo "surface-water preview: http://localhost:$PORT/surface.html"
cd "$DIR" && exec python3 -m http.server "$PORT"
