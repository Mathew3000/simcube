#!/usr/bin/env bash
# Serves the web frontend. A server is required: ES modules and WASM cannot be loaded from
# file:// because of CORS.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR="$ROOT/platform/wasm/web"
PORT="${1:-8080}"

if [ ! -f "$DIR/public/partsim.wasm" ]; then
  echo "error: $DIR/public/partsim.wasm missing -- run scripts/build_wasm.sh first" >&2
  exit 1
fi

echo "serving $DIR on http://localhost:$PORT/"
cd "$DIR" && exec python3 -m http.server "$PORT"
