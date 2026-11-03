#!/usr/bin/env bash
# Starts the simulation server and the browser viewer together and opens the viewer in the browser.
# Arguments go to raiway_demo (--lan, --port N, --key-port N); --no-browser skips opening the page.
set -e
cd "$(dirname "$0")"
RAISIM2LIB_DIR="${RAISIM2LIB_DIR:-../raisim2Lib}"
[ -x build/raiway_demo ] || { echo "build first: cmake -S . -B build && cmake --build build -j8"; exit 1; }
source "$RAISIM2LIB_DIR/raisim_env.sh" >/dev/null

port=8080 browser=1 args=()
while [ $# -gt 0 ]; do
  case "$1" in
    --port) port="$2"; args+=("$1" "$2"); shift 2 ;;
    --no-browser) browser=0; shift ;;
    *) args+=("$1"); shift ;;
  esac
done

python3 web/serve_viewer.py --port 8000 &
viewer=$!
trap 'kill $viewer 2>/dev/null' EXIT
url="http://localhost:8000/web-examples/index.html?connect=127.0.0.1:$port"
echo "viewer: $url"
if [ "$browser" = 1 ]; then (sleep 2; xdg-open "$url" >/dev/null 2>&1 || true) & fi
./build/raiway_demo "${args[@]}"
