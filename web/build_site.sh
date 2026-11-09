#!/usr/bin/env bash
# Builds the browser demo into build-web/site, the folder GitHub Pages serves; needs Emscripten 4.0.0 on the PATH.
# The RaiSim WebAssembly SDK comes from RAISIM_WASM_SDK, or is downloaded from the raisim2Lib release into build-web.
set -euo pipefail
cd "$(dirname "$0")/.."
version=2.14.0
sha256=e68b46224484ce854aff3ffc37cfdd21eb0d22163b0ce6a301555012a66de8f8
asset=723243747  # webassembly-sdk-$version.zip's id in the release, changed with the version

emcc --version 2>/dev/null | grep -q ' 4\.0\.0 ' || { echo "needs Emscripten 4.0.0: source emsdk_env.sh of an emsdk with 4.0.0 activated"; exit 1; }
if [ -n "${RAISIM_WASM_SDK:-}" ]; then
  sdk="$(realpath "$RAISIM_WASM_SDK")"
  [ -d "$sdk/lib/cmake/rayrai" ] || { echo "RAISIM_WASM_SDK=$RAISIM_WASM_SDK is not an unpacked webassembly-sdk"; exit 1; }
else
  sdk="$PWD/build-web/webassembly-sdk-$version"
fi
if [ ! -d "$sdk/lib/cmake/rayrai" ]; then
  mkdir -p build-web
  zip=build-web/webassembly-sdk-$version.zip
  auth=()
  [ -n "${GH_TOKEN:-}" ] && auth=(-H "Authorization: Bearer $GH_TOKEN")
  # the release page's download link often answers 504; the API serves the same file
  curl -fL --retry 5 --retry-all-errors -H "Accept: application/octet-stream" "${auth[@]}" -o "$zip" \
      "https://api.github.com/repos/raisimTech/raisim2Lib/releases/assets/$asset" ||
    curl -fL --retry 5 -o "$zip" "https://github.com/raisimTech/raisim2Lib/releases/download/v$version/webassembly-sdk-$version.zip"
  echo "$sha256  $zip" | sha256sum -c -
  unzip -q -o "$zip" -d build-web
fi

emcmake cmake -S . -B build-web -DCMAKE_BUILD_TYPE=Release -DRAISIM_WASM_SDK="$sdk" -Drayrai_DIR="$sdk/lib/cmake/rayrai" \
  -Draisim_DIR="$sdk/lib/cmake/raisim" -DRAIWAY_WEB_URL="${RAIWAY_WEB_URL:-}"
cmake --build build-web --parallel
echo "site in build-web/site; try it with: python3 $sdk/share/rayrai/web/serve.py --directory build-web/site"
