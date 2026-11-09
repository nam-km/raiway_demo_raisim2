# Raiway demo on RaiSim2

The Raiway wheel-legged perceptive policy, trained in RaiSim2 (`raisimGymForSegway`), running on RaiSim2 itself, with
the training physics, contacts and height map. It comes in two forms built from the same controller code:

- **Browser page** — RaiSim and the policy compiled to WebAssembly, nothing to install:
  <https://nam-km.github.io/raiway_demo_raisim2/> (recent Chrome or Edge; on Linux enable
  `chrome://flags/#enable-unsafe-webgpu` and `#enable-vulkan`).
- **Native server** — a `RaisimServer` program viewed with `rayrai_tcp_viewer` or the RaiSim browser viewer.

The MuJoCo sibling, `raiway_demo_mujoco`, runs the same policies in MuJoCo.

## Controls

Velocity command sliders (vx ±1.5 m/s, wz ±2 rad/s), terrain (flat, stairs, steps, holes, hurdles) and level, policy,
reset. Holding ↑/↓ or W/S drives at the key forward speed, ←/→ or A/D turns at the key yaw rate; releasing returns to
the sliders; R resets. A fall (a contact other than wheels and shanks, or tilt past 60°) resets after a second,
leaving the map at once. Orange spheres are the actor's height scan; the page's *Height scan* box hides them and
its bottom-left corner names the running policy.

## Browser page

Needs Emscripten **4.0.0** (the version the RaiSim WebAssembly SDK is built with):

```bash
git clone https://github.com/emscripten-core/emsdk.git && emsdk/emsdk install 4.0.0 && emsdk/emsdk activate 4.0.0
source emsdk/emsdk_env.sh
web/build_site.sh                    # downloads webassembly-sdk-2.14.0 into build-web/ unless RAISIM_WASM_SDK is set
python3 build-web/webassembly-sdk-2.14.0/share/rayrai/web/serve.py --directory build-web/site   # http://localhost:8000
```

Every push to `main` rebuilds the page and publishes it to GitHub Pages (`.github/workflows/pages.yml`, Pages source
"GitHub Actions"). The page needs cross-origin isolation, which the SDK's `isolation-worker.js` provides on GitHub
Pages.

The SDK's license (`build-web/site/raisim-license.txt`) lets academic users distribute programs built with it, for
academic purposes only, and requires the notice the page shows; the SDK itself is never committed or published.

## Native server

Needs raisim2Lib next to this directory (or `-DRAISIM2LIB_DIR`), a C++20 compiler (g++-10 is picked when present),
CMake 3.18 and the RaiSim activation file in `~/.raisim`; libX11 is optional and gives the held-key control.

```bash
cmake -S . -B build && cmake --build build -j8
./run.sh                          # server + browser viewer, opens the page; arguments go to the server
./build/raiway_demo --lan         # serve viewers and keys from other computers (--port, --key-port)
python3 web/keys.py <host>        # send this computer's held keys to a server elsewhere (pip install pynput)
./build/raiway_demo --headless stairs 1.0 0.0 10 1.0 0   # terrain vx wz seconds [level] [policy]: prints tracking
```

`web/serve_viewer.py` serves the browser viewer of raisim2Lib's `webassembly-<version>.zip` (unzipped next to this
repository) with the robot's files preloaded, so the meshes show; pages from `localhost` can reach a server.

## Policies

`rsc/raiway` is generated, do not edit it by hand:

```bash
cd ../raisimGymForSegway/raisimGymForSegway/sim2sim
python export_raisim2_demo.py -w <RUN>/full_XXX.pt [-l "label"] [--default]
```

It adds `policies/<name>/policy.txt` and its line in `policies.txt`, and writes the URDF, `raiway.rscene` and meshes.
`policy.txt` holds the observation normalizer, the actor (LeakyReLU, `action_bound` tanh) and the optional estimator.
Rebuild the page (or push) after an export.

## Code

- `src/raiway_core.hpp` — terrains, policy loading and the controller: the deployed part of `SegwayController.hpp`
  and `Environment.hpp` (observation, estimator history, PD at 2.5 ms substeps, resets, `cfg.yaml` material
  defaults, no randomization).
- `src/raiway_web.cpp` — the browser page: Rayrai rendering, ImGui panel, keys.
- `src/raiway_demo.cpp` — the native server, remote keys and `--headless`.

Terrains use the `raiway_demo_mujoco` layouts on the 0.02 m training grid, with the boxes training buries in riser
faces and hurdles. On `STRAIGHT_STAIRS` at 0.3 m and 1.0 m/s the port matches the training environment to the second
decimal for three seconds; with a zero command these checkpoints creep backward at about 0.08 m/s, as in training.
