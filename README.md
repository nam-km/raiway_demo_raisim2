# Raiway demo on RaiSim2

The Raiway wheel-legged perceptive policy, trained in RaiSim2 (`raisimGymForSegway`), running on RaiSim2 itself: a
`RaisimServer` program in the form of a raisim2Lib server example, driven from the viewer's Controls window. The
simulation runs natively on a computer with raisim2Lib; the viewer can be the desktop `rayrai_tcp_viewer` or the
browser viewer of the RaiSim WebAssembly bundle, on the same computer or on another one that has no RaiSim at all.

The MuJoCo sibling, `raiway_demo_mujoco`, runs the same policies as a static web page; this one keeps the training
physics, contacts and height map, so what the viewer shows is what the policy was trained on.

## Build and run

Needs raisim2Lib (its `raisim` SDK and `examples/include`) next to this directory, or elsewhere through
`-DRAISIM2LIB_DIR`, g++-10 or another C++20 compiler, CMake 3.18 and the RaiSim activation file in `~/.raisim`.
libX11 is optional and gives the keyboard control.

```bash
cmake -S . -B build
cmake --build build -j8
./run.sh                       # server, browser viewer and the page in the browser, all at once
```

`run.sh` sources `../raisim2Lib/raisim_env.sh` (`RAISIM2LIB_DIR` points elsewhere), serves the viewer on port 8000,
opens `http://localhost:8000/web-examples/index.html?connect=127.0.0.1:8080` and runs the server in the foreground;
Ctrl+C ends both. Its arguments go to the server, `--no-browser` aside. The two halves also run on their own:

```bash
source ../raisim2Lib/raisim_env.sh
./build/raiway_demo            # RaisimServer on 127.0.0.1:8080, remote keys on UDP 8081
./build/raiway_demo --lan      # also accepts viewers and keys from other computers
./build/raiway_demo --port 8082 --key-port 8083
python3 web/serve_viewer.py    # the browser viewer, see below
```

`rsc/raiway` is found relative to the executable like raisim2Lib's `rsc`, so the program also runs from another
working directory.

The Controls window of the viewer has eight sliders:

- **Forward speed vx** (-1.5 to 1.5 m/s) and **Yaw rate wz** (-2 to 2 rad/s): the velocity command, the same
  interface the policy sees in both training episode types. Leaving both near zero makes the robot stand.
- **Key forward speed** and **Key yaw rate**: the command a held key gives (below).
- **Terrain** (0 flat, 1 stairs, 2 steps, 3 holes, 4 hurdles) and **Terrain level** (0 to 1): the terrain is
  rebuilt and the robot reset half a second after the slider stops moving.
- **Policy**: index into the exported checkpoints, listed with their labels in the slider name and on the console.
- **Reset**: moving it anywhere resets the robot. A fall (any contact other than the wheels and shanks, or the base
  tilted past 60°) resets it one second later, and so does driving off the map.

Held keys drive like a joystick: while ↑/↓ or W/S is down the command is ±the key forward speed, while ←/→ or
A/D is down ±the key yaw rate, and releasing them returns to the vx/wz sliders; R resets. The program polls the
key state of the X display it was started on (`DISPLAY`), so the keys work whichever window has focus, the browser
included. From another computer, `web/keys.py <host> [port]` sends that computer's held keys to the server's UDP key
port (8081) twenty times a second, and the server releases them half a second after the last message. The viewer
itself cannot send keys.

The orange spheres are the actor's height scan, the points the policy reads the terrain at. The viewer's own
Simulation row pauses and steps the server.

## Viewing from a browser

The `webassembly-<version>.zip` of a raisim2Lib release holds the browser version of `rayrai_tcp_viewer`. A browser
cannot read the robot's meshes from the server's disk, so `web/serve_viewer.py` serves a copy of the bundle whose
viewer page stores `rsc/raiway` in the viewer's file system (under `/examples/rsc/raiway`, the one place its file
writer accepts) and starts the viewer with that directory as a resource directory, where the streamed meshes and
the shared scene files are found by their path tails; the robot then shows without any download prompt. Unzip the bundle next to this repository (or pass `--bundle`) and run, on the
computer whose browser will view the demo:

```bash
python3 web/serve_viewer.py              # http://localhost:8000/web-examples/index.html, --port and --out to change
```

Open that address, choose *Connect to a server* and enter the host and port of the computer running `raiway_demo`
(`127.0.0.1:8080` on the same computer), or open it as `index.html?connect=127.0.0.1:8080`. The copy lives in
`build/web` and is refreshed from the bundle and `rsc/raiway` on every start, so re-run the script after an export.
The browser needs WebGPU, so a current Chrome or Edge; on Linux, Chrome still keeps it behind
`chrome://flags/#enable-unsafe-webgpu` and `#enable-vulkan`. A server reached from another computer has to be
started with `--lan`; pages served from `localhost` are accepted without further setup, and `https://raisim.com`'s
copy of the viewer can only reach a server on its own computer, and shows no robot since it has no way to get the
meshes.

## Viewing from a Windows (or any other) computer without RaiSim

The other computer needs Python 3 and Chrome or Edge, nothing of RaiSim. On the Linux computer start the server for
the network and open its ports if a firewall is on:

```bash
./run.sh --lan                              # or ./build/raiway_demo --lan
sudo ufw allow 8080/tcp && sudo ufw allow 8081/udp
```

On the other computer, clone this repository (for `rsc/raiway` and the scripts), unzip the WebAssembly bundle next
to it, then:

```bat
python web\serve_viewer.py
pip install pynput
python web\keys.py <IP of the Linux computer>
```

and open `http://localhost:8000/web-examples/index.html?connect=<IP of the Linux computer>:8080`. The viewer page
must be served on the viewing computer itself (`localhost`), since the server accepts browser pages from `localhost`
and from `https://raisim.com` only; a page on any other host needs `RaisimServer::allowWebViewerOrigin()`.

## Policies

`rsc/raiway` is generated by `raisimGymForSegway/raisimGymForSegway/sim2sim/export_raisim2_demo.py`; do not edit
it by hand.

```bash
cd ../raisimGymForSegway/raisimGymForSegway/sim2sim
python export_raisim2_demo.py -w <RUN>/full_XXX.pt [-l "label"] [--default]   # writes ../../../raiway_demo_raisim2/rsc/raiway (-o overrides)
```

Each export adds or replaces `policies/<name>/policy.txt` and its line in `policies.txt`, writes the URDF with the
nominal wheel parameters filled in (`raiway.urdf`) and the `raiway.rscene` that places it, and copies the meshes
from the training repository; the name is the run directory unless `-n` gives another. `--default` names the
policy the program starts with. To drop a policy, delete its folder and its line.

`policy.txt` is plain text: the observation normalizer, the actor's linear layers (LeakyReLU between them, the
`action_bound` tanh on the output) and, for a checkpoint trained with an estimator, the estimator's input normalizer,
layers and output statistics. A `--no_est` checkpoint has `est_dim` 0 and no estimator block. Line and ring actor
scans are both supported; the layout is read off the observation width.

## What the program mirrors

`src/raiway_demo.cpp` reimplements the deployed part of `SegwayController.hpp` and `Environment.hpp` on the
raisim2Lib SDK alone, without the training repository:

- The URDF template `raiway_param_20inch_new.urdf` at the nominal wheel parameters, in `FORCE_AND_TORQUE` mode
  with the training PD gains applied every 2.5 ms substep (four per 10 ms control step); the 90 Nm joint effort
  limit of the URDF is applied by RaiSim as in training.
- The actor observation (base z axis, angular velocity, joint positions and velocities, the last two actions and
  the per-tire height scan through `HeightMap::getHeight`), the estimator history and the reset that walks the
  lower rim of both tires down onto the terrain.
- `cfg.yaml`'s material defaults: friction 0.8, restitution 0.5 with a 0.1 m/s threshold. Every randomization is
  left out.

On `STRAIGHT_STAIRS` at 0.3 m with a 1.0 m/s command, the program and the training environment in test mode agree on
base position and yaw to the second decimal for the first three seconds and then drift apart slowly at the step
edges, with the same oblique climb the policy shows in training. With a zero command the policy creeps backward at
about 0.08 m/s, in the training environment as well; it is a behaviour of these checkpoints, not of the port.

Terrains use the `raiway_demo_mujoco` layouts on the training grid (0.02 m) so the two demos can be compared,
built the way training builds them: a height map plus the full-width boxes training
buries in riser faces and hurdles, since the grid alone rounds a vertical edge off.

- Stairs: step height 0.02 + 0.28·level, tread max(height / tan 35°, 0.3) + 0.1 m, eight steps up from 0.75 m past
  the spawn, a 2 m landing, then eight steps down, with a 4 cm riser plate in each face. Training has no descent.
- Steps: 0.42 m square blocks with heights U(0, 0.05 + 0.25·level).
- Holes: depth 0.05 + 0.75·level², two independent 3 m lanes with gaps U(0.4, 0.7) m, landings U(0.3, 0.6) m and
  landing heights U(-0.1, 0.1) m, and a flat 1.5 m spawn strip.
- Hurdles: 0.05 m thick boxes 0.02 + 0.28·level high, 1.5 m apart from 1 m past the spawn.

The map spans x from -3 m to 13 m and y from -4 m to 4 m; the random layouts of steps and holes come from a fixed
seed and differ from the MuJoCo demo's.

## Checks

```bash
./build/raiway_demo --headless stairs 1.0 0.0 10 1.0 0   # terrain vx wz seconds [level] [policy index]; no server
```

It prints the base position, yaw and speed once a second, every fall or map exit, and the mean tracked velocity.

## Running it as a raisim2Lib browser example

The source follows the raisim2Lib server example conventions (`example_resources.hpp`, `rayrai_tcp_viewer_hint.hpp`,
assets under `rsc/`), so dropped into `examples/src/server/robots/` with `rsc/raiway` it builds as one of them. The
WebAssembly bundle that raisim.com's *Run on web* buttons use is compiled from those sources with the RaiSim source
checkout and Emscripten, which the SDK distribution does not include, so becoming a browser-side example needs that
build on raisimTech's side.
