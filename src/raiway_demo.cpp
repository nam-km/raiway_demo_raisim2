// Raiway, a wheel-legged biped, drives its perceptive RL policy over stairs, blocks, holes and hurdles.
// The viewer's Controls window and the arrow keys, held on this display or sent by web/keys.py from
// another computer, give the velocity command; the Controls window also chooses terrain, level and policy.

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>

#include "example_resources.hpp"
#include "raisim/RaisimServer.hpp"
#include "raiway_core.hpp"
#include "rayrai_tcp_viewer_hint.hpp"

/// after Eigen, whose enumerators X.h's macros would rename
#ifdef RAIWAY_X11
#include <X11/Xlib.h>
#include <X11/keysym.h>
#undef Success
#undef None
#undef Status
#undef Bool
#undef Always
#endif

namespace {

using namespace raiway;

/// Arrow or WASD keys held anywhere on this X display drive the robot like a joystick; R resets.
class HeldKeys {
 public:
  struct State {
    double forward = 0.0, turn = 0.0;
    bool reset = false;
  };

  HeldKeys() {
#ifdef RAIWAY_X11
    display_ = XOpenDisplay(nullptr);
    if (!display_) {
      std::cout << "no X display: the sliders alone give the command" << std::endl;
      return;
    }
    for (auto& [sym, code] : keys_) code = XKeysymToKeycode(display_, sym);
    std::cout << "hold up/down or W/S to drive, left/right or A/D to turn, anywhere on this display; R resets" << std::endl;
#else
    std::cout << "built without X11: the sliders alone give the command" << std::endl;
#endif
  }

  State poll() {
    State state;
#ifdef RAIWAY_X11
    if (!display_) return state;
    char map[32];
    XQueryKeymap(display_, map);
    auto down = [&](size_t i) { return (map[keys_[i].second / 8] >> (keys_[i].second % 8)) & 1; };
    state.forward = double(down(0) || down(4)) - double(down(1) || down(5));
    state.turn = double(down(2) || down(6)) - double(down(3) || down(7));
    const bool resetDown = down(8);
    state.reset = resetDown && !resetWasDown_;
    resetWasDown_ = resetDown;
#endif
    return state;
  }

 private:
#ifdef RAIWAY_X11
  Display* display_ = nullptr;
  std::array<std::pair<KeySym, KeyCode>, 9> keys_{{{XK_Up, 0}, {XK_Down, 0}, {XK_Left, 0}, {XK_Right, 0},
                                                   {XK_w, 0}, {XK_s, 0}, {XK_a, 0}, {XK_d, 0}, {XK_r, 0}}};
#endif
  bool resetWasDown_ = false;
};

/// Held keys sent by web/keys.py over UDP as "forward turn reset" lines, for a viewer on another computer.
class KeyLink {
 public:
  explicit KeyLink(int port) {
    if (port <= 0) return;
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(uint16_t(port));
    if (socket_ < 0 || bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
      std::cout << "UDP port " << port << " is taken: no remote keys" << std::endl;
      if (socket_ >= 0) close(socket_);
      socket_ = -1;
      return;
    }
    fcntl(socket_, F_SETFL, O_NONBLOCK);
    std::cout << "remote keys: run web/keys.py <this host> " << port << " on the viewer's computer" << std::endl;
  }
  ~KeyLink() { if (socket_ >= 0) close(socket_); }

  HeldKeys::State poll() {
    HeldKeys::State state;
    if (socket_ < 0) return state;
    char buffer[64];
    sockaddr_in from{};
    socklen_t fromLength = sizeof(from);
    ssize_t length;
    while ((length = recvfrom(socket_, buffer, sizeof(buffer) - 1, 0, reinterpret_cast<sockaddr*>(&from), &fromLength)) > 0) {
      buffer[length] = 0;
      int reset = 0;
      if (std::sscanf(buffer, "%lf %lf %d", &forward_, &turn_, &reset) == 3) {
        resetHeld_ = reset != 0;
        received_ = std::chrono::steady_clock::now();
        if (!announced_) {
          std::cout << "keys from " << inet_ntoa(from.sin_addr) << std::endl;
          announced_ = true;
        }
      }
    }
    /// a client that stopped sending, or lost its network, releases its keys after half a second
    if (std::chrono::steady_clock::now() - received_ > std::chrono::milliseconds(500)) return state;
    state.forward = std::clamp(forward_, -1.0, 1.0);
    state.turn = std::clamp(turn_, -1.0, 1.0);
    state.reset = resetHeld_ && !resetWasHeld_;
    resetWasHeld_ = resetHeld_;
    return state;
  }

 private:
  int socket_ = -1;
  double forward_ = 0.0, turn_ = 0.0;
  bool resetHeld_ = false, resetWasHeld_ = false, announced_ = false;
  std::chrono::steady_clock::time_point received_{};
};

int terrainIndex(const std::string& name) {
  const auto it = std::find(kTerrainNames.begin(), kTerrainNames.end(), name);
  RSFATAL_IF(it == kTerrainNames.end(), "no terrain named " << name << "; flat, stairs, steps, holes or hurdles")
  return int(it - kTerrainNames.begin());
}

/// --headless <terrain> <vx> <wz> <seconds> [level] [policy index]: drives without a server and prints the tracking.
int runHeadless(raisim::World& world, RaiwayDemo& demo, int argc, char* argv[]) {
  RSFATAL_IF(argc < 6, "usage: raiway_demo --headless <terrain> <vx> <wz> <seconds> [level] [policy index]")
  const int terrain = terrainIndex(argv[2]);
  const double vx = std::stod(argv[3]), wz = std::stod(argv[4]), seconds = std::stod(argv[5]);
  const double level = argc > 6 ? std::stod(argv[6]) : 1.0;
  const int policyIndex = argc > 7 ? std::stoi(argv[7]) : 0;
  demo.setPolicy(policyIndex);
  demo.setTerrain(terrain, level);
  const int substeps = demo.substeps();
  const int steps = int(std::lround(seconds / demo.policy().conDt));
  double sumVx = 0.0, sumWz = 0.0;
  int falls = 0, resets = 0;
  for (int step = 0; step < steps; step++) {
    demo.control(vx, wz);
    for (int k = 0; k < substeps; k++) {
      demo.applyPd();
      world.integrate();
    }
    sumVx += demo.bodyLinVel()[0];
    sumWz += demo.bodyAngVel()[2];
    if ((step + 1) % int(std::lround(1.0 / demo.policy().conDt)) == 0) {
      const auto& gc = demo.robot()->getGeneralizedCoordinate();
      const double yaw = std::atan2(2.0 * (gc[3] * gc[6] + gc[4] * gc[5]), 1.0 - 2.0 * (gc[5] * gc[5] + gc[6] * gc[6]));
      std::cout << "t " << world.getWorldTime() << " s: x " << gc[0] << " m, y " << gc[1] << " m, yaw " << yaw << " rad, vx " << demo.bodyLinVel()[0] << " m/s" << std::endl;
    }
    if (demo.checkEpisode()) {
      resets++;
      falls += demo.lastResetWasFall();
    }
  }
  std::cout << kTerrainNames[terrain] << " level " << level << ", policy " << demo.policy().label << ", command (" << vx << ", " << wz
            << "): mean vx " << sumVx / steps << " m/s, mean wz " << sumWz / steps << " rad/s, x " << demo.robot()->getGeneralizedCoordinate()[0]
            << " m, falls " << falls << ", resets " << resets << " in " << seconds << " s" << std::endl;
  return 0;
}

}  // namespace

int main(int argc, char* argv[]) {
  const std::string rsc = exampleRscPath(argv[0], "raiway");
  const bool headless = argc > 1 && std::string(argv[1]) == "--headless";
  bool lan = false;
  int port = 8080, keyPort = 8081;
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--lan") lan = true;
    else if (std::string(argv[i]) == "--port" && i + 1 < argc) port = std::stoi(argv[++i]);
    else if (std::string(argv[i]) == "--key-port" && i + 1 < argc) keyPort = std::stoi(argv[++i]);
  }
  raisim::World world(rsc + "/raiway.rscene");
  if (headless) {
    RaiwayDemo demo(rsc, world);
    return runHeadless(world, demo, argc, argv);
  }

  raisim::RaisimServer server(&world);
  RaiwayDemo demo(rsc, world, [&server](bool locked) {
    if (locked) server.lockVisualizationServerMutex();
    else server.unlockVisualizationServerMutex();
  });
  std::string policyNames;
  for (size_t i = 0; i < demo.policies().size(); i++) {
    std::cout << "policy " << i << ": " << demo.policies()[i].label << " (" << demo.policies()[i].name << ")" << std::endl;
    policyNames += (i ? ", " : "") + std::to_string(i) + ": " + demo.policies()[i].label;
  }
  const size_t vxSlider = server.addSlider("Forward speed vx [m/s]", -1.5, 1.5, 0.0);
  const size_t wzSlider = server.addSlider("Yaw rate wz [rad/s]", -2.0, 2.0, 0.0);
  const size_t keyVxSlider = server.addSlider("Key forward speed [m/s]", 0.0, 1.5, 1.0);
  const size_t keyWzSlider = server.addSlider("Key yaw rate [rad/s]", 0.0, 2.0, 1.5);
  const size_t terrainSlider = server.addSlider("Terrain (0 flat, 1 stairs, 2 steps, 3 holes, 4 hurdles)", 0.0, 4.0, 1.0);
  const size_t levelSlider = server.addSlider("Terrain level", 0.0, 1.0, 1.0);
  const size_t policySlider = server.addSlider("Policy (" + policyNames + ")", 0.0, double(demo.policies().size() - 1), 0.0);
  const size_t resetSlider = server.addSlider("Reset (move to reset)", 0.0, 1.0, 0.0);
  server.setBindLoopbackOnly(!lan);
  server.launchServer(port);
  server.focusOn(demo.robot());
  raisim_examples::warnIfNoClientConnected(server);
  auto* scanVisual = server.addInstancedVisuals("scan points", raisim::Shape::Sphere, {0.03, 0.03, 0.03}, {1.0, 0.4, 0.1, 1.0}, {1.0, 0.4, 0.1, 1.0});
  scanVisual->resize(demo.scanPoints().size());

  using Clock = std::chrono::steady_clock;
  int applied[3] = {1, 100, 0};
  int pending[3] = {1, 100, 0};
  Clock::time_point changed = Clock::now();
  double lastReset = server.getSlider(resetSlider);
  HeldKeys keys;
  KeyLink keyLink(keyPort);
  int tick = 0;
  bool controlDue = true;
  for (;;) {
    RS_TIMED_LOOP(int(1e6 * world.getTimeStep()))
    /// a held key, local or remote, overrides the vx/wz sliders; released, the sliders' command returns
    HeldKeys::State held = keys.poll();
    const HeldKeys::State remote = keyLink.poll();
    if (remote.forward != 0.0 || remote.turn != 0.0) held = remote;
    double vx = server.getSlider(vxSlider), wz = server.getSlider(wzSlider);
    if (held.forward != 0.0 || held.turn != 0.0) {
      vx = held.forward * server.getSlider(keyVxSlider);
      wz = held.turn * server.getSlider(keyWzSlider);
    }
    if (held.reset || remote.reset) demo.reset();
    /// a dragged slider settles for half a second before its terrain, level or policy is rebuilt
    const int wanted[3] = {int(std::lround(server.getSlider(terrainSlider))), int(std::lround(100.0 * server.getSlider(levelSlider))),
                           int(std::lround(server.getSlider(policySlider)))};
    for (int i = 0; i < 3; i++)
      if (wanted[i] != pending[i]) { pending[i] = wanted[i]; changed = Clock::now(); }
    if (Clock::now() - changed > std::chrono::milliseconds(500)) {
      if (pending[2] != applied[2]) {
        applied[2] = pending[2];
        demo.setPolicy(applied[2]);
        scanVisual->resize(demo.scanPoints().size());
        std::cout << "policy " << demo.policy().label << std::endl;
      }
      if (pending[0] != applied[0] || pending[1] != applied[1]) {
        applied[0] = pending[0];
        applied[1] = pending[1];
        demo.setTerrain(applied[0], applied[1] / 100.0);
        std::cout << kTerrainNames[applied[0]] << " at level " << applied[1] / 100.0 << std::endl;
      }
    }
    const double resetValue = server.getSlider(resetSlider);
    if (resetValue != lastReset) {
      lastReset = resetValue;
      demo.reset();
    }
    if (controlDue) {
      demo.control(vx, wz);
      for (size_t i = 0; i < demo.scanPoints().size(); i++) scanVisual->setPosition(i, demo.scanPoints()[i]);
      controlDue = false;
    }
    demo.applyPd();
    const double before = world.getWorldTime();
    server.integrateWorldThreadSafe();
    if (world.getWorldTime() == before) continue;
    if (++tick % demo.substeps() == 0) {
      demo.checkEpisode();
      controlDue = true;
    }
  }
}
