// Raiway in the browser: RaiSim and the policy run in WebAssembly on the page and Rayrai draws them on its canvas.
// The panel and held arrow/WASD keys give the velocity command; the panel also chooses terrain, level and policy.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "raiway_core.hpp"
#include "rayrai/PhysicsSnapshot.hpp"
#include "rayrai/example_common.hpp"

using namespace raiway;

namespace {

/// a stalled tab or a slow frame costs simulated time instead of making the next frame catch up
constexpr double kMaxStepPerFrame = 0.1;
constexpr double kPlotSeconds = 5.0;
constexpr float kPlotWidth = 190.0f, kPlotHeight = 44.0f, kRegionHeight = 100.0f;
const std::array<std::pair<const char*, ImU32>, 3> kJointGroups{
    {{"hip", IM_COL32(70, 130, 245, 255)}, {"knee", IM_COL32(235, 95, 65, 255)}, {"wheel", IM_COL32(60, 190, 110, 255)}}};

struct Controls {
  float vx = 0.0f, wz = 0.0f, keyVx = 1.0f, keyWz = 1.5f, levelSlider = 1.0f, level = 1.0f;
  int terrain = 1, policy = 0;
  bool paused = false, showScan = true;
};

struct Readout {
  double vx = 0.0, wz = 0.0, realTime = 1.0;
};

/// The last kPlotSeconds of joint velocities (rows 0-5) and torques (rows 6-11), one sample per control step.
struct JointHistory {
  int length = 2, head = 0, count = 0;
  std::vector<std::array<float, 2 * kJoints>> samples;

  void clear(double controlDt) {
    length = std::max(2, int(std::lround(kPlotSeconds / controlDt)));
    samples.assign(length, {});
    head = count = 0;
  }
  void record(const Eigen::VectorXd& velocity, const Eigen::VectorXd& torque) {
    for (int j = 0; j < kJoints; j++) {
      samples[head][j] = float(velocity[j]);
      samples[head][kJoints + j] = float(torque[j]);
    }
    head = (head + 1) % length;
    count = std::min(count + 1, length);
  }
  float at(int row, int k) const { return samples[(head - count + k + length) % length][row]; }
};

/// The left joint solid and the right one faint, as in raiway_demo_mujoco; a zero range scales to the data.
void plotJoint(const char* title, const JointHistory& h, int row, ImU32 color, const char* unit, double range) {
  ImGui::BeginGroup();
  ImGui::TextUnformatted(title);
  const ImVec2 p = ImGui::GetCursorScreenPos();
  ImGui::Dummy(ImVec2(kPlotWidth, kPlotHeight));
  if (range <= 0.0) {
    range = 1.0;
    for (int k = 0; k < h.count; k++) range = std::max({range, std::abs(double(h.at(row, k))), std::abs(double(h.at(row + 3, k)))});
    range = std::ceil(range);
  }
  ImDrawList* draw = ImGui::GetWindowDrawList();
  const float mid = p.y + 0.5f * kPlotHeight;
  draw->AddRectFilled(p, ImVec2(p.x + kPlotWidth, p.y + kPlotHeight), IM_COL32(255, 255, 255, 14), 3.0f);
  draw->AddLine(ImVec2(p.x, mid), ImVec2(p.x + kPlotWidth, mid), IM_COL32(255, 255, 255, 40));
  std::vector<ImVec2> line(h.count);
  for (int side = 0; side < 2; side++) {
    for (int k = 0; k < h.count; k++) {
      const double value = std::clamp(h.at(row + 3 * side, k) / range, -1.0, 1.0);
      line[k] = ImVec2(p.x + kPlotWidth * float(h.length - h.count + k) / float(h.length - 1), mid - 0.5f * kPlotHeight * float(value));
    }
    draw->AddPolyline(line.data(), h.count, side == 0 ? color : (color & 0x00FFFFFF) | IM_COL32(0, 0, 0, 100), ImDrawFlags_None, 1.5f);
  }
  char scale[32];
  std::snprintf(scale, sizeof(scale), "\xC2\xB1%.0f %s", range, unit);
  draw->AddText(ImGui::GetFont(), 0.85f * ImGui::GetFontSize(), ImVec2(p.x + 3.0f, p.y + 1.0f), IM_COL32(255, 255, 255, 120), scale);
  ImGui::EndGroup();
}

/// One joint pair in its torque-speed plane: the URDF effort and velocity limits as the region, older points fainter.
void plotRegion(const char* title, const JointHistory& h, int joint, ImU32 color, double speedLimit, double torqueLimit, float width) {
  /// RaiSim clips the torque to the limit, so a point on it is a saturated motor
  const auto atLimit = [&](double torque) { return std::abs(torque) >= 0.99 * torqueLimit; };
  int saturated = 0;
  for (int side = 0; side < 2; side++)
    for (int k = 0; k < h.count; k++) saturated += atLimit(h.at(kJoints + joint + 3 * side, k));
  ImGui::BeginGroup();
  ImGui::Text("%s  sat %.0f%%", title, h.count ? 50.0 * saturated / h.count : 0.0);
  const ImVec2 p = ImGui::GetCursorScreenPos();
  ImGui::Dummy(ImVec2(width, kRegionHeight));
  if (speedLimit <= 0.0)
    for (int k = 0; k < h.count; k++) speedLimit = std::max({speedLimit, std::abs(double(h.at(joint, k))), std::abs(double(h.at(joint + 3, k))), 1.0});
  const double xRange = 1.2 * speedLimit, yRange = 1.2 * torqueLimit;
  const auto toScreen = [&](double speed, double torque) {
    return ImVec2(p.x + 0.5f * width * (1.0f + float(std::clamp(speed / xRange, -1.0, 1.0))),
                  p.y + 0.5f * kRegionHeight * (1.0f - float(std::clamp(torque / yRange, -1.0, 1.0))));
  };
  ImDrawList* draw = ImGui::GetWindowDrawList();
  draw->AddRectFilled(p, ImVec2(p.x + width, p.y + kRegionHeight), IM_COL32(255, 255, 255, 14), 3.0f);
  draw->AddLine(toScreen(-xRange, 0.0), toScreen(xRange, 0.0), IM_COL32(255, 255, 255, 40));
  draw->AddLine(toScreen(0.0, -yRange), toScreen(0.0, yRange), IM_COL32(255, 255, 255, 40));
  const ImVec2 lo = toScreen(-speedLimit, torqueLimit), hi = toScreen(speedLimit, -torqueLimit);
  draw->AddRectFilled(lo, hi, IM_COL32(70, 190, 105, 35));
  draw->AddRect(lo, hi, IM_COL32(95, 215, 125, 200));
  for (int side = 0; side < 2; side++)
    for (int k = 0; k < h.count; k++) {
      const double speed = h.at(joint + 3 * side, k), torque = h.at(kJoints + joint + 3 * side, k);
      const int alpha = (side == 0 ? 50 : 25) + (side == 0 ? 200 : 100) * k / std::max(1, h.count);
      const ImU32 rgb = atLimit(torque) ? IM_COL32(255, 160, 40, 0) : (color & 0x00FFFFFF);
      const ImVec2 q = toScreen(speed, torque);
      draw->AddRectFilled(ImVec2(q.x - 1.2f, q.y - 1.2f), ImVec2(q.x + 1.2f, q.y + 1.2f), rgb | IM_COL32(0, 0, 0, alpha));
    }
  char text[48];
  const float small = 0.85f * ImGui::GetFontSize();
  const ImU32 label = IM_COL32(255, 255, 255, 130);
  std::snprintf(text, sizeof(text), "\xC2\xB1%.0f N\xC2\xB7m", torqueLimit);
  draw->AddText(ImGui::GetFont(), small, ImVec2(lo.x + 2.0f, lo.y + 1.0f), label, text);
  std::snprintf(text, sizeof(text), "\xC2\xB1%.0f rad/s", speedLimit);
  draw->AddText(ImGui::GetFont(), small, ImVec2(hi.x - ImGui::CalcTextSize(text).x * 0.85f - 2.0f, hi.y - small - 1.0f), label, text);
  ImGui::EndGroup();
}

/// Level edits reach c.level only once the slider is released, since every change rebuilds the terrain.
void drawPanel(Controls& c, const RaiwayDemo& demo, const Readout& r, const JointHistory& h, bool& reset) {
  ImGui::SetNextWindowPos(ImVec2(12, 12), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowBgAlpha(0.85f);
  ImGui::Begin("Raiway", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::TextUnformatted("Policy");
  ImGui::SetNextItemWidth(2.0f * kPlotWidth + ImGui::GetStyle().ItemSpacing.x);
  if (ImGui::BeginCombo("##policy", demo.policies()[c.policy].label.c_str())) {
    for (int i = 0; i < int(demo.policies().size()); i++)
      if (ImGui::Selectable(demo.policies()[i].label.c_str(), i == c.policy)) c.policy = i;
    ImGui::EndCombo();
  }
  ImGui::Combo("Terrain", &c.terrain, "flat\0stairs\0steps\0holes\0hurdles\0");
  ImGui::SliderFloat("Terrain level", &c.levelSlider, 0.0f, 1.0f, "%.2f");
  if (ImGui::IsItemDeactivatedAfterEdit()) c.level = c.levelSlider;
  ImGui::Separator();
  ImGui::SliderFloat("Forward speed vx [m/s]", &c.vx, -1.5f, 1.5f, "%.2f");
  ImGui::SliderFloat("Yaw rate wz [rad/s]", &c.wz, -2.0f, 2.0f, "%.2f");
  ImGui::SliderFloat("Key forward speed [m/s]", &c.keyVx, 0.0f, 1.5f, "%.2f");
  ImGui::SliderFloat("Key yaw rate [rad/s]", &c.keyWz, 0.0f, 2.0f, "%.2f");
  reset |= ImGui::Button("Reset (R)");
  ImGui::SameLine();
  ImGui::Checkbox("Pause", &c.paused);
  ImGui::SameLine();
  ImGui::Checkbox("Height scan", &c.showScan);
  ImGui::Separator();
  ImGui::Text("Command    vx %+.2f m/s   wz %+.2f rad/s", r.vx, r.wz);
  ImGui::Text("Measured   vx %+.2f m/s   wz %+.2f rad/s", demo.bodyLinVel()[0], demo.bodyAngVel()[2]);
  ImGui::Text("Real-time  %.2fx", r.realTime);
  const float fullWidth = 2.0f * kPlotWidth + ImGui::GetStyle().ItemSpacing.x;
  if (ImGui::CollapsingHeader("Joint Vel & Torque", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::TextDisabled("last 5 s, left leg solid, right leg faint");
    for (int j = 0; j < 3; j++) {
      const std::string name = kJointGroups[j].first;
      plotJoint((name + " velocity").c_str(), h, j, kJointGroups[j].second, "rad/s", 0.0);
      ImGui::SameLine();
      plotJoint((name + " torque").c_str(), h, kJoints + j, kJointGroups[j].second, "N\xC2\xB7m", demo.torqueLimit());
    }
  }
  if (ImGui::CollapsingHeader("Motor Operating Region", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::TextDisabled("torque vs speed, last 5 s, green: URDF limits\norange: at the torque limit");
    const float width = (fullWidth - 2.0f * ImGui::GetStyle().ItemSpacing.x) / 3.0f;
    for (int j = 0; j < 3; j++) {
      if (j) ImGui::SameLine();
      plotRegion(kJointGroups[j].first, h, j, kJointGroups[j].second, demo.speedLimit(), demo.torqueLimit(), width);
    }
  }
  ImGui::TextDisabled("Hold arrows or W/A/S/D to drive, R resets.\nDrag to orbit the camera, wheel to zoom.");
  ImGui::End();
}

/// bottom left, since the page's license notice takes the bottom right
void drawPolicyNote(const Policy& p) {
  const std::string text = "policy " + p.label + "  (" + p.name + ")";
  const float size = 0.85f * ImGui::GetFontSize();
  const ImVec2 at(12.0f, ImGui::GetIO().DisplaySize.y - size - 10.0f);
  ImGui::GetForegroundDrawList()->AddText(ImGui::GetFont(), size, at, IM_COL32(25, 25, 30, 140), text.c_str());
}

}  // namespace

int main() {
  const std::string rsc = RAIWAY_RSC;
  raisim::World world(rsc + "/raiway.rscene");
  RaiwayDemo demo(rsc, world);

  ExampleApp app;
  if (!app.init("Raiway on RaiSim", 1280, 720)) return 1;
  ImGui::GetIO().IniFilename = nullptr;
  {
    /// the renderer owns an empty world and sees the simulation only through copied snapshots
    raisin::PhysicsSnapshotCapture snapshots;
    raisin::RayraiWindow viewer(std::make_shared<raisim::World>(), 1280, 720);
    viewer.setRenderQualitySettings(raisin::RayraiWindow::defaultRenderQualitySettings(raisin::RayraiWindow::RenderQualityPreset::Balanced));
    auto& camera = viewer.getCamera();
    camera.position = {-2.0f, -2.6f, 1.9f};
    camera.yaw = 52.0f;
    camera.pitch = -22.0f;
    camera.setTargetObject(demo.robot());

    auto scan = viewer.addInstancedVisuals("scan points", raisim::Shape::Sphere, {0.03f, 0.03f, 0.03f}, {1.0f, 0.4f, 0.1f, 1.0f},
                                           {1.0f, 0.4f, 0.1f, 1.0f});
    Controls controls, applied;
    Readout readout;
    JointHistory history;
    history.clear(demo.policy().conDt);
    bool controlDue = true;
    int tick = 0;
    double owed = 0.0, windowWall = 0.0, windowSim = 0.0;
    using Clock = std::chrono::steady_clock;
    auto last = Clock::now();
    while (!app.quit) {
      app.processEvents();
      if (app.quit) break;
      const auto now = Clock::now();
      const double elapsed = std::chrono::duration<double>(now - last).count();
      owed += std::min(kMaxStepPerFrame, elapsed);
      last = now;
      app.beginFrame();
      const auto viewport = app.beginViewport();

      readout.vx = controls.vx;
      readout.wz = controls.wz;
      bool reset = false;
      if (!ImGui::GetIO().WantTextInput) {
        const auto down = [](ImGuiKey a, ImGuiKey b) { return ImGui::IsKeyDown(a) || ImGui::IsKeyDown(b); };
        const double forward = double(down(ImGuiKey_UpArrow, ImGuiKey_W)) - double(down(ImGuiKey_DownArrow, ImGuiKey_S));
        const double turn = double(down(ImGuiKey_LeftArrow, ImGuiKey_A)) - double(down(ImGuiKey_RightArrow, ImGuiKey_D));
        if (forward != 0.0 || turn != 0.0) {
          readout.vx = forward * controls.keyVx;
          readout.wz = turn * controls.keyWz;
        }
        reset = ImGui::IsKeyPressed(ImGuiKey_R, false);
      }

      if (controls.paused) owed = 0.0;
      const double simStart = world.getWorldTime();
      for (; owed >= world.getTimeStep(); owed -= world.getTimeStep()) {
        if (controlDue) {
          demo.control(readout.vx, readout.wz);
          controlDue = false;
        }
        demo.applyPd();
        world.integrate();
        if (++tick % demo.substeps() == 0) {
          history.record(demo.jointVelocity(), demo.jointTorque());
          if (demo.checkEpisode()) history.clear(demo.policy().conDt);
          controlDue = true;
        }
      }
      windowSim += world.getWorldTime() - simStart;
      windowWall += elapsed;
      if (windowWall >= 0.5) {
        readout.realTime = windowSim / windowWall;
        windowSim = windowWall = 0.0;
      }

      viewer.applyPhysicsSnapshot(snapshots.capture(world));
      const auto& points = demo.scanPoints();
      const size_t shown = controls.showScan ? points.size() : 0;
      if (scan->count() != shown) scan->resize(shown);
      for (size_t i = 0; i < shown; i++) scan->setPosition(i, glm::vec3(points[i].x(), points[i].y(), points[i].z()));
      app.endViewport(viewer, viewport, true, false);

      drawPanel(controls, demo, readout, history, reset);
      drawPolicyNote(demo.policy());
      const bool terrainChanged = controls.terrain != applied.terrain || controls.level != applied.level;
      if (controls.policy != applied.policy) demo.setPolicy(controls.policy);
      if (terrainChanged) demo.setTerrain(controls.terrain, controls.level);
      else if (reset) demo.reset();
      /// a new episode starts on a control step, as in training
      if (terrainChanged || reset || controls.policy != applied.policy) {
        controlDue = true;
        tick = 0;
        history.clear(demo.policy().conDt);
      }
      applied = controls;
      app.endFrame();
    }
  }
  app.shutdown();
  return 0;
}
