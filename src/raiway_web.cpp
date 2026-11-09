// Raiway in the browser: RaiSim and the policy run in WebAssembly on the page and Rayrai draws them on its canvas.
// The panel and held arrow/WASD keys give the velocity command; the panel also chooses terrain, level and policy.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "raiway_core.hpp"
#include "rayrai/PhysicsSnapshot.hpp"
#include "rayrai/example_common.hpp"

using namespace raiway;

namespace {

/// a stalled tab or a slow frame costs simulated time instead of making the next frame catch up
constexpr double kMaxStepPerFrame = 0.1;

struct Controls {
  float vx = 0.0f, wz = 0.0f, keyVx = 1.0f, keyWz = 1.5f, levelSlider = 1.0f, level = 1.0f;
  int terrain = 1, policy = 0;
  bool paused = false, showScan = true;
};

/// Level edits reach c.level only once the slider is released, since every change rebuilds the terrain.
void drawPanel(Controls& c, const RaiwayDemo& demo, bool& reset) {
  ImGui::SetNextWindowPos(ImVec2(12, 12), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowBgAlpha(0.85f);
  ImGui::Begin("Raiway", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::SliderFloat("Forward speed vx [m/s]", &c.vx, -1.5f, 1.5f, "%.2f");
  ImGui::SliderFloat("Yaw rate wz [rad/s]", &c.wz, -2.0f, 2.0f, "%.2f");
  ImGui::SliderFloat("Key forward speed [m/s]", &c.keyVx, 0.0f, 1.5f, "%.2f");
  ImGui::SliderFloat("Key yaw rate [rad/s]", &c.keyWz, 0.0f, 2.0f, "%.2f");
  ImGui::Separator();
  ImGui::Combo("Terrain", &c.terrain, "flat\0stairs\0steps\0holes\0hurdles\0");
  ImGui::SliderFloat("Terrain level", &c.levelSlider, 0.0f, 1.0f, "%.2f");
  if (ImGui::IsItemDeactivatedAfterEdit()) c.level = c.levelSlider;
  if (ImGui::BeginCombo("Policy", demo.policies()[c.policy].label.c_str())) {
    for (int i = 0; i < int(demo.policies().size()); i++)
      if (ImGui::Selectable(demo.policies()[i].label.c_str(), i == c.policy)) c.policy = i;
    ImGui::EndCombo();
  }
  reset |= ImGui::Button("Reset (R)");
  ImGui::SameLine();
  ImGui::Checkbox("Pause", &c.paused);
  ImGui::SameLine();
  ImGui::Checkbox("Height scan", &c.showScan);
  ImGui::Separator();
  ImGui::Text("body vx %+.2f m/s, wz %+.2f rad/s", demo.bodyLinVel()[0], demo.bodyAngVel()[2]);
  ImGui::TextDisabled("Hold arrows or W/A/S/D to drive, R resets.\nDrag to orbit the camera, wheel to zoom.\n"
                      "Orange spheres: the policy's height scan.");
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
    bool controlDue = true;
    int tick = 0;
    double owed = 0.0;
    using Clock = std::chrono::steady_clock;
    auto last = Clock::now();
    while (!app.quit) {
      app.processEvents();
      if (app.quit) break;
      const auto now = Clock::now();
      owed += std::min(kMaxStepPerFrame, std::chrono::duration<double>(now - last).count());
      last = now;
      app.beginFrame();
      const auto viewport = app.beginViewport();

      double vx = controls.vx, wz = controls.wz;
      bool reset = false;
      if (!ImGui::GetIO().WantTextInput) {
        const auto down = [](ImGuiKey a, ImGuiKey b) { return ImGui::IsKeyDown(a) || ImGui::IsKeyDown(b); };
        const double forward = double(down(ImGuiKey_UpArrow, ImGuiKey_W)) - double(down(ImGuiKey_DownArrow, ImGuiKey_S));
        const double turn = double(down(ImGuiKey_LeftArrow, ImGuiKey_A)) - double(down(ImGuiKey_RightArrow, ImGuiKey_D));
        if (forward != 0.0 || turn != 0.0) {
          vx = forward * controls.keyVx;
          wz = turn * controls.keyWz;
        }
        reset = ImGui::IsKeyPressed(ImGuiKey_R, false);
      }

      if (controls.paused) owed = 0.0;
      for (; owed >= world.getTimeStep(); owed -= world.getTimeStep()) {
        if (controlDue) {
          demo.control(vx, wz);
          controlDue = false;
        }
        demo.applyPd();
        world.integrate();
        if (++tick % demo.substeps() == 0) {
          demo.checkEpisode();
          controlDue = true;
        }
      }

      viewer.applyPhysicsSnapshot(snapshots.capture(world));
      const auto& points = demo.scanPoints();
      const size_t shown = controls.showScan ? points.size() : 0;
      if (scan->count() != shown) scan->resize(shown);
      for (size_t i = 0; i < shown; i++) scan->setPosition(i, glm::vec3(points[i].x(), points[i].y(), points[i].z()));
      app.endViewport(viewer, viewport, true, false);

      drawPanel(controls, demo, reset);
      drawPolicyNote(demo.policy());
      const bool terrainChanged = controls.terrain != applied.terrain || controls.level != applied.level;
      if (controls.policy != applied.policy) demo.setPolicy(controls.policy);
      if (terrainChanged) demo.setTerrain(controls.terrain, controls.level);
      else if (reset) demo.reset();
      /// a new episode starts on a control step, as in training
      if (terrainChanged || reset || controls.policy != applied.policy) {
        controlDue = true;
        tick = 0;
      }
      applied = controls;
      app.endFrame();
    }
  }
  app.shutdown();
  return 0;
}
