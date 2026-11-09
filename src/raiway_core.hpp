// What raiway_demo and the browser build share: the terrains, the exported policies and the controller
// that runs them on a RaiSim world as the training environment does.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "raisim/World.hpp"

namespace raiway {

constexpr double kGrid = 0.02, kX0 = -3.0, kX1 = 13.0, kY0 = -4.0, kY1 = 4.0;
constexpr int kNx = 801, kNy = 401;
constexpr double kStairMaxSlope = 35.0 * M_PI / 180.0, kSpawnHalfLength = 0.75, kHoleLaneWidth = 3.0;
constexpr double kWheelRadius = 0.254, kTireOffset = 0.09745, kWheelHalfWidth = 0.026, kBaseHeight = 0.814;
constexpr int kJoints = 6, kObHead = 28;
constexpr int kEstHead = 16, kEstTailStart = 28, kEstLen = 4, kEstStride = 2, kEstDepth = 1 + (kEstLen - 1) * kEstStride;
constexpr double kScanLookMin = 0.2, kScanFovMax = 1.0, kScanLookBack = 0.2, kScanLookTime = 0.8;
constexpr double kRingSpacingX = 0.2, kRingSpacingY = 0.1;
constexpr std::array<int, 4> kRings{1, 6, 11, 16};
constexpr double kFallHoldTime = 1.0;
const std::vector<std::string> kTerrainNames{"flat", "stairs", "steps", "holes", "hurdles"};

double xAt(int i) { return kX0 + i * kGrid; }
double yAt(int j) { return kY0 + j * kGrid; }

/// Heights on the training grid plus the full-width boxes training buries in riser faces and hurdles.
struct Terrain {
  struct Box { double x, z, thickness, height; };
  std::vector<double> h = std::vector<double>(kNx * kNy, 0.0);
  std::vector<Box> boxes;
  double& at(int x, int y) { return h[y * kNx + x]; }
};

Terrain stairs(double level) {
  Terrain t;
  const double h = 0.02 + 0.28 * level;
  const double w = std::max(h / std::tan(kStairMaxSlope), 0.3) + 0.1;
  const int n = 8;
  const double downStart = kSpawnHalfLength + n * w + 2.0;
  for (int i = 0; i < kNx; i++) {
    const double x = xAt(i);
    double k = std::clamp(std::floor((x - kSpawnHalfLength) / w) + 1.0, 0.0, double(n));
    if (x >= downStart) k = std::clamp(n - std::floor((x - downStart) / w) - 1.0, 0.0, double(n));
    for (int j = 0; j < kNy; j++) t.at(i, j) = k * h;
  }
  for (int m = 0; m < n; m++) {
    t.boxes.push_back({kSpawnHalfLength + m * w, (m + 0.5) * h + 1e-4, 0.04, h});
    t.boxes.push_back({downStart + m * w, (n - m - 0.5) * h + 1e-4, 0.04, h});
  }
  return t;
}

Terrain steps(double level, unsigned seed) {
  Terrain t;
  const double maxHeight = 0.05 + 0.25 * level;
  const int cells = int(0.4 / kGrid) + 1;
  const int bx = (kNx + cells - 1) / cells, by = (kNy + cells - 1) / cells;
  std::mt19937 gen(seed + 11);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  std::vector<double> blocks(bx * by);
  for (auto& b : blocks) b = maxHeight * u(gen);
  for (int j = 0; j < kNy; j++)
    for (int i = 0; i < kNx; i++) t.at(i, j) = blocks[(j / cells) * bx + i / cells];
  return t;
}

Terrain holes(double level, unsigned seed) {
  Terrain t;
  const double depth = 0.05 + 0.75 * level * level;
  std::mt19937 gen(seed);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  std::array<std::vector<double>, 2> lanes;
  for (auto& lane : lanes) {
    lane.assign(kNx, -depth);
    double start = kX0 - 0.6 + 1.3 * u(gen);
    while (start < kX1) {
      const double end = start + 0.3 + 0.3 * u(gen);
      const double landing = -0.1 + 0.2 * u(gen);
      for (int i = 0; i < kNx; i++)
        if (xAt(i) >= start - 1e-9 && xAt(i) <= end + 1e-9) lane[i] = landing;
      start = end + 0.4 + 0.3 * u(gen);
    }
  }
  for (int j = 0; j < kNy; j++) {
    const double y = yAt(j);
    for (int i = 0; i < kNx; i++) {
      double v = y > 0 ? lanes[0][i] : lanes[1][i];
      if (std::abs(y) <= kGrid / 2 + 1e-9) v = std::min(lanes[0][i], lanes[1][i]);
      if (std::abs(y) > kHoleLaneWidth) v = -depth;
      if (std::abs(xAt(i)) <= kSpawnHalfLength && std::abs(y) <= kHoleLaneWidth) v = 0.0;
      t.at(i, j) = v;
    }
  }
  return t;
}

Terrain hurdles(double level) {
  Terrain t;
  const double height = 0.02 + 0.28 * level, thickness = 0.05, spacing = 1.5, start = 1.0;
  for (int i = 0; i < kNx; i++) {
    const double x = xAt(i);
    const double phase = std::fmod(x - start + thickness / 2, spacing);
    if (x >= start - thickness / 2 && phase < thickness)
      for (int j = 0; j < kNy; j++) t.at(i, j) = height;
  }
  for (double x = start; x < kX1; x += spacing) t.boxes.push_back({x, height / 2 + 0.001, thickness, height});
  return t;
}

Terrain makeTerrain(int type, double level) {
  switch (type) {
    case 1: return stairs(level);
    case 2: return steps(level, 0);
    case 3: return holes(level, 0);
    case 4: return hurdles(level);
    default: return Terrain();
  }
}

struct Mlp {
  std::vector<Eigen::MatrixXd> W;
  std::vector<Eigen::VectorXd> b;
  Eigen::VectorXd operator()(Eigen::VectorXd x) const {
    for (size_t i = 0; i < W.size(); i++) {
      x = W[i] * x + b[i];
      if (i + 1 < W.size()) x = x.unaryExpr([](double v) { return v > 0.0 ? v : 0.01 * v; });
    }
    return x;
  }
};

/// One exported checkpoint: normalization, actor and the optional estimator, read from policy.txt.
struct Policy {
  std::string name, label;
  double simDt = 0.0025, conDt = 0.01;
  int obDim = 0, scanType = 0, scanN = 0, estDim = 0, estInputDim = 0;
  Eigen::VectorXd obMean, obStd, bound, estInMean, estInStd, estOutMean, estOutStd;
  Mlp actor, estimator;
};

Policy loadPolicy(const std::string& path) {
  std::ifstream in(path);
  RSFATAL_IF(!in, "cannot open " << path)
  auto expect = [&](const char* key) {
    std::string s;
    in >> s;
    RSFATAL_IF(s != key, path << ": expected " << key << ", got " << s)
  };
  auto readVec = [&](const char* key, int n) {
    expect(key);
    Eigen::VectorXd v(n);
    for (int i = 0; i < n; i++) in >> v[i];
    return v;
  };
  auto readMlp = [&]() {
    expect("layers");
    int layers;
    in >> layers;
    Mlp m;
    for (int l = 0; l < layers; l++) {
      expect("linear");
      int ni, no;
      in >> ni >> no;
      Eigen::MatrixXd W(no, ni);
      for (int r = 0; r < no; r++)
        for (int c = 0; c < ni; c++) in >> W(r, c);
      Eigen::VectorXd b(no);
      for (int r = 0; r < no; r++) in >> b[r];
      m.W.push_back(W);
      m.b.push_back(b);
    }
    return m;
  };
  Policy p;
  int version;
  expect("raiway_policy");
  in >> version;
  RSFATAL_IF(version != 1, path << ": unknown policy file version " << version)
  std::string checkpoint;
  expect("checkpoint");
  in >> checkpoint;
  expect("simulation_dt");
  in >> p.simDt;
  expect("control_dt");
  in >> p.conDt;
  expect("ob_dim");
  in >> p.obDim;
  expect("scan_type");
  in >> p.scanType;
  expect("est_dim");
  in >> p.estDim;
  p.scanN = (p.obDim - kObHead) / 2;
  p.obMean = readVec("ob_mean", p.obDim);
  p.obStd = readVec("ob_std", p.obDim);
  expect("action_bound");
  int bounds;
  in >> bounds;
  p.bound.resize(bounds);
  for (int i = 0; i < bounds; i++) in >> p.bound[i];
  expect("actor");
  p.actor = readMlp();
  expect("estimator");
  in >> p.estInputDim;
  if (p.estInputDim) {
    p.estInMean = readVec("est_in_mean", p.estInputDim);
    p.estInStd = readVec("est_in_std", p.estInputDim);
    p.estimator = readMlp();
    p.estOutMean = readVec("est_out_mean", p.estDim);
    p.estOutStd = readVec("est_out_std", p.estDim);
  }
  RSFATAL_IF(!in, path << ": truncated")
  RSFATAL_IF(p.actor.W.front().cols() != p.obDim + 2 + p.estDim, path << ": actor input width disagrees with ob_dim and est_dim")
  return p;
}

/// Lines of policies.txt: "default<TAB>name" then "name<TAB>label" per exported policy.
std::vector<Policy> loadPolicies(const std::string& rsc) {
  std::ifstream in(rsc + "/policies.txt");
  RSFATAL_IF(!in, "cannot open " << rsc << "/policies.txt")
  std::vector<Policy> policies;
  std::string line, defaultName;
  while (std::getline(in, line)) {
    const auto tab = line.find('\t');
    if (tab == std::string::npos) continue;
    const std::string key = line.substr(0, tab), rest = line.substr(tab + 1);
    if (key == "default") { defaultName = rest; continue; }
    Policy p = loadPolicy(rsc + "/policies/" + key + "/policy.txt");
    p.name = key;
    p.label = rest;
    policies.push_back(std::move(p));
  }
  RSFATAL_IF(policies.empty(), "no policy listed in " << rsc << "/policies.txt")
  const auto it = std::find_if(policies.begin(), policies.end(), [&](const Policy& p) { return p.name == defaultName; });
  if (it != policies.end()) std::rotate(policies.begin(), it, it + 1);
  return policies;
}

/// The robot, its terrain and the controller the training environment runs: observation, policy, PD torques, resets.
class RaiwayDemo {
 public:
  /// lockWorld(true/false) brackets every change to the world's objects, for a server streaming it from another thread
  RaiwayDemo(const std::string& rsc, raisim::World& world, std::function<void(bool)> lockWorld = {})
      : world_(world), lockWorld_(std::move(lockWorld)), policies_(loadPolicies(rsc)) {
    /// cfg.yaml's restitution defaults, what training uses with its restitution randomization off
    world_.setDefaultMaterial(0.8, 0.5, 0.1);
    /// the robot comes from raiway.rscene, so a server can send its meshes to a browser viewer
    robot_ = dynamic_cast<raisim::ArticulatedSystem*>(world_.getObject("/World/Raiway"));
    RSFATAL_IF(!robot_, "the world holds no /World/Raiway; build it from " << rsc << "/raiway.rscene")
    robot_->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    gcDim_ = int(robot_->getGeneralizedCoordinateDim());
    gvDim_ = int(robot_->getDOF());
    gc_.setZero(gcDim_);
    gv_.setZero(gvDim_);
    genForce_.setZero(gvDim_);
    for (const char* name : {"Segway_L_WHEEL", "Segway_R_WHEEL", "Segway_L_SHANK", "Segway_R_SHANK"})
      contactBodies_.push_back(robot_->getBodyIdx(name));
    footFrames_ = {robot_->getFrameIdxByName("L_WHEEL"), robot_->getFrameIdxByName("R_WHEEL")};
    nominalJoint_ << 0.8, -1.6, 0.0, 0.8, -1.6, 0.0;
    actionStd_ << 0.4, 0.4, 4.0, 0.4, 0.4, 4.0;
    kp_ << 50.0, 50.0, 0.0, 50.0, 50.0, 0.0;
    kd_ << 0.5, 0.5, 4.0, 0.5, 0.5, 4.0;
    setPolicy(0);
    setTerrain(1, 1.0);
  }

  const std::vector<Policy>& policies() const { return policies_; }
  const Policy& policy() const { return policies_[policyIndex_]; }
  raisim::ArticulatedSystem* robot() const { return robot_; }
  int substeps() const { return int(std::lround(policy().conDt / policy().simDt)); }
  const std::vector<Eigen::Vector3d>& scanPoints() const { return scanPoints_; }
  const Eigen::VectorXd& estimate() const { return estimate_; }
  Eigen::Vector3d bodyLinVel() const { return baseRot_.transpose() * gv_.segment(0, 3); }
  Eigen::Vector3d bodyAngVel() const { return baseRot_.transpose() * gv_.segment(3, 3); }

  void setPolicy(int index) {
    policyIndex_ = std::clamp(index, 0, int(policies_.size()) - 1);
    const Policy& p = policy();
    world_.setTimeStep(p.simDt);
    scan_.assign(2, std::vector<double>(p.scanN, 0.0));
    scanPoints_.assign(2 * p.scanN, Eigen::Vector3d::Zero());
    history_.setZero(kEstDepth, p.obDim);
    if (heightMap_) reset();
  }

  void setTerrain(int type, double level) {
    terrainType_ = std::clamp(type, 0, int(kTerrainNames.size()) - 1);
    Terrain t = makeTerrain(terrainType_, level);
    lock();
    for (auto* obj : terrainObjects_) world_.removeObject(obj);
    terrainObjects_.clear();
    const bool plated = !t.boxes.empty();
    const double xSize = (kNx - 1) * kGrid, ySize = (kNy - 1) * kGrid;
    heightMap_ = plated ? world_.addHeightMap(kNx, kNy, xSize, ySize, kX0 + xSize / 2, kY0 + ySize / 2, t.h, "default",
                                              raisim::CollisionGroup(2), raisim::CollisionGroup(1))
                        : world_.addHeightMap(kNx, kNy, xSize, ySize, kX0 + xSize / 2, kY0 + ySize / 2, t.h);
    terrainObjects_.push_back(heightMap_);
    for (const auto& b : t.boxes) {
      auto* box = world_.addBox(b.thickness, ySize, b.height, 1000.0, "default", raisim::CollisionGroup(3), raisim::CollisionGroup(1));
      box->setPosition(b.x, kY0 + ySize / 2, b.z);
      box->setBodyType(raisim::BodyType::STATIC);
      box->setAppearance("white");
      terrainObjects_.push_back(box);
    }
    unlock();
    reset();
  }

  void reset() {
    lock();
    Eigen::VectorXd gc = Eigen::VectorXd::Zero(gcDim_), gv = Eigen::VectorXd::Zero(gvDim_);
    gc.segment(2, 5) << heightMap_->getHeight(0.0, 0.0) + kBaseHeight, 1.0, 0.0, 0.0, 0.0;
    gc.tail(kJoints) = nominalJoint_;
    robot_->setState(gc, gv);
    gc[2] += rimClearanceShift();
    robot_->setState(gc, gv);
    robot_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));
    unlock();
    posTarget_ = nominalJoint_;
    velTarget_.setZero();
    prevAction_ << gc[7], gc[8], 0.0, gc[10], gc[11], 0.0;
    prevprevAction_ = prevAction_;
    historyInit_ = false;
    estimate_.setZero(policy().estDim);
    fallTime_ = -1.0;
    updateState();
  }

  /// One control step: observe, run the policy and set the PD targets the next substeps track.
  void control(double vx, double wz) {
    const Policy& p = policy();
    command_ << vx, 0.0, wz;
    updateState();
    const Eigen::VectorXd ob = observe();
    Eigen::VectorXd input(p.obDim + 2 + p.estDim);
    input.head(p.obDim) = (ob - p.obMean).cwiseQuotient(p.obStd);
    input.segment(p.obDim, 2) << vx, wz;
    if (p.estDim) {
      Eigen::VectorXd estInput(p.estInputDim);
      const int tail = p.obDim - kEstTailStart, frame = kEstHead + tail;
      for (int k = 0; k < kEstLen; k++) {
        estInput.segment(k * frame, kEstHead) = history_.row(k * kEstStride).head(kEstHead);
        estInput.segment(k * frame + kEstHead, tail) = history_.row(k * kEstStride).tail(tail);
      }
      const Eigen::VectorXd est = p.estimator((estInput - p.estInMean).cwiseQuotient(p.estInStd));
      input.tail(p.estDim) = est;
      estimate_ = est.cwiseProduct(p.estOutStd) + p.estOutMean;
    }
    Eigen::VectorXd action = p.actor(input);
    if (p.bound.size()) action = p.bound.cwiseProduct((action.cwiseQuotient(p.bound)).array().tanh().matrix());
    const Eigen::VectorXd scaled = action.cwiseProduct(actionStd_) + nominalJoint_;
    posTarget_ << scaled[0], scaled[1], 0.0, scaled[3], scaled[4], 0.0;
    velTarget_ << 0.0, 0.0, scaled[2], 0.0, 0.0, scaled[5];
    prevprevAction_ = prevAction_;
    prevAction_ = scaled;
  }

  void applyPd() {
    robot_->getState(gc_, gv_);
    genForce_.tail(kJoints) = kp_.cwiseProduct(posTarget_ - gc_.tail(kJoints)) + kd_.cwiseProduct(velTarget_ - gv_.tail(kJoints));
    robot_->setGeneralizedForce(genForce_);
  }

  /// After a control step: resets a robot that fell (held a moment so the fall is visible) or left the map.
  bool checkEpisode() {
    updateState();
    const double t = world_.getWorldTime();
    if (fallTime_ < 0.0 && fallen()) {
      fallTime_ = t;
      std::cout << "fell at t " << t << " s, x " << gc_[0] << " m" << std::endl;
    }
    const bool outside = gc_[0] < kX0 + 0.5 || gc_[0] > kX1 - 0.5 || std::abs(gc_[1]) > kY1 - 0.5;
    if (outside || (fallTime_ >= 0.0 && t - fallTime_ >= kFallHoldTime)) {
      lastResetWasFall_ = fallTime_ >= 0.0;
      if (!lastResetWasFall_) std::cout << "left the map at t " << t << " s, x " << gc_[0] << " m, y " << gc_[1] << " m" << std::endl;
      reset();
      return true;
    }
    return false;
  }

  bool lastResetWasFall() const { return lastResetWasFall_; }

  bool fallen() const {
    for (const auto& contact : robot_->getContacts()) {
      const bool allowed = std::find(contactBodies_.begin(), contactBodies_.end(), contact.getlocalBodyIndex()) != contactBodies_.end();
      if (!allowed || contact.isSelfCollision()) return true;
    }
    return baseRot_(2, 2) < 0.5;
  }

 private:
  void lock() { if (lockWorld_) lockWorld_(true); }
  void unlock() { if (lockWorld_) lockWorld_(false); }

  /// Walks the lower rim of both tires, as training does, so the robot starts resting on the terrain.
  double rimClearanceShift() const {
    double shift = -1e9;
    for (int i = 0; i < 2; i++) {
      const size_t body = contactBodies_[i];
      const double offset = i == 0 ? kTireOffset : -kTireOffset;
      raisim::Mat<3, 3> rot;
      raisim::Vec<3> pos;
      robot_->getBodyPose(body, rot, pos);
      const double phiDown = std::atan2(-rot[8], -rot[2]);
      for (int side = -1; side <= 1; side += 2) {
        for (int k = 0; k < 72; k++) {
          const double phi = phiDown + M_PI * (double(k) / 71 - 0.5);
          raisim::Vec<3> rimInBody{kWheelRadius * std::cos(phi), offset + side * kWheelHalfWidth, kWheelRadius * std::sin(phi)}, rimInWorld;
          robot_->getPosition(body, rimInBody, rimInWorld);
          shift = std::max(shift, heightMap_->getContactHeight(rimInWorld[0], rimInWorld[1]) - rimInWorld[2]);
        }
      }
    }
    return shift;
  }

  void updateState() {
    robot_->getState(gc_, gv_);
    baseRot_ = robot_->getBaseOrientation().e();
    for (int i = 0; i < 2; i++) {
      raisim::Vec<3> pos;
      raisim::Mat<3, 3> rot;
      robot_->getFramePosition(footFrames_[i], pos);
      robot_->getFrameOrientation(footFrames_[i], rot);
      tireCenter_[i] = pos.e() + rot.e() * Eigen::Vector3d(0.0, i == 0 ? kTireOffset : -kTireOffset, 0.0);
    }
    controlX_ = Eigen::Vector3d(baseRot_(0, 0), baseRot_(1, 0), 0.0).normalized();
    controlY_ = Eigen::Vector3d::UnitZ().cross(controlX_);
  }

  Eigen::VectorXd observe() {
    const Policy& p = policy();
    Eigen::VectorXd ob(p.obDim);
    ob.segment(0, 3) = baseRot_.row(2);
    ob.segment(3, 3) = bodyAngVel();
    ob.segment(6, 4) << gc_[7], gc_[8], gc_[10], gc_[11];
    ob.segment(10, kJoints) = gv_.tail(kJoints);
    ob.segment(16, kJoints) = prevAction_;
    ob.segment(22, kJoints) = prevprevAction_;
    heightScan();
    for (int i = 0; i < 2; i++)
      for (int j = 0; j < p.scanN; j++) ob[kObHead + i * p.scanN + j] = scan_[i][j];
    if (!historyInit_) {
      history_.rowwise() = ob.transpose();
      historyInit_ = true;
    } else {
      for (int k = kEstDepth - 1; k > 0; k--) history_.row(k) = history_.row(k - 1);
      history_.row(0) = ob.transpose();
    }
    return ob;
  }

  /// Terrain height under each tire's scan points, as body-z offsets from the tire center, clamped to ±2 m.
  void heightScan() {
    const Policy& p = policy();
    for (int i = 0; i < 2; i++) {
      const Eigen::Vector3d& tire = tireCenter_[i];
      std::vector<std::pair<double, double>> offsets;
      if (p.scanType == 3) {
        for (size_t k = 0; k < kRings.size(); k++)
          for (int j = 0; j < kRings[k]; j++) {
            const double a = 2.0 * M_PI * j / kRings[k];
            offsets.emplace_back(kRingSpacingX * k * std::cos(a), kRingSpacingY * k * std::sin(a));
          }
      } else {
        const Eigen::Vector3d r = tire - gc_.head(3);
        const double rx = controlX_.dot(r), ry = controlY_.dot(r);
        const double vix = command_[0] - command_[2] * ry, viy = command_[1] + command_[2] * rx;
        const double speed = std::hypot(vix, viy), span = kScanFovMax - kScanLookMin;
        const double L = kScanLookMin + span * std::tanh(speed * kScanLookTime / span);
        Eigen::Vector2d t(1.0, 0.0), n(0.0, 1.0);
        double kappa = 0.0;
        if (speed > 1e-4) {
          t << vix / speed, viy / speed;
          n << -t[1], t[0];
          kappa = command_[2] / speed;
        }
        for (int j = 0; j < p.scanN; j++) {
          const double s = p.scanN > 1 ? -kScanLookBack + j * (kScanLookBack + L) / (p.scanN - 1) : 0.0;
          double u = s, w = 0.0;
          if (std::abs(kappa) > 1e-4) {
            u = std::sin(kappa * s) / kappa;
            w = (1.0 - std::cos(kappa * s)) / kappa;
          }
          offsets.emplace_back(u * t[0] + w * n[0], u * t[1] + w * n[1]);
        }
      }
      for (int j = 0; j < p.scanN; j++) {
        const auto [ox, oy] = offsets[j];
        Eigen::Vector3d point(tire[0] + ox * controlX_[0] + oy * controlY_[0], tire[1] + ox * controlX_[1] + oy * controlY_[1], 0.0);
        point[2] = heightMap_->getHeight(point[0], point[1]);
        scanPoints_[i * p.scanN + j] = point;
        scan_[i][j] = std::clamp((baseRot_.transpose() * (tire - point))[2], -2.0, 2.0);
      }
    }
  }

  raisim::World& world_;
  std::function<void(bool)> lockWorld_;
  std::vector<Policy> policies_;
  int policyIndex_ = 0, terrainType_ = 1;
  raisim::ArticulatedSystem* robot_ = nullptr;
  raisim::HeightMap* heightMap_ = nullptr;
  std::vector<raisim::Object*> terrainObjects_;
  int gcDim_ = 0, gvDim_ = 0;
  std::vector<size_t> contactBodies_;
  std::array<size_t, 2> footFrames_{};
  Eigen::VectorXd gc_, gv_, genForce_;
  Eigen::Matrix<double, kJoints, 1> nominalJoint_, actionStd_, kp_, kd_, posTarget_, velTarget_, prevAction_, prevprevAction_;
  Eigen::Matrix3d baseRot_ = Eigen::Matrix3d::Identity();
  std::array<Eigen::Vector3d, 2> tireCenter_{};
  Eigen::Vector3d controlX_ = Eigen::Vector3d::UnitX(), controlY_ = Eigen::Vector3d::UnitY(), command_ = Eigen::Vector3d::Zero();
  std::vector<std::vector<double>> scan_;
  std::vector<Eigen::Vector3d> scanPoints_;
  Eigen::MatrixXd history_;
  bool historyInit_ = false;
  Eigen::VectorXd estimate_;
  double fallTime_ = -1.0;
  bool lastResetWasFall_ = false;
};

}  // namespace raiway
