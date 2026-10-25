// URRC practice traffic: deterministic continuous pose interpolation at each Gazebo simulation step.
// Kinematic obstacles (not force-based Ackermann dynamics); collisions/visuals are in the SDF model.
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Element.hh>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

namespace urrc {
struct Point { double x, y, z, s; };
class PracticeMotion final : public gz::sim::System,
                             public gz::sim::ISystemConfigure,
                             public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    entity_ = entity;
    if (sdf->HasElement("start_gate_file")) gateFile_ = sdf->Get<std::string>("start_gate_file");
    if (!sdf->HasElement("centerline_csv") || !sdf->HasElement("speed_mps") ||
        !sdf->HasElement("start_m") || !sdf->HasElement("lateral_m") ||
        !sdf->HasElement("spawn_x") || !sdf->HasElement("spawn_y") ||
        !sdf->HasElement("spawn_road_z") || !sdf->HasElement("spawn_yaw")) return;
    const auto path = sdf->Get<std::string>("centerline_csv");
    speed_ = sdf->Get<double>("speed_mps");
    initial_ = sdf->Get<double>("start_m");
    lateral_ = sdf->Get<double>("lateral_m");
    spawnX_ = sdf->Get<double>("spawn_x");
    spawnY_ = sdf->Get<double>("spawn_y");
    spawnZ_ = sdf->Get<double>("spawn_road_z");
    spawnYaw_ = sdf->Get<double>("spawn_yaw");
    if (!std::isfinite(speed_) || speed_ <= 0 || !std::isfinite(initial_) || !std::isfinite(lateral_) ||
        !std::isfinite(spawnX_) || !std::isfinite(spawnY_) ||
        !std::isfinite(spawnZ_) || !std::isfinite(spawnYaw_)) return;
    std::ifstream in(path);
    if (!in) return;
    std::string line;
    std::getline(in, line);  // CSV header
    double running = 0;
    while (std::getline(in, line)) {
      std::istringstream row(line);
      std::string item;
      std::vector<std::string> cells;
      while (std::getline(row, item, ',')) cells.push_back(item);
      if (cells.size() < 4) continue;
      try {
        const double x = std::stod(cells.at(1)), y = std::stod(cells.at(2)), z = std::stod(cells.at(3));
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
        if (!points_.empty()) {
          const auto &p = points_.back();
          const double d = std::hypot(x - p.x, y - p.y);
          if (d < 1e-6) continue;
          running += d;
        }
        points_.push_back({x, y, z, running});
      } catch (...) { continue; }
    }
    if (points_.size() < 5) return;
    length_ = points_.back().s + std::hypot(points_.front().x - points_.back().x,
                                           points_.front().y - points_.back().y);
    ready_ = length_ > 0;
    if (ready_) {
      const auto initial = At(initial_);
      const auto ahead = At(initial_ + 0.22);
      const auto behind = At(initial_ - 0.22);
      const double heading = std::atan2(ahead.y - behind.y, ahead.x - behind.x);
      // Blend from the exact painted-grid center to the target lane over 6 m.
      offsetX_ = spawnX_ - (initial.x - std::sin(heading) * lateral_);
      offsetY_ = spawnY_ - (initial.y + std::cos(heading) * lateral_);
      offsetZ_ = spawnZ_ - initial.z;
      yawDelta_ = std::atan2(std::sin(spawnYaw_ - heading), std::cos(spawnYaw_ - heading));
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (!ready_ || info.paused) return;
    // Hold each car at painted grid center until lights are fully out.
    if (!gateFile_.empty() && !std::filesystem::exists(gateFile_)) return;
    // Preserve the exact painted-box centre for the first 5 simulation seconds.
    // Do not use absolute world time as a travel offset on startup.
    const double simulationSeconds = std::chrono::duration<double>(info.simTime).count();
    if (!started_) { initialSimSeconds_ = simulationSeconds; started_ = true; }
    const double elapsed = std::max(0.0, simulationSeconds - initialSimSeconds_);
    const double s = initial_ + speed_ * elapsed;
    const auto here = At(s);
    const auto ahead = At(s + 0.22);
    const auto behind = At(s - 0.22);
    const double heading = std::atan2(ahead.y - behind.y, ahead.x - behind.x);
    const double planar = std::hypot(ahead.x - behind.x, ahead.y - behind.y);
    const double pitch = -std::atan2(ahead.z - behind.z, planar);
    const double progress = std::clamp(speed_ * elapsed / 6., 0., 1.);
    const double blend = progress * progress * (3. - 2. * progress);
    const double remaining = 1. - blend;
    const double x = here.x - std::sin(heading) * lateral_ + remaining * offsetX_;
    const double y = here.y + std::cos(heading) * lateral_ + remaining * offsetY_;
    const double correctedYaw = heading + remaining * yawDelta_;
    // Reference model origin is 0.09 m above road. Wheel bottoms are at local -0.09 m.
    gz::sim::Model(entity_).SetWorldPoseCmd(ecm, gz::math::Pose3d(x, y, here.z + remaining * offsetZ_ + 0.09, 0., pitch * blend, correctedYaw));
  }

 private:
  Point At(double distance) const {
    const double s = std::fmod(std::fmod(distance, length_) + length_, length_);
    auto it = std::upper_bound(points_.begin(), points_.end(), s,
                               [](double v, const Point &p) { return v < p.s; });
    const size_t index = it == points_.begin() ? 0 : static_cast<size_t>(it - points_.begin() - 1);
    const auto &a = points_.at(index);
    const auto &b = points_.at((index + 1) % points_.size());
    const double end = index + 1 == points_.size() ? length_ : b.s;
    const double t = std::clamp((s - a.s) / (end - a.s), 0., 1.);
    return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, s};
  }
  gz::sim::Entity entity_{gz::sim::kNullEntity};
  std::vector<Point> points_;
  std::string gateFile_;
  double speed_{0}, initial_{0}, lateral_{0}, length_{0};
  double spawnX_{0}, spawnY_{0}, spawnZ_{0}, spawnYaw_{0};
  double offsetX_{0}, offsetY_{0}, offsetZ_{0}, yawDelta_{0};
  bool ready_{false};
  bool started_{false};
  double initialSimSeconds_{0.0};
};
}
GZ_ADD_PLUGIN(urrc::PracticeMotion, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
