// Exhibition pace cars: kinematic offset-lane tracking, not participant AI.
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Element.hh>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

namespace urrc {
struct Point {double s,x,y,z,yaw;};
struct Car {
  std::string name;
  std::vector<Point> points;
  gz::sim::Entity entity{gz::sim::kNullEntity};
  double speed{0}, zOffset{0.003};
};
class PaceCars : public gz::sim::System,
                 public gz::sim::ISystemConfigure,
                 public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &world,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override {
    this->worldEntity=world;
    const std::string topic = sdf->Get<std::string>("start_topic", "/urrc/pace/start").first;
    if (!this->node.Subscribe(topic, &PaceCars::OnStart, this))
      std::cerr << "[PACE] Cannot subscribe to " << topic << std::endl;
    this->readyPublisher = this->node.Advertise<gz::msgs::Boolean>(topic + "/ready");
    if (sdf->HasElement("car")) {
      auto element=sdf->FindElement("car");
      while (element) {
        Car car;
        car.name=element->Get<std::string>("name");
        car.speed=element->Get<double>("speed_mps");
        car.zOffset=element->Get<double>("z_offset_m");
        const auto path=element->Get<std::string>("route_file");
        std::ifstream file(path);
        if (!file.is_open()) {
          std::cerr << "[PACE] Missing route " << path << std::endl;
        } else {
          std::string line;
          std::getline(file,line); // CSV header
          while(std::getline(file,line)){
            std::replace(line.begin(),line.end(),',',' ');
            std::istringstream in(line);
            Point p{};
            if(in>>p.s>>p.x>>p.y>>p.z>>p.yaw)car.points.push_back(p);
          }
        }
        if(car.points.size()<2 || car.speed<=0 || car.points.back().s<=0)
          std::cerr << "[PACE] Invalid car/route " << car.name << std::endl;
        else this->cars.push_back(std::move(car));
        element=element->GetNextElement("car");
      }
    }
    std::cout << "[PACE] Armed " << cars.size() << " cars, waiting for all five lights OFF" << std::endl;
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused)return;
    if (this->startRequested.load() && !this->started){
      this->started=true;
      this->startTime=std::chrono::duration<double>(info.simTime).count();
      std::cout << "[PACE] ALL 5 LIGHTS OFF: all pace cars started" << std::endl;
    }
    if (!this->started)return;
    const double now=std::chrono::duration<double>(info.simTime).count();
    const double elapsed=std::max(0.0,now-this->startTime);
    for(auto &car : cars) {
      if(car.entity==gz::sim::kNullEntity){
        ecm.Each<gz::sim::components::Model,
                 gz::sim::components::Name,
                 gz::sim::components::ParentEntity>(
          [&](const gz::sim::Entity &ent,
              const gz::sim::components::Model *,
              const gz::sim::components::Name *name,
              const gz::sim::components::ParentEntity *parent)->bool{
            if(name->Data()==car.name && parent->Data()==this->worldEntity){
              car.entity=ent;return false;
            }
            return true;
          });
      }
      if(car.entity==gz::sim::kNullEntity)continue;
      const auto &path=car.points;
      const double lap=path.back().s;
      const double dist=std::fmod(elapsed*car.speed,lap);
      auto it=std::upper_bound(path.begin(),path.end(),dist,
          [](double s,const Point &p){return s<p.s;});
      if(it==path.begin())++it;
      if(it==path.end())it=path.end()-1;
      const auto &p1=*(it-1),&p2=*it;
      const double t=std::clamp((dist-p1.s)/std::max(1e-9,p2.s-p1.s),0.0,1.0);
      const double dyaw=std::atan2(std::sin(p2.yaw-p1.yaw),std::cos(p2.yaw-p1.yaw));
      gz::math::Pose3d pose(p1.x+(p2.x-p1.x)*t,p1.y+(p2.y-p1.y)*t,
                           p1.z+(p2.z-p1.z)*t+car.zOffset,
                           0,0,p1.yaw+dyaw*t);
      gz::sim::Model(car.entity).SetWorldPoseCmd(ecm,pose);
    }
  }
 private:
  void OnStart(const gz::msgs::Boolean &msg){
    if(msg.data())this->startRequested.store(true);
  }
  gz::sim::Entity worldEntity{gz::sim::kNullEntity};
  gz::transport::Node node;
  gz::transport::Node::Publisher readyPublisher;
  std::vector<Car> cars;
  std::atomic<bool> startRequested{false};
  bool started{false};
  double startTime{0};
};
} // namespace urrc
GZ_ADD_PLUGIN(urrc::PaceCars, gz::sim::System,
              urrc::PaceCars::ISystemConfigure,
              urrc::PaceCars::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(urrc::PaceCars, "urrc::PaceCars")
