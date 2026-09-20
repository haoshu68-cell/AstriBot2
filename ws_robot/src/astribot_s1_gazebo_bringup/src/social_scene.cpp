// Independent geometric observer of the simulated collision bodies, not a controller.
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/rendering/Events.hh>
#include <ignition/gazebo/components/Actor.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/common/Mesh.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/Util.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>
#include <ignition/rendering/GpuRays.hh>
#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Sphere.hh>
#include <sdf/Mesh.hh>
#include <filesystem>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <atomic>

namespace sim = ignition::gazebo;
namespace math = ignition::math;
namespace astribot {
class SocialScene : public sim::System, public sim::ISystemConfigure,
                    public sim::ISystemPreUpdate, public sim::ISystemPostUpdate {
  struct Person { std::string name; double radius, height; sim::Entity actor{0}, proxy{0};
                  math::Vector3d previous; bool seen{false}; };
  struct Bounds { math::Vector3d low, high; math::Pose3d pose; double radius; };
  std::vector<Person> people_;
  std::string robot_name_;
  sim::Entity robot_{0};
  ignition::transport::Node transport_;
  ignition::transport::Node::Publisher publisher_;
  std::unordered_map<sim::Entity, Bounds> previous_;
  std::unordered_map<std::string, std::pair<math::Vector3d, math::Vector3d>> meshes_;
  bool interval_valid_{true};
  double publish_at_{0}, last_time_{-1}, interval_min_{1e9};
  math::Vector3d swept_low_{1e9,1e9,1e9}, swept_high_{-1e9,-1e9,-1e9};
  uint64_t epoch_{0}, samples_{0}, overlap_steps_{0};
  ignition::common::ConnectionPtr render_connection_;
  std::vector<std::string> actor_names_;
  std::atomic<unsigned int> masked_actors_{0}, masked_lidars_{0};

  void ConfigureSensorVisibility() {
    // Animated skins are for RGB/depth and GUI. The co-located cylinder is the
    // lidar geometry: do not feed both the skin and its proxy to SLAM.
    constexpr uint32_t actor_flag = 1u << 27;
    constexpr uint32_t proxy_flag = 1u << 26;
    unsigned int actors=0, lidars=0;
    for(const auto &name:ignition::rendering::loadedEngines()) {
      auto engine=ignition::rendering::engine(name);
      if(!engine)continue;
      for(unsigned int s=0;s<engine->SceneCount();++s) {
        auto scene=engine->SceneByIndex(s);
        if(!scene)continue;
        for(unsigned int v=0;v<scene->VisualCount();++v) {
          auto visual=scene->VisualByIndex(v);
          if(!visual)continue;
          for(const auto &actor:actor_names_) {
            const auto &n=visual->Name();
            if(n==actor || n.rfind(actor+"::",0)==0) {
              if(visual->VisibilityFlags()!=actor_flag)visual->SetVisibilityFlags(actor_flag);
              ++actors;
            } else if(n==actor+"_collision" || n.rfind(actor+"_collision::",0)==0) {
              if(visual->VisibilityFlags()!=proxy_flag)visual->SetVisibilityFlags(proxy_flag);
            }
          }
        }
        for(unsigned int i=0;i<scene->SensorCount();++i) {
          auto sensor=scene->SensorByIndex(i);
          if(!sensor)continue;
          auto mask=sensor->VisibilityMask();
          if(std::dynamic_pointer_cast<ignition::rendering::GpuRays>(sensor)) {
            if(mask & actor_flag)sensor->SetVisibilityMask(mask & ~actor_flag);
            ++lidars;
          } else {
            if(mask & proxy_flag)sensor->SetVisibilityMask(mask & ~proxy_flag);
          }
        }
      }
    }
    masked_actors_.store(actors);masked_lidars_.store(lidars);
  }

  bool belongs(sim::Entity entity, const sim::EntityComponentManager &ecm) const {
    while (entity) {
      if (entity == robot_) return true;
      auto parent = ecm.Component<sim::components::ParentEntity>(entity);
      if (!parent) return false;
      entity = parent->Data();
    }
    return false;
  }
  bool bounds(sim::Entity entity, const sdf::Geometry &geometry,
              const sim::EntityComponentManager &ecm, Bounds &box) {
    auto pose = sim::worldPose(entity, ecm);
    math::Vector3d extent;
    if (geometry.Type() == sdf::GeometryType::SPHERE) {
      double r = geometry.SphereShape()->Radius(); extent.Set(r,r,r);
    } else if (geometry.Type() == sdf::GeometryType::CYLINDER) {
      auto cylinder = geometry.CylinderShape();
      auto axis = pose.Rot().RotateVector(math::Vector3d::UnitZ);
      for (int i=0; i<3; ++i) extent[i] = cylinder->Radius()*std::sqrt(std::max(0.,1-axis[i]*axis[i])) + std::abs(axis[i])*cylinder->Length()/2;
    } else if (geometry.Type() == sdf::GeometryType::BOX) {
      auto half = geometry.BoxShape()->Size()/2;
      for (int axis=0; axis<3; ++axis) {
        math::Vector3d v; v[axis]=half[axis]; v=pose.Rot().RotateVector(v);
        for (int i=0; i<3; ++i) extent[i]+=std::abs(v[i]);
      }
    } else if (geometry.Type() == sdf::GeometryType::MESH) {
      auto shape=geometry.MeshShape();
      if (!shape->Submesh().empty()) return false;
      auto uri=sim::asFullPath(shape->Uri(),shape->FilePath());
      auto found=meshes_.find(uri);
      if(found==meshes_.end()) {
        std::string file=ignition::common::findFile(uri);
        if(file.empty()) {
          auto relative=shape->Uri();
          for(const auto &scheme:{std::string("model://"),std::string("package://"),std::string("file://")})
            if(relative.rfind(scheme,0)==0)relative=relative.substr(scheme.size());
          for(const auto &prefix:sim::resourcePaths()) {
            auto candidate=std::filesystem::path(prefix)/relative;
            if(std::filesystem::is_regular_file(candidate)){file=candidate.string();break;}
          }
        }
        if(file.empty()) return false;
        auto mesh=ignition::common::MeshManager::Instance()->Load(file);
        if(!mesh)return false;
        found=meshes_.emplace(uri,std::make_pair(mesh->Min(),mesh->Max())).first;
      }
      const auto center=(found->second.first+found->second.second)*.5*shape->Scale();
      const auto half=(found->second.second-found->second.first)*.5*shape->Scale();
      pose.Pos()+=pose.Rot().RotateVector(center);
      for(int axis=0;axis<3;++axis) {
        math::Vector3d v;v[axis]=half[axis];v=pose.Rot().RotateVector(v);
        for(int i=0;i<3;++i)extent[i]+=std::abs(v[i]);
      }
    } else return false;
    box={pose.Pos()-extent, pose.Pos()+extent, pose, extent.Length()}; return true;
  }
public:
  void Configure(const sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
                 sim::EntityComponentManager &, sim::EventManager &events) override {
    auto config=sdf->Clone(); robot_name_=config->Get<std::string>("robot_name");
    if (config->HasElement("person")) {
      for (auto p=config->GetElement("person"); p; p=p->GetNextElement("person"))
        people_.push_back({p->Get<std::string>("name"),p->Get<double>("radius"),p->Get<double>("height")});
    }
    for(const auto &p:people_)actor_names_.push_back(p.name);
    if(config->Get<bool>("actor_lidar_proxy_only",true).first)
      render_connection_=events.Connect<sim::events::PreRender>([this](){ConfigureSensorVisibility();});
    publisher_=transport_.Advertise<ignition::msgs::StringMsg>("/social_sim/state");
  }
  void PreUpdate(const sim::UpdateInfo &info, sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    for (auto &p:people_) {
      if (!p.actor) p.actor=ecm.EntityByComponents(sim::components::Name(p.name));
      if (!p.proxy) p.proxy=ecm.EntityByComponents(sim::components::Name(p.name+"_collision"));
      auto trajectory=ecm.Component<sim::components::TrajectoryPose>(p.actor);
      if (!p.proxy || !trajectory) continue;
      const auto &pose=trajectory->Data();
      sim::Model(p.proxy).SetWorldPoseCmd(ecm,math::Pose3d(pose.Pos().X(),pose.Pos().Y(),0,0,0,0));
    }
  }
  void PostUpdate(const sim::UpdateInfo &info, const sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    double now=std::chrono::duration<double>(info.simTime).count();
    if (now<last_time_) { ++epoch_; previous_.clear(); samples_=overlap_steps_=0; publish_at_=0; interval_min_=1e9; interval_valid_=true; swept_low_.Set(1e9,1e9,1e9); swept_high_.Set(-1e9,-1e9,-1e9); for(auto &p:people_)p.seen=false; }
    last_time_=now;
    if (!robot_) robot_=ecm.EntityByComponents(sim::components::Name(robot_name_));
    if (!robot_) return;
    bool valid=true; size_t count=0,unsupported=0; double minimum=1e9;
    std::vector<math::Vector3d> positions; double human_step=0;
    for (auto &p:people_) {
      if (!p.proxy) { valid=false; positions.push_back({}); continue; }
      auto pos=sim::worldPose(p.proxy,ecm).Pos(); positions.push_back(pos);
      if(p.seen) human_step=std::max(human_step,(pos-p.previous).Length());
      p.previous=pos; p.seen=true;
    }
    ecm.Each<sim::components::Collision,sim::components::Geometry>(
      [&](const sim::Entity &entity,const sim::components::Collision *,const sim::components::Geometry *geometry) {
        if(!belongs(entity,ecm))return true;
        ++count; Bounds b;
        if(!bounds(entity,geometry->Data(),ecm,b)) {valid=false;++unsupported;return true;}
        double robot_step=0;
        auto old=previous_.find(entity);
        if(old!=previous_.end()) {
          auto delta=old->second.pose.Rot().Inverse()*b.pose.Rot();
          double angle=2*std::acos(std::clamp(std::abs(delta.W()),0.,1.));
          robot_step=(b.pose.Pos()-old->second.pose.Pos()).Length()+angle*std::max(b.radius,old->second.radius);
        }
        previous_[entity]=b;
        // World-space union over every physics tick, including a conservative
        // bound between ticks. Validation can compare this to physical walls
        // without consuming the navigation policy's own collision decision.
        for(int axis=0;axis<3;++axis) {
          swept_low_[axis]=std::min(swept_low_[axis],b.low[axis]-robot_step);
          swept_high_[axis]=std::max(swept_high_[axis],b.high[axis]+robot_step);
        }
        for(size_t i=0;i<people_.size();++i) {
          auto &p=people_[i];const auto &pos=positions[i];
          if(b.low.Z()>p.height || b.high.Z()<0)continue;
          double dx=std::max({b.low.X()-pos.X(),0.,pos.X()-b.high.X()});
          double dy=std::max({b.low.Y()-pos.Y(),0.,pos.Y()-b.high.Y()});
          minimum=std::min(minimum,std::hypot(dx,dy)-p.radius-robot_step-human_step);
        }
        return true;
      });
    valid=valid && count>0; ++samples_;
    interval_valid_=interval_valid_ && valid;
    if(valid && minimum<=0)++overlap_steps_;
    interval_min_=std::min(interval_min_,minimum);
    if(now<publish_at_)return;
    publish_at_=now+.05;
    const auto pose=sim::worldPose(robot_,ecm);
    std::ostringstream out; out.precision(12);
    out<<"{\"stamp_ns\":"<<std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count()
       <<",\"epoch\":"<<epoch_<<",\"geometry_valid\":"<<(interval_valid_?"true":"false")
       <<",\"unsupported_geometry_count\":"<<unsupported
       <<",\"robot_collision_count\":"<<count<<",\"physics_samples\":"<<samples_
       <<",\"lidar_masked_actor_visuals\":"<<masked_actors_.load()<<",\"lidar_masked_sensors\":"<<masked_lidars_.load()
       <<",\"conservative_overlap_steps\":"<<overlap_steps_<<",\"clearance_lower_bound_m\":"<<interval_min_
       <<",\"robot_swept_bounds\":[["<<swept_low_.X()<<","<<swept_low_.Y()<<","<<swept_low_.Z()
       <<"],["<<swept_high_.X()<<","<<swept_high_.Y()<<","<<swept_high_.Z()<<"]]"
       <<",\"robot\":["<<pose.Pos().X()<<","<<pose.Pos().Y()<<","<<pose.Rot().Yaw()<<"],\"people\":[";
    for(size_t i=0;i<people_.size();++i) {
      if(i)out<<",";
      auto trajectory=ecm.Component<sim::components::TrajectoryPose>(people_[i].actor);
      auto base=sim::worldPose(people_[i].actor,ecm);
      auto visual=trajectory?base*trajectory->Data():base;
      out<<"{\"name\":\""<<people_[i].name<<"\",\"x\":"<<positions[i].X()<<",\"y\":"<<positions[i].Y()<<",\"radius\":"<<people_[i].radius
         <<",\"visual_root\":["<<visual.Pos().X()<<","<<visual.Pos().Y()<<","<<visual.Pos().Z()<<","<<visual.Rot().Yaw()<<"]}";
    }
    out<<"]}"; ignition::msgs::StringMsg message;message.set_data(out.str());publisher_.Publish(message);interval_min_=1e9;interval_valid_=true;
    swept_low_.Set(1e9,1e9,1e9);swept_high_.Set(-1e9,-1e9,-1e9);
  }
};
}
IGNITION_ADD_PLUGIN(astribot::SocialScene, sim::System, sim::ISystemConfigure, sim::ISystemPreUpdate, sim::ISystemPostUpdate)
