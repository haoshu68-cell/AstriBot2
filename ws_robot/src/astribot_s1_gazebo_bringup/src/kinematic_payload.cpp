// Simulation-only kinematic attachment. This is not a force/contact grasp.
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/msgs/Utility.hh>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <optional>
#include <sstream>
#include <iomanip>

namespace sim=ignition::gazebo;
namespace math=ignition::math;
namespace astribot {
class KinematicPayload : public sim::System, public sim::ISystemConfigure,
    public sim::ISystemPreUpdate, public sim::ISystemPostUpdate {
  sim::Entity model_{sim::kNullEntity},parent_{sim::kNullEntity};
  std::string parent_model_,parent_link_,parent_name_,error_;
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher state_;
  std::mutex mutex_;
  std::optional<ignition::msgs::Pose> pending_;
  uint32_t accepted_{0},applied_{0};
  bool attached_{false};
  math::Pose3d offset_;
  std::chrono::steady_clock::duration last_publish_{0};

  bool command(const ignition::msgs::Pose & req,ignition::msgs::Boolean & response) {
    const auto pose=ignition::msgs::Convert(req);
    const auto & q=req.orientation();
    const double norm=q.w()*q.w()+q.x()*q.x()+q.y()*q.y()+q.z()*q.z();
    std::lock_guard<std::mutex> lock(mutex_);
    // Commands are single-shot, ordered transactions. A transport response
    // means queued; the task must observe applied_ at a fresh simulation stamp.
    if(!pose.IsFinite() || !std::isfinite(norm) || std::abs(norm-1.)>.001 ||
      req.id()<=accepted_ || pending_ || (!req.name().empty() && req.name()!=parent_name_)) {
      response.set_data(false);return true;
    }
    accepted_=req.id();pending_=req;response.set_data(true);return true;
  }
public:
  void Configure(const sim::Entity & entity,const std::shared_ptr<const sdf::Element> & sdf,
      sim::EntityComponentManager & ecm,sim::EventManager &) override {
    model_=entity;
    parent_model_=sdf->Get<std::string>("parent_model");
    parent_link_=sdf->Get<std::string>("parent_link");
    parent_name_=parent_model_+"::"+parent_link_;
    const auto topic="/model/"+sim::Model(entity).Name(ecm)+"/kinematic_attachment";
    node_.Advertise(topic+"/command",&KinematicPayload::command,this);
    state_=node_.Advertise<ignition::msgs::StringMsg>(topic+"/state");
  }
  void PreUpdate(const sim::UpdateInfo & info,sim::EntityComponentManager & ecm) override {
    if(info.paused)return;
    if(parent_==sim::kNullEntity || !ecm.HasEntity(parent_)) {
      const auto robot=ecm.EntityByComponents(sim::components::Model(),sim::components::Name(parent_model_));
      parent_=robot==sim::kNullEntity ? sim::kNullEntity : sim::Model(robot).LinkByName(ecm,parent_link_);
    }
    std::optional<ignition::msgs::Pose> request;
    {std::lock_guard<std::mutex> lock(mutex_);request.swap(pending_);}
    if(request) {
      if(!request->name().empty() && parent_==sim::kNullEntity) {
        error_="PARENT_NOT_FOUND";
      } else {
        offset_=ignition::msgs::Convert(*request);
        attached_=!request->name().empty();applied_=request->id();error_.clear();
      }
    }
    if(attached_) {
      if(parent_==sim::kNullEntity) {error_="PARENT_LOST";return;}
      sim::Model(model_).SetWorldPoseCmd(ecm,sim::worldPose(parent_,ecm)*offset_);
    } else if(request && applied_==request->id()) {
      sim::Model(model_).SetWorldPoseCmd(ecm,offset_);
    }
  }
  void PostUpdate(const sim::UpdateInfo & info,const sim::EntityComponentManager & ecm) override {
    if(info.paused || (info.simTime>=last_publish_ && info.simTime-last_publish_<std::chrono::milliseconds(20)))return;
    last_publish_=info.simTime;
    double position_error=0.,rotation_error=0.;
    if(attached_ && parent_!=sim::kNullEntity && ecm.HasEntity(parent_)) {
      const auto delta=(sim::worldPose(parent_,ecm)*offset_).Inverse()*sim::worldPose(model_,ecm);
      position_error=delta.Pos().Length();
      rotation_error=2.*std::acos(std::clamp(std::abs(delta.Rot().W()),0.,1.));
    }
    std::ostringstream json;json<<std::setprecision(17)
      <<"{\"stamp_ns\":"<<std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count()
      <<",\"command_id\":"<<applied_<<",\"attached\":"<<(attached_?"true":"false")
      <<",\"position_error_m\":"<<position_error<<",\"rotation_error_rad\":"<<rotation_error
      <<",\"error\":\""<<error_<<"\"";
    // Independent ROS/TF probes need the actual and target poses from this
    // same PostUpdate, not a second publisher's differently sampled pose.
    auto pose_json=[&json](const char * key,const math::Pose3d & pose) {
      json<<",\""<<key<<"\":["<<pose.Pos().X()<<','<<pose.Pos().Y()<<','<<pose.Pos().Z()
        <<','<<pose.Rot().X()<<','<<pose.Rot().Y()<<','<<pose.Rot().Z()<<','<<pose.Rot().W()<<']';
    };
    pose_json("actual_world_xyzw",sim::worldPose(model_,ecm));
    if(attached_ && parent_!=sim::kNullEntity && ecm.HasEntity(parent_)) {
      pose_json("expected_world_xyzw",sim::worldPose(parent_,ecm)*offset_);
    }
    json<<",\"physics_dt_ns\":"<<std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).count()<<'}';
    ignition::msgs::StringMsg message;message.set_data(json.str());state_.Publish(message);
  }
};
}
IGNITION_ADD_PLUGIN(astribot::KinematicPayload,sim::System,
  sim::ISystemConfigure,sim::ISystemPreUpdate,sim::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(astribot::KinematicPayload,"astribot::KinematicPayload")
