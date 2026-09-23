// Read-only EMPTY evidence for the explicitly restricted navigation warehouse.
// No attach/detach, PlanningScene writes, action requests or motion commands.
#include "astribot_s1_gazebo_bringup/inventory_gate.hpp"
#include "astribot_s1_gazebo_bringup/payload_geometry.hpp"
#include "astribot_s1_payload_state/ledger.hpp"
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/ParentLinkName.hh>
#include <ignition/gazebo/components/ChildLinkName.hh>
#include <ignition/gazebo/Util.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Model.hh>
#include <sdf/Link.hh>
#include <sdf/Joint.hh>
#include <filesystem>
#include <sdf/SemanticPose.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <ignition/gazebo/components/SystemPluginInfo.hh>
#include <ignition/plugin/Register.hh>
#include <astribot_payload_msgs/msg/attachment_observation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <random>
#include <optional>
#include <cmath>
#include <sstream>
#include <fstream>
#include <regex>

namespace sim=ignition::gazebo;
namespace astribot {
class EmptyInventory:public sim::System,public sim::ISystemConfigure,public sim::ISystemPostUpdate {
  using Observation=astribot_payload_msgs::msg::AttachmentObservation;
  sim::Entity world_{sim::kNullEntity};
  std::string robot_,world_name_,session_,source_,epoch_;
  std::string policy_="static_world_empty_only_v1";
  struct Registration {std::string object_id,parent,tcp;ignition::math::Pose3d parent_from_tcp;};
  std::map<std::string,Registration> payload_registry_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<Observation>::SharedPtr observations_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_;
  simulation::InventoryVersion version_;
  int64_t last_=-1;
  struct Pending {
    Observation observation;std_msgs::msg::String diagnostic;simulation::Version version;
    int64_t capture,deadline,wall;
  };
  std::optional<Pending> pending_;
  void publish(const Pending &sample) {
    observations_->publish(sample.observation);diagnostics_->publish(sample.diagnostic);
  }
  struct Background {ignition::math::Pose3d pose;bool is_static;uint64_t links;};
  std::map<std::string,Background> background_;
  std::set<std::string> robot_links_;
  simulation::JointConnections robot_joints_;
  std::map<std::string,sim::Entity> bound_entities_;
  void background(const sdf::Model &m,const std::string &prefix,const ignition::math::Pose3d &parent,bool parent_static) {
    ignition::math::Pose3d local;
    if(!m.SemanticPose().Resolve(local).empty())throw std::invalid_argument("BASELINE_POSE_UNRESOLVED");
    const auto name=prefix.empty()?m.Name():prefix+"::"+m.Name();
    const auto pose=parent*local;
    const bool fixed=parent_static || m.Static();
    if((!fixed && m.LinkCount()) || m.JointCount())throw std::invalid_argument("BASELINE_NOT_STATIC");
    if(!background_.emplace(name,Background{pose,fixed,m.LinkCount()}).second || background_.size()>4096)
      throw std::invalid_argument("INVALID_BACKGROUND_MANIFEST");
    for(uint64_t i=0;i<m.ModelCount();++i)background(*m.ModelByIndex(i),name,pose,fixed);
  }
  std::string model_path(sim::Entity id,const sim::EntityComponentManager &ecm) const {
    std::vector<std::string> parts;
    for(unsigned depth=0;id!=world_ && depth<64;++depth) {
      const auto *name=ecm.Component<sim::components::Name>(id);
      const auto *parent=ecm.Component<sim::components::ParentEntity>(id);
      if(!name || !parent)return {};
      parts.push_back(name->Data());id=parent->Data();
    }
    if(id!=world_)return {};
    std::string value;for(auto i=parts.rbegin();i!=parts.rend();++i)value+=(value.empty()?"":"::")+*i;
    return value;
  }
  static builtin_interfaces::msg::Time stamp(int64_t ns) {
    if(ns<0 || ns/1000000000>INT32_MAX)throw std::invalid_argument("INVALID_SIM_TIME");
    builtin_interfaces::msg::Time out;out.sec=static_cast<int32_t>(ns/1000000000);
    out.nanosec=static_cast<uint32_t>(ns%1000000000);return out;
  }
  bool member(sim::Entity entity,sim::Entity root,const sim::EntityComponentManager &ecm) const {
    for(unsigned depth=0;depth<64 && entity!=sim::kNullEntity;++depth) {
      if(entity==root)return true;
      const auto *parent=ecm.Component<sim::components::ParentEntity>(entity);
      if(!parent)return false;
      entity=parent->Data();
    }
    return false;
  }
public:
  void Configure(const sim::Entity &entity,const std::shared_ptr<const sdf::Element> &config,
                 sim::EntityComponentManager &ecm,sim::EventManager &) override {
    try {configure(entity,config,ecm);}
    catch(const std::exception &error) {
      observations_.reset();diagnostics_.reset();node_.reset();
      std::cerr<<"[EmptyInventory] CONFIGURATION_REJECTED: "<<error.what()<<std::endl;
    }
  }
  void configure(const sim::Entity &entity,const std::shared_ptr<const sdf::Element> &config,
                 sim::EntityComponentManager &ecm) {
    world_=entity;
    if(!ecm.Component<sim::components::World>(world_))throw std::invalid_argument("INVENTORY_REQUIRES_WORLD_ENTITY");
    const auto required=[&](const char *key) {
      if(!config->HasElement(key))throw std::invalid_argument(std::string("MISSING_")+key);
      const auto s=config->Get<std::string>(key);
      if(s.empty() || s.size()>128)throw std::invalid_argument(std::string("INVALID_")+key);
      return s;
    };
    robot_=required("robot_model");world_name_=required("world_name");
    session_=required("session_id");source_=required("source_id");
    policy_=required("inventory_policy");
    if(policy_!="static_world_empty_only_v1" && policy_!="kinematic_inventory_v1")throw std::invalid_argument("UNSUPPORTED_INVENTORY_POLICY");
    const auto file=[&](const char *key) {
      if(!config->HasElement(key))throw std::invalid_argument(std::string("MISSING_")+key);
      auto path=config->Get<std::string>(key);
      if(path.empty() || path.size()>4096 || !std::filesystem::is_regular_file(path))
        throw std::invalid_argument(std::string("INVALID_FILE_")+key);
      return path;
    };
    sdf::Root robot_reference;
    if(!robot_reference.Load(file("baseline_robot_sdf")).empty() || !robot_reference.Model() || robot_reference.Model()->ModelCount())
      throw std::invalid_argument("ROBOT_BASELINE_LOAD_FAILED");
    const auto *robot_model=robot_reference.Model();
    for(uint64_t i=0;i<robot_model->LinkCount();++i)robot_links_.insert(robot_model->LinkByIndex(i)->Name());
    for(uint64_t i=0;i<robot_model->JointCount();++i) {
      const auto *joint=robot_model->JointByIndex(i);
      robot_joints_[joint->Name()]={joint->ParentLinkName(),joint->ChildLinkName()};
    }
    if(robot_links_.empty())throw std::invalid_argument("EMPTY_ROBOT_BASELINE");
    if(policy_=="kinematic_inventory_v1") {
      std::ifstream registration_file(file("payload_registry"));nlohmann::json records;registration_file>>records;
      std::ifstream urdf_file(file("baseline_robot_urdf"));std::ostringstream robot_xml;robot_xml<<urdf_file.rdbuf();
      if(!records.is_array() || records.empty() || records.size()>32)throw std::invalid_argument("INVALID_PAYLOAD_REGISTRY");
      std::set<std::string> object_ids;
      for(const auto &record:records) {
        const auto id=[&](const char *key) {auto value=record.at(key).get<std::string>();
          if(!std::regex_match(value,std::regex("[A-Za-z0-9_.-]{1,128}")))throw std::invalid_argument("INVALID_PAYLOAD_IDENTITY");
          return value;};
        const auto model=id("model");Registration entry{id("object_id"),id("physical_parent_link"),id("attachment_link"),{}};
        if(model==robot_ || !robot_links_.count(entry.parent) || !object_ids.insert(entry.object_id).second ||
           (entry.tcp!="astribot_arm_left_tcp_link" && entry.tcp!="astribot_arm_right_tcp_link"))throw std::invalid_argument("INVALID_PAYLOAD_REGISTRY");
        entry.parent_from_tcp=simulation::fixed_parent_from_tcp(robot_xml.str(),entry.parent,entry.tcp);
        if(!payload_registry_.emplace(model,entry).second)throw std::invalid_argument("DUPLICATE_PAYLOAD_MODEL");
      }
    }
    sdf::Root baseline;
    if(!baseline.Load(file("baseline_world_sdf")).empty() || baseline.WorldCount()!=1)
      throw std::invalid_argument("BASELINE_WORLD_LOAD_FAILED");
    const auto *reference=baseline.WorldByIndex(0);
    if(reference->Name()!=world_name_)throw std::invalid_argument("BASELINE_WORLD_MISMATCH");
    for(uint64_t i=0;i<reference->ModelCount();++i)background(*reference->ModelByIndex(i),"",{},false);
    if(background_.empty())throw std::invalid_argument("EMPTY_BACKGROUND_MANIFEST");
    std::random_device entropy;std::ostringstream key;
    for(unsigned i=0;i<8;++i)key<<std::hex<<entropy();
    epoch_=key.str();
    context_=std::make_shared<rclcpp::Context>();context_->init(0,nullptr);
    node_=std::make_shared<rclcpp::Node>("simulation_empty_inventory",rclcpp::NodeOptions().context(context_).use_global_arguments(false));
    observations_=node_->create_publisher<Observation>("/payload/attachment_observation",rclcpp::QoS(4).reliable());
    diagnostics_=node_->create_publisher<std_msgs::msg::String>("/payload/simulation_inventory_diagnostics",rclcpp::QoS(4).reliable());
    RCLCPP_INFO(node_->get_logger(),"World inventory: session=%s source=%s policy=%s; unsupported mechanisms reject",session_.c_str(),source_.c_str(),policy_.c_str());
  }
  void PostUpdate(const sim::UpdateInfo &info,const sim::EntityComponentManager &ecm) override {
    if(info.paused || !node_)return;
    const int64_t capture=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    if(last_>=0 && capture>=last_ && capture-last_<50000000)return;
    last_=capture;
    simulation::InventorySnapshot snapshot;
    const auto *world_name=ecm.Component<sim::components::Name>(world_);
    snapshot.world_present=ecm.HasEntity(world_) && world_name && world_name->Data()==world_name_;
    sim::Entity robot=sim::kNullEntity;
    ecm.Each<sim::components::Model,sim::components::Name>([&](const sim::Entity &id,const auto *,const auto *name) {
      if(name->Data()==robot_ && member(id,world_,ecm)) {robot=id;++snapshot.robot_count;}return true;
    });
    nlohmann::json universe=nlohmann::json::array(),plugins=nlohmann::json::array();
    std::map<sim::Entity,simulation::PayloadExecution> payloads;
    std::map<sim::Entity,std::string> payload_names;
    std::string payload_error;
    snapshot.background_matches=true;std::set<std::string> seen;
    std::map<sim::Entity,unsigned> direct_links;
    std::set<std::string> live_links;simulation::JointConnections live_joints;
    bool robot_entities_valid=true;
    auto bind_robot=[&](const std::string &name,sim::Entity id) {
      const auto key="robot::"+name;
      if(bound_entities_.count(key) && bound_entities_.at(key)!=id)robot_entities_valid=false;
      else bound_entities_.emplace(key,id);
    };
    ecm.Each<sim::components::Link,sim::components::ParentEntity>([&](const sim::Entity &id,const auto *,const auto *parent) {
      ++direct_links[parent->Data()];
      if(parent->Data()==robot) {
        const auto *name=ecm.Component<sim::components::Name>(id);
        if(!name || !live_links.insert(name->Data()).second)robot_entities_valid=false;
        else bind_robot("link::"+name->Data(),id);
      }
      return true;
    });
    ecm.Each<sim::components::Model>([&](const sim::Entity &id,const auto *) {
      const auto *is_static=ecm.Component<sim::components::Static>(id);
      const bool is_robot=robot!=sim::kNullEntity && id==robot;
      if(!is_static || !member(id,world_,ecm))snapshot.model_metadata_missing=true;
      const auto path=model_path(id,ecm);
      bool fixed=is_static && is_static->Data();
      const auto registered=payload_registry_.find(path);
      if(!is_robot && registered!=payload_registry_.end()) {
        const auto execution=simulation::read_execution(id,ecm);
        if(!execution || direct_links[id]!=1)payload_error="PAYLOAD_EXECUTION_OR_STRUCTURE_MISSING";
        else {payloads.emplace(id,*execution);payload_names.emplace(id,path);}
        const auto key="payload::"+path;
        if(bound_entities_.count(key) && bound_entities_.at(key)!=id)payload_error="PAYLOAD_ENTITY_REPLACED";
        else bound_entities_.emplace(key,id);
      }else if(!is_robot) {
        const auto expected=background_.find(path);
        if(expected==background_.end() || !seen.insert(path).second) snapshot.background_matches=false;
        else {
          const auto delta=expected->second.pose.Inverse()*sim::worldPose(id,ecm);
          // Zero-link SDF wrappers can have static=false while containing only static models.
          fixed=fixed || (expected->second.links==0 && direct_links[id]==0);
          if(direct_links[id]!=expected->second.links || (expected->second.is_static && !(is_static && is_static->Data())) ||
             !delta.IsFinite() || delta.Pos().Length()>1e-5 ||
             2.*std::acos(std::clamp(std::abs(delta.Rot().W()),0.,1.))>1e-5)
            snapshot.background_matches=false;
          if(bound_entities_.count(path) && bound_entities_.at(path)!=id)snapshot.background_matches=false;
          else bound_entities_.emplace(path,id);
        }
      }
      snapshot.models.push_back({id,is_robot,fixed});
      universe.push_back({id,path,is_robot,fixed});return snapshot.models.size()<=4096;
    });
    if(seen.size()!=background_.size())snapshot.background_matches=false;
    ecm.Each<sim::components::SystemPluginInfo>([&](const sim::Entity &id,const auto *systems) {
      if(id==world_ && systems->Data().plugins_size()>0)snapshot.world_metadata=true;
      for(const auto &plugin:systems->Data().plugins()) {
        snapshot.plugins.push_back({id,plugin.name(),plugin.filename()});
        plugins.push_back({{"entity",id},{"name",plugin.name()},{"filename",plugin.filename()}});
        if(snapshot.plugins.size()>256)return false;
      }return true;
    });
    ecm.Each<sim::components::DetachableJoint>([&](const auto &,const auto *) {++snapshot.detachable_joints;return false;});
    ecm.Each<sim::components::Joint>([&](const sim::Entity &id,const auto *) {
      const auto *parent=ecm.Component<sim::components::ParentLinkName>(id);
      const auto *child=ecm.Component<sim::components::ChildLinkName>(id);
      const auto *owner=ecm.Component<sim::components::ParentEntity>(id);
      const auto *name=ecm.Component<sim::components::Name>(id);
      if(owner && owner->Data()==robot) {
        if(!name || !parent || !child || !live_joints.emplace(name->Data(),std::make_pair(parent->Data(),child->Data())).second)
          robot_entities_valid=false;
        else bind_robot("joint::"+name->Data(),id);
      }
      if(robot==sim::kNullEntity || !member(id,robot,ecm) || !parent || !child ||
         parent->Data()=="world" || parent->Data().find("::")!=std::string::npos || child->Data().find("::")!=std::string::npos)
        ++snapshot.external_joints;
      return true;
    });
    snapshot.robot_structure_matches=robot_entities_valid &&
      simulation::robot_structure_matches(robot_links_,robot_joints_,live_links,live_joints);
    std::string reason=simulation::empty_inventory_reason(snapshot);
    Observation value;nlohmann::json execution_versions=nlohmann::json::array();
    if(policy_=="kinematic_inventory_v1") {
      reason=simulation::kinematic_inventory_reason(snapshot,payloads,capture);
      if(payloads.size()!=payload_registry_.size())payload_error="PAYLOAD_REGISTRY_INCOMPLETE";
      if(!payload_error.empty())reason=payload_error;
      if(reason=="EMPTY_INVENTORY_OBSERVED" || reason=="ATTACHED_INVENTORY_OBSERVED")try {
        for(const auto &[id,p]:payloads) {
          const auto &entry=payload_registry_.at(payload_names.at(id));
          const auto *name=ecm.Component<sim::components::Name>(p.parent);
          const auto *parent=ecm.Component<sim::components::ParentEntity>(p.parent);
          if(p.parent_model!=robot_ || p.parent_link!=entry.parent || !name || name->Data()!=entry.parent || !parent || parent->Data()!=robot)
            throw std::invalid_argument("PAYLOAD_PARENT_IDENTITY_MISMATCH");
          execution_versions.push_back({{"entity",id},{"epoch",p.epoch},{"clock_epoch",p.clock_epoch},
            {"accepted",p.accepted},{"applied",p.applied},{"attached",p.attached},{"parent",p.parent}});
          if(p.attached)value.objects.push_back(simulation::observed_payload(ecm,id,p,entry.object_id,entry.tcp,entry.parent_from_tcp));
        }
      }catch(const std::exception &error) {reason=error.what();value.objects.clear();}
    }
    bool complete=reason=="EMPTY_INVENTORY_OBSERVED" || reason=="ATTACHED_INVENTORY_OBSERVED";
    nlohmann::json object_geometry=nlohmann::json::array();
    try {object_geometry=payload::canonical(value.objects,{"astribot_arm_left_tcp_link","astribot_arm_right_tcp_link"});}
    catch(const std::exception &error) {reason=error.what();complete=false;value.objects.clear();}
    const auto semantic=nlohmann::json{{"models",universe},{"plugins",plugins},{"reason",reason},{"execution",execution_versions},{"object_geometry",object_geometry},
      {"detachable_joints",snapshot.detachable_joints},{"external_joints",snapshot.external_joints},{"background_matches",snapshot.background_matches},{"robot_structure_matches",snapshot.robot_structure_matches}};
    try {
      const auto v=version_.sample(capture,semantic.dump());
      value.environment="simulation";value.session_id=session_;value.source_id=source_;
      value.source_epoch=epoch_;value.clock_epoch=v.clock_epoch;value.sequence=v.sequence;value.revision=v.revision;
      value.observed_at=stamp(capture);value.valid_until=stamp(capture+300000000);
      value.full_inventory=complete;value.status=complete?(value.objects.empty()?Observation::EMPTY:Observation::ATTACHED):Observation::UNKNOWN;
      value.transaction_id=epoch_+":"+std::to_string(v.revision);
      auto detail=semantic;detail["stamp_ns"]=capture;detail["source_epoch"]=epoch_;detail["revision"]=v.revision;
      detail["sequence"]=v.sequence;detail["clock_epoch"]=v.clock_epoch;detail["world"]=world_name_;
      detail["robot_model"]=robot_;detail["policy"]=policy_;
      std_msgs::msg::String message;message.data=detail.dump();
      const auto wall=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
      Pending current{value,message,v,capture,capture+300000000,wall};
      if(!complete) {
        // Revoke immediately; an older EMPTY must not survive a changed inventory.
        pending_.reset();publish(current);
      } else {
        // Gazebo PostUpdate may reach DDS before its matching /clock. Publish
        // the previous capture only if this new physics capture confirms the
        // same inventory. Original capture/deadline are never rewritten.
        if(pending_ && simulation::requires_revocation(pending_->version,v)) {
          auto revoked=current;revoked.observation.full_inventory=false;
          revoked.observation.status=Observation::TRANSITION;revoked.observation.objects.clear();
          auto diagnostic=nlohmann::json::parse(revoked.diagnostic.data);
          diagnostic["reason"]="INVENTORY_CHANGED_RECONCILIATION_REQUIRED";
          revoked.diagnostic.data=diagnostic.dump();publish(revoked);pending_.reset();
          // Recovery needs a capture strictly newer than the revocation. Do
          // not later republish this same capture as positive evidence.
          return;
        }
        if(pending_ && simulation::publication_window(pending_->version,v,pending_->capture,capture,
                                                     pending_->deadline,pending_->wall,wall))publish(*pending_);
        pending_=std::move(current);
      }
    }catch(const std::exception &error) {
      // Silence expires the existing lease. Never renew with a synthetic capture.
      RCLCPP_ERROR(node_->get_logger(),"Inventory capture rejected: %s",error.what());
    }
  }
  ~EmptyInventory() override {
    observations_.reset();diagnostics_.reset();node_.reset();if(context_)context_->shutdown("inventory plugin unloaded");
  }
};
}
IGNITION_ADD_PLUGIN(astribot::EmptyInventory,sim::System,sim::ISystemConfigure,sim::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(astribot::EmptyInventory,"astribot::EmptyInventory")
