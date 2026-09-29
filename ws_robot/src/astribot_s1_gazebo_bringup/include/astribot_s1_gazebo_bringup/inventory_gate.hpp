#pragma once
#include <cstdint>
#include <limits>
#include <map>
#include <algorithm>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace astribot::simulation {
// Deliberately EMPTY-only. Any mechanism that could carry an unenumerated object
// blocks the claim. This is a closed-world simulation policy, not contact grasp detection.
struct InventoryModel {uint64_t id;bool robot_member,is_static;};
struct InventoryPlugin {uint64_t entity;std::string name,filename;};
struct InventorySnapshot {
  bool world_present=false,world_metadata=false,model_metadata_missing=false,background_matches=false,robot_structure_matches=false;
  uint64_t world_entity=0;
  unsigned robot_count=0,detachable_joints=0,external_joints=0;
  std::vector<InventoryModel> models;
  std::vector<InventoryPlugin> plugins;
};
inline std::string library_name(std::string value) {
  const auto slash=value.find_last_of('/');if(slash!=std::string::npos)value.erase(0,slash+1);
  if(value.rfind("lib",0)==0)value.erase(0,3);
  const auto so=value.find(".so");if(so!=std::string::npos)value.erase(so);
  return value;
}
inline bool known_nonattachment_plugin(const InventoryPlugin &p,uint64_t world_entity) {
  if(p.name=="astribot::HeightSliceMap")
    return world_entity!=0 && p.entity==world_entity &&
      p.filename.substr(p.filename.find_last_of('/')+1)=="libastribot_height_slice_map.so";
  auto lib=library_name(p.filename);auto name=p.name;
  if(name.rfind("gz::sim::",0)==0)name.replace(0,9,"ignition::gazebo::");
  if(lib.rfind("gz-sim-",0)==0)lib.replace(0,7,"ignition-gazebo-");
  const std::set<std::pair<std::string,std::string>> allowed={
    {"ignition::gazebo::systems::Physics","ignition-gazebo-physics-system"},
    {"ignition::gazebo::systems::UserCommands","ignition-gazebo-user-commands-system"},
    {"ignition::gazebo::systems::SceneBroadcaster","ignition-gazebo-scene-broadcaster-system"},
    {"ignition::gazebo::systems::Sensors","ignition-gazebo-sensors-system"},
    {"ignition::gazebo::systems::Imu","ignition-gazebo-imu-system"},
    {"ignition::gazebo::systems::OdometryPublisher","ignition-gazebo-odometry-publisher-system"},
    {"ignition::gazebo::systems::PosePublisher","ignition-gazebo-pose-publisher-system"},
    {"ignition::gazebo::systems::JointStatePublisher","ignition-gazebo-joint-state-publisher-system"},
    {"gz_ros2_control::GazeboSimROS2ControlPlugin","gz_ros2_control-system"},
    {"astribot::ContactEvidence","astribot_contact_evidence"},
    {"astribot::EmptyInventory","astribot_empty_inventory"}};
  return allowed.count({name,lib});
}
using JointConnections=std::map<std::string,std::pair<std::string,std::string>>;
inline bool robot_structure_matches(const std::set<std::string> &reference_links,const JointConnections &reference_joints,
    const std::set<std::string> &actual_links,const JointConnections &actual_joints) {
  return !reference_links.empty() && reference_links==actual_links && reference_joints==actual_joints;
}
inline std::string empty_inventory_reason(const InventorySnapshot &s) {
  if(!s.world_present)return "WORLD_UNAVAILABLE";
  if(s.robot_count!=1)return "ROBOT_IDENTITY_AMBIGUOUS";
  if(!s.world_metadata)return "WORLD_PLUGIN_INVENTORY_UNAVAILABLE";
  if(s.models.size()>4096 || s.plugins.size()>256)return "INVENTORY_LIMIT_EXCEEDED";
  if(s.model_metadata_missing)return "MODEL_METADATA_UNAVAILABLE";
  if(!s.robot_structure_matches || std::count_if(s.models.begin(),s.models.end(),
      [](const auto &m){return m.robot_member;})!=1)return "ROBOT_STRUCTURE_MISMATCH";
  if(!s.background_matches)return "BACKGROUND_MANIFEST_MISMATCH";
  if(s.detachable_joints)return "UNSUPPORTED_DETACHABLE_JOINT";
  if(s.external_joints)return "UNSUPPORTED_EXTERNAL_JOINT";
  std::set<uint64_t> ids;
  for(const auto &m:s.models) {
    if(!m.id || !ids.insert(m.id).second)return "DUPLICATE_MODEL_ID";
    if(!m.robot_member && !m.is_static)return "UNSUPPORTED_DYNAMIC_MODEL";
  }
  for(const auto &p:s.plugins)if(!known_nonattachment_plugin(p,s.world_entity))return "UNSUPPORTED_SYSTEM_PLUGIN";
  return "EMPTY_INVENTORY_OBSERVED";
}
struct Version {uint64_t clock_epoch=0,sequence=0,revision=0;};
inline bool requires_revocation(const Version &previous,const Version &current) {
  return current.clock_epoch!=previous.clock_epoch || current.revision!=previous.revision;
}
inline bool publication_window(const Version &previous,const Version &current,int64_t captured,int64_t now,
    int64_t deadline,int64_t received_wall,int64_t wall) {
  return current.clock_epoch==previous.clock_epoch && current.revision==previous.revision &&
    current.sequence>previous.sequence && captured>=0 && now>captured && now<deadline &&
    deadline>captured && deadline-captured<=300000000 && wall>=received_wall &&
    wall-received_wall<deadline-captured;
}
class InventoryVersion {
  Version v_;int64_t last_=-1;std::string content_;
public:
  Version sample(int64_t capture,const std::string &content) {
    if(capture<0 || capture==last_)throw std::invalid_argument("NEW_PHYSICS_CAPTURE_REQUIRED");
    if(v_.sequence==UINT64_MAX || v_.revision==UINT64_MAX || v_.clock_epoch==UINT64_MAX)
      throw std::overflow_error("INVENTORY_VERSION_EXHAUSTED");
    const bool reset=last_>=0 && capture<last_;
    if(reset)++v_.clock_epoch;
    if(v_.revision==0 || reset || content!=content_)++v_.revision;
    ++v_.sequence;last_=capture;content_=content;return v_;
  }
};
} // namespace astribot::simulation
