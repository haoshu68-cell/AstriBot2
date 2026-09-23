#include "astribot_s1_gazebo_bringup/kinematic_inventory.hpp"
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Serialization.hh>
#include <nlohmann/json.hpp>
namespace ignition::gazebo::components {
using AstribotPayloadExecution=Component<std::string,class AstribotPayloadExecutionTag,serializers::StringSerializer>;
IGN_GAZEBO_REGISTER_COMPONENT("astribot.components.PayloadExecution.v1",AstribotPayloadExecution)
}
namespace astribot::simulation {
void write_execution(uint64_t model,ignition::gazebo::EntityComponentManager &ecm,const PayloadExecution &p) {
  const auto &v=p.offset.Pos();const auto &q=p.offset.Rot();
  nlohmann::json j={{"epoch",p.epoch},{"parent_model",p.parent_model},{"parent_link",p.parent_link},
    {"error",p.error},{"parent",p.parent},{"accepted",p.accepted},{"applied",p.applied},
    {"clock_epoch",p.clock_epoch},{"capture",p.capture},{"attached",p.attached},{"pending",p.pending},
    {"offset",{v.X(),v.Y(),v.Z(),q.X(),q.Y(),q.Z(),q.W()}}};
  ecm.SetComponentData<ignition::gazebo::components::AstribotPayloadExecution>(model,j.dump());
}
std::optional<PayloadExecution> read_execution(uint64_t model,const ignition::gazebo::EntityComponentManager &ecm) {
  const auto *component=ecm.Component<ignition::gazebo::components::AstribotPayloadExecution>(model);
  if(!component || component->Data().size()>4096)return {};
  try {
    const auto j=nlohmann::json::parse(component->Data());PayloadExecution p;
    p.epoch=j.at("epoch");p.parent_model=j.at("parent_model");p.parent_link=j.at("parent_link");p.error=j.at("error");
    p.parent=j.at("parent");p.accepted=j.at("accepted");p.applied=j.at("applied");p.clock_epoch=j.at("clock_epoch");
    p.capture=j.at("capture");p.attached=j.at("attached");p.pending=j.at("pending");
    const auto v=j.at("offset").get<std::vector<double>>();if(v.size()!=7)return {};
    for(double d:v)if(!std::isfinite(d))return {};
    const double norm=std::hypot(std::hypot(v[3],v[4]),std::hypot(v[5],v[6]));
    if(std::abs(norm-1.)>1e-6)return {};
    p.offset=ignition::math::Pose3d(ignition::math::Vector3d(v[0],v[1],v[2]),ignition::math::Quaterniond(v[6],v[3],v[4],v[5]));
    return p;
  }catch(const std::exception &) {return {};}
}
std::string kinematic_inventory_reason(const InventorySnapshot &input,const std::map<uint64_t,PayloadExecution> &payloads,int64_t capture) {
  if(payloads.empty() || payloads.size()>32)return "PAYLOAD_REGISTRY_INCOMPLETE";
  auto s=input;bool attached=false;std::set<uint64_t> seen;std::map<uint64_t,unsigned> executors;
  s.models.erase(std::remove_if(s.models.begin(),s.models.end(),[&](const auto &m) {
    if(!payloads.count(m.id))return false;
    if(m.robot_member || !seen.insert(m.id).second)s.model_metadata_missing=true;
    return true;
  }),s.models.end());
  if(seen.size()!=payloads.size())return "PAYLOAD_MODEL_UNAVAILABLE";
  s.plugins.erase(std::remove_if(s.plugins.begin(),s.plugins.end(),[&](const auto &p) {
    if(payloads.count(p.entity) && p.name=="astribot::KinematicPayload" && library_name(p.filename)=="astribot_kinematic_payload") {
      ++executors[p.entity];return true;
    }return false;
  }),s.plugins.end());
  const auto reason=empty_inventory_reason(s);if(reason!="EMPTY_INVENTORY_OBSERVED")return reason;
  for(const auto &[id,p]:payloads) {
    if(executors[id]!=1)return "PAYLOAD_EXECUTOR_AMBIGUOUS";
    if(p.epoch.empty() || p.epoch.size()>128 || p.capture!=capture || capture<0 || !p.offset.IsFinite())return "PAYLOAD_EXECUTION_STALE_OR_INVALID";
    if(p.pending || p.accepted!=p.applied || !p.error.empty())return "PAYLOAD_TRANSITION_OR_ERROR";
    if(p.parent_model.empty() || p.parent_link.empty() || (p.attached && (!p.parent || !p.applied)))return "PAYLOAD_PARENT_UNCONFIRMED";
    attached=attached || p.attached;
  }
  return attached?"ATTACHED_INVENTORY_OBSERVED":"EMPTY_INVENTORY_OBSERVED";
}
}
