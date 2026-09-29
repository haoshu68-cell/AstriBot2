#include "astribot_s1_transport_native/mtc_plan.hpp"
#include <cmath>
#include <map>
#include <stdexcept>

namespace astribot::transport {
MtcPlan::MtcPlan(const std::string &operation,std::vector<Stage> stages,
 const std::string &context,const std::string &returned_context,int64_t created,
 std::set<std::string> joints):stages_(std::move(stages)),operation_(operation),created_(created),joints_(std::move(joints)) {
 if(context.empty()||context!=returned_context)throw std::runtime_error("MTC_CONTEXT_MISMATCH");
 const std::vector<std::pair<std::string,std::string>> pick={{"PREGRASP","ARM"},{"GRASP_APPROACH","ARM"},{"GRASP_CONFIRM","GRIPPER"},{"ATTACH_CONFIRM","ATTACH"},{"LIFT","ARM"},{"TRANSPORT_POSTURE","ARM"}};
 const std::vector<std::pair<std::string,std::string>> place={{"PREPLACE","ARM"},{"PLACE_APPROACH","ARM"},{"RELEASE","GRIPPER"},{"DETACH_CONFIRM","DETACH"},{"RETREAT","ARM"},{"STOW","ARM"}};
 if(operation!="PICK"&&operation!="PLACE")throw std::runtime_error("MTC_INCOMPLETE_SEQUENCE");
 const auto &order=operation=="PICK"?pick:place;
 if(stages_.size()!=order.size())throw std::runtime_error("MTC_INCOMPLETE_SEQUENCE");
 for(size_t i=0;i<order.size();++i)
  if(stages_[i].stage_id.substr(0,stages_[i].stage_id.find(':'))!=order[i].first||stages_[i].kind!=order[i].second)
   throw std::runtime_error("MTC_INCOMPLETE_SEQUENCE");
}
const MtcPlan::Stage &MtcPlan::check(const std::string &name,const std::string &kind,int64_t steady,
 const sensor_msgs::msg::JointState &actual,const std::set<std::string> &attached) const {
 if(complete())throw std::runtime_error("MTC_PLAN_ALREADY_CONSUMED");
 const auto &stage=stages_[index_];
 if(stage.stage_id.substr(0,stage.stage_id.find(':'))!=name||stage.kind!=kind)throw std::runtime_error("MTC_STAGE_ORDER");
 if(steady<created_||steady-created_>120000000000LL)throw std::runtime_error("MTC_PLAN_EXPIRED");
 const auto &expected=stage.expected_start.joint_state;
 if(joints_.empty()||expected.name.size()!=expected.position.size()||actual.name.size()!=actual.position.size())throw std::runtime_error("MTC_START_STATE_CHANGED");
 std::map<std::string,double> measured,planned;
 for(size_t i=0;i<actual.name.size();++i)
  if(!measured.emplace(actual.name[i],actual.position[i]).second)throw std::runtime_error("MTC_START_STATE_CHANGED");
 for(size_t i=0;i<expected.name.size();++i)
  if(!planned.emplace(expected.name[i],expected.position[i]).second)throw std::runtime_error("MTC_START_STATE_CHANGED");
 for(const auto &joint:joints_) {
  auto a=measured.find(joint),p=planned.find(joint);
  if(a==measured.end()||p==planned.end()||!std::isfinite(a->second)||!std::isfinite(p->second)||std::abs(a->second-p->second)>.025)
   throw std::runtime_error("MTC_START_STATE_CHANGED");
 }
 std::set<std::string> expected_attached;
 for(const auto &object:stage.expected_start.attached_collision_objects)expected_attached.insert(object.object.id);
 if(attached!=expected_attached)throw std::runtime_error("MTC_ATTACHMENT_CHANGED");
 return stage;
}
void MtcPlan::acknowledge(){
 if(complete())throw std::logic_error("MTC_PLAN_ALREADY_CONSUMED");
 ++index_;
}
void MtcPlan::adoptPayloadSuffix(const std::vector<Stage>& suffix,bool replanned,int64_t steady) {
 if(index_!=3||suffix.size()!=2||(replanned&&operation_!="PICK"))throw std::runtime_error("MTC_PAYLOAD_SUFFIX_SCOPE");
 if(steady<created_||steady-created_>120000000000LL)throw std::runtime_error("MTC_PLAN_EXPIRED");
 auto candidate=stages_;
 for(size_t i=0;i<suffix.size();++i) {
  const auto& previous=stages_[i+4];const auto& next=suffix[i];
  if(next.stage_id!=previous.stage_id||next.kind!=previous.kind||
     next.expected_start.joint_state.name!=previous.expected_start.joint_state.name||
     next.expected_start.joint_state.position!=previous.expected_start.joint_state.position||
     next.expected_start.multi_dof_joint_state!=previous.expected_start.multi_dof_joint_state)
   throw std::runtime_error("MTC_PAYLOAD_SUFFIX_STAGE_CHANGED");
  std::set<std::pair<std::string,std::string>> before_bodies,after_bodies;
  for(const auto& body:previous.expected_start.attached_collision_objects)before_bodies.emplace(body.object.id,body.link_name);
  for(const auto& body:next.expected_start.attached_collision_objects)after_bodies.emplace(body.object.id,body.link_name);
  if(before_bodies!=after_bodies)throw std::runtime_error("MTC_PAYLOAD_SUFFIX_ATTACHMENT_CHANGED");
  if(!replanned||i==0) {
   if(next.trajectory!=previous.trajectory)throw std::runtime_error("MTC_PAYLOAD_UNCHANGED_PATH_CHANGED");
  }else {
   const auto& path=next.trajectory.joint_trajectory;const auto& old=previous.trajectory.joint_trajectory;
   if(path.joint_names!=old.joint_names||path.points.empty()||old.points.empty()||
      next.trajectory.multi_dof_joint_trajectory!=previous.trajectory.multi_dof_joint_trajectory)
    throw std::runtime_error("MTC_PAYLOAD_REPLANNED_PATH_INVALID");
   int64_t previous_time=-1;
   for(const auto& point:path.points) {
    const auto time=int64_t(point.time_from_start.sec)*1000000000+point.time_from_start.nanosec;
    if(time<0||time<=previous_time||time>120000000000LL||point.positions.size()!=path.joint_names.size()||
       (!point.velocities.empty()&&point.velocities.size()!=point.positions.size())||
       (!point.accelerations.empty()&&point.accelerations.size()!=point.positions.size())||!point.effort.empty())
     throw std::runtime_error("MTC_PAYLOAD_REPLANNED_PATH_INVALID");
    for(const auto* values:{&point.positions,&point.velocities,&point.accelerations})
     for(double v:*values)if(!std::isfinite(v))throw std::runtime_error("MTC_PAYLOAD_REPLANNED_PATH_INVALID");
    previous_time=time;
   }
   for(size_t j=0;j<path.joint_names.size();++j) {
    // The start is copied from the validated LIFT endpoint; the planner may
    // finish within its existing joint-goal tolerance, never the 25 mrad gate.
    if(std::abs(path.points.front().positions[j]-old.points.front().positions.at(j))>1e-9||
       std::abs(path.points.back().positions[j]-old.points.back().positions.at(j))>1e-4)
     throw std::runtime_error("MTC_PAYLOAD_REPLANNED_ENDPOINT_CHANGED");
   }
  }
  candidate[i+4]=next;
 }
 stages_.swap(candidate);
}
}
