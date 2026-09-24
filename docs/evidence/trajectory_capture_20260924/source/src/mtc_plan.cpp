#include "astribot_s1_transport_native/mtc_plan.hpp"
#include <cmath>
#include <map>
#include <stdexcept>

namespace astribot::transport {
MtcPlan::MtcPlan(const std::string &operation,std::vector<Stage> stages,
 const std::string &context,const std::string &returned_context,int64_t created,
 std::set<std::string> joints):stages_(std::move(stages)),created_(created),joints_(std::move(joints)) {
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
}
