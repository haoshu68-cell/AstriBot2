#pragma once
#include <astribot_transport_msgs/msg/manipulation_stage.hpp>
#include <set>
#include <string>
#include <vector>

namespace astribot::transport {
// The MTC result is an external input. Preserve the legacy PlanGuard's complete
// ordered operation, 120 s lifetime, actual-start and attachment-set contract.
// The owning executor acknowledges only after the corresponding physical event.
class MtcPlan {
public:
 using Stage=astribot_transport_msgs::msg::ManipulationStage;
 MtcPlan(const std::string &operation,std::vector<Stage> stages,
   const std::string &context,const std::string &returned_context,
   int64_t created_steady,std::set<std::string> owned_joints);
 const Stage &check(const std::string &name,const std::string &kind,int64_t steady,
   const sensor_msgs::msg::JointState &actual,const std::set<std::string> &attached) const;
 void acknowledge();
 bool complete() const{return index_==stages_.size();}
 size_t index() const{return index_;}
private:
 std::vector<Stage> stages_;
 int64_t created_;
 std::set<std::string> joints_;
 size_t index_=0;
};
}
