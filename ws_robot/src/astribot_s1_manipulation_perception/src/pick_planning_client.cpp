#include <astribot_s1_manipulation_perception/pick_planning_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <mutex>
#include <thread>
#include "pending_action.hpp"

namespace astribot::perception_planning {
namespace {
using Steady=std::chrono::steady_clock;
using namespace std::chrono_literals;
using detail::Pending;
} // namespace

struct PickPlanningClient::Impl {
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp_action::Client<Pose>::SharedPtr pose;
  rclcpp_action::Client<Grasps>::SharedPtr grasps;
  rclcpp_action::Client<Plan>::SharedPtr planner;
  std::mutex mutex;
  bool unresolved{false};
  explicit Impl(rclcpp::Node::SharedPtr n):node(std::move(n)) {
    executor.add_node(node);
    pose=rclcpp_action::create_client<Pose>(node,"/perception/estimate_object_pose");
    grasps=rclcpp_action::create_client<Grasps>(node,"/perception/compute_grasps");
    planner=rclcpp_action::create_client<Plan>(node,"/transport/plan_manipulation");
  }
  void tick() {executor.spin_some();std::this_thread::sleep_for(2ms);}
};
PickPlanningClient::PickPlanningClient(rclcpp::Node::SharedPtr node):impl_(std::make_unique<Impl>(std::move(node))) {}
PickPlanningClient::~PickPlanningClient()=default;
bool PickPlanningClient::ready() const {
  return impl_->pose->action_server_is_ready()&&impl_->grasps->action_server_is_ready()&&
    impl_->planner->action_server_is_ready();
}

Outcome PickPlanningClient::plan(const Request &r,const std::function<Context()> &current,
                                const std::function<bool()> &cancel_requested) {
  auto &i=*impl_;Outcome out;
  std::unique_lock<std::mutex> lock(i.mutex,std::try_to_lock);
  if(!lock.owns_lock()) {out.reason="CLIENT_BUSY";out.terminal_confirmed=false;return out;}
  if(i.unresolved) {out.reason="PREVIOUS_ACTION_TERMINAL_UNCONFIRMED";out.terminal_confirmed=false;return out;}
  Pending<Pose> pose;pose.client=i.pose;
  Pending<Grasps> grasps;grasps.client=i.grasps;
  Pending<Plan> planning;planning.client=i.planner;
  const auto start=Steady::now();
  const auto first_now=i.node->now().nanoseconds();
  const auto deadline=r.result_deadline_steady;
  int64_t previous_now=first_now;
  auto check=[&] {
    if(cancel_requested())throw std::runtime_error("CANCELED");
    if(!rclcpp::ok(i.node->get_node_base_interface()->get_context()))throw std::runtime_error("ROS_CONTEXT_STOPPED");
    const auto now=i.node->now().nanoseconds();
    if(now<previous_now)throw std::runtime_error("CLOCK_ROLLBACK");
    previous_now=now;
    check_context(r,current(),now);
    if(Steady::now()>=deadline)throw std::runtime_error("ORIGINAL_DEADLINE_EXPIRED");
  };
  // Cleanup is bounded in wall time even with frozen simulation time. Every
  // pending goal is canceled by its own handle, never by cancel-all.
  auto cleanup=[&] {
    const auto until=Steady::now()+1s;
    while(Steady::now()<until) {
      pose.cancel();grasps.cancel();planning.cancel();
      if(pose.terminal()&&grasps.terminal()&&planning.terminal())return true;
      i.tick();
    }
    pose.poll();grasps.poll();planning.poll();
    return pose.terminal()&&grasps.terminal()&&planning.terminal();
  };
  try {
    check();
    // Require discovered servers before spending the <=0.5 s capture admission.
    if(!i.pose->action_server_is_ready()||!i.grasps->action_server_is_ready()||!i.planner->action_server_is_ready())
      throw std::runtime_error("ACTION_SERVER_NOT_READY");
    const auto goals=inference_goals(r,current(),i.node->now().nanoseconds());
    pose.send(goals.pose);grasps.send(goals.grasps);
    const auto inference_deadline=start+std::chrono::duration<double>(r.inference_timeout_s);
    while(true) {
      check();i.tick();pose.poll();grasps.poll();
      if(pose.rejected)throw std::runtime_error("POSE_GOAL_REJECTED");
      if(grasps.rejected)throw std::runtime_error("GRASP_GOAL_REJECTED");
      if(pose.received()&&!pose.terminal())throw std::runtime_error("POSE_TERMINAL_UNKNOWN");
      if(grasps.received()&&!grasps.terminal())throw std::runtime_error("GRASP_TERMINAL_UNKNOWN");
      // Surface one endpoint's failure immediately; cancel the other endpoint.
      if(pose.terminal()) {
        const auto p=pose.result.get();
        if(p.code!=rclcpp_action::ResultCode::SUCCEEDED||!p.result->success)
          throw std::runtime_error("POSE_FAILED:"+p.result->reason_code);
      }
      if(grasps.terminal()) {
        const auto g=grasps.result.get();
        if(g.code!=rclcpp_action::ResultCode::SUCCEEDED||!g.result->success)
          throw std::runtime_error("GRASP_FAILED:"+g.result->reason_code);
      }
      if(pose.terminal()&&grasps.terminal())break;
      if(Steady::now()>=inference_deadline)throw std::runtime_error("INFERENCE_TIMEOUT");
    }
    const auto object_pose=*pose.result.get().result;
    const auto proposals=grasps.result.get().result;
    if(proposals->candidates.empty())throw std::runtime_error("NO_GRASP_CANDIDATES");
    for(const auto &candidate:proposals->candidates) {
      check();
      Plan::Goal goal;
      GraspMapping mapping;
      try {goal=planning_goal(r,object_pose,candidate,current(),i.node->now().nanoseconds(),&mapping);}
      catch(const std::runtime_error &e) {
        if(std::string(e.what())!="GRIPPER_WIDTH_UNSUPPORTED"&&
           std::string(e.what()).rfind("BOX_GRASP_UNSUPPORTED:",0)!=0)throw;
        out.candidate_rejections.push_back(candidate.candidate_id+":"+e.what());continue;
      }
      planning=Pending<Plan>();planning.client=i.planner;planning.send(goal);
      const auto plan_deadline=Steady::now()+std::chrono::duration<double>(goal.timeout_s);
      while(!planning.terminal()) {
        check();i.tick();planning.poll();
        if(planning.received()&&!planning.terminal())throw std::runtime_error("MTC_TERMINAL_UNKNOWN");
        if(Steady::now()>=plan_deadline&&!planning.terminal())throw std::runtime_error("MTC_TIMEOUT");
      }
      if(planning.rejected)throw std::runtime_error("MTC_GOAL_REJECTED");
      const auto result=planning.result.get();check();
      if(result.code==rclcpp_action::ResultCode::ABORTED&&!result.result->success) {
        out.candidate_rejections.push_back(candidate.candidate_id+":"+result.result->reason);continue;
      }
      if(result.code!=rclcpp_action::ResultCode::SUCCEEDED)throw std::runtime_error("MTC_ACTION_NOT_SUCCEEDED");
      check_plan(goal,*result.result);
      out.pick=PlannedPick{r.context,r.object_cloud.header,r.valid_until,r.result_deadline_steady,candidate,object_pose,goal,*result.result,mapping,
        r.geometry_revision,r.grasp_registration_revision,r.grasp_registration_evidence};
      out.success=true;out.reason="MTC_PICK_PLANNED_REQUIRES_OWNER_EXECUTION_CHECKS";return out;
    }
    throw std::runtime_error("NO_MTC_FEASIBLE_CANDIDATE");
  } catch(const std::exception &e) {
    out.reason=e.what(); // Original failure is never replaced by cleanup status.
    try {out.terminal_confirmed=cleanup();}
    catch(const std::exception &cleanup_error) {
      out.terminal_confirmed=false;out.reason+=";CLEANUP_ERROR:"+std::string(cleanup_error.what());
    }
    i.unresolved=!out.terminal_confirmed;
    return out;
  }
}
} // namespace astribot::perception_planning
