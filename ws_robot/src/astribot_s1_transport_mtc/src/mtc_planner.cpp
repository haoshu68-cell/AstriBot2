#include <atomic>
#include <mutex>
#include <thread>
#include <sstream>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/robot_state/conversions.h>
#include <astribot_transport_msgs/action/plan_manipulation.hpp>
#include <astribot_transport_msgs/srv/revalidate_manipulation.hpp>
#include <astribot_s1_manipulation/external_trajectory_validator.hpp>
#include <astribot_s1_manipulation/gripper_commander.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "astribot_s1_transport_mtc/joint_margin.hpp"
#include "astribot_s1_transport_mtc/canonical_octomap.hpp"
#include "astribot_s1_transport_mtc/payload_transition.hpp"

namespace mtc=moveit::task_constructor;
using Action=astribot_transport_msgs::action::PlanManipulation;
using Handle=rclcpp_action::ServerGoalHandle<Action>;
using Segment=astribot_transport_msgs::msg::ManipulationStage;
using namespace astribot_s1_manipulation;

// Preserve the rejected path and add a read-only next-state diagnostic. This
// never turns a partial Cartesian solution into an executable trajectory.
class AuditedCartesianPath : public mtc::solvers::CartesianPath {
public:
  using CartesianPath::plan;
  Result plan(const planning_scene::PlanningSceneConstPtr& from,
              const moveit::core::LinkModel& link,const Eigen::Isometry3d& offset,
              const Eigen::Isometry3d& target,const moveit::core::JointModelGroup* group,
              double timeout,robot_trajectory::RobotTrajectoryPtr& trajectory,
              const moveit_msgs::msg::Constraints& constraints) override {
    auto result=CartesianPath::plan(from,link,offset,target,group,timeout,trajectory,constraints);
    if(!result && trajectory && trajectory->getWayPointCount()) {
      auto state=trajectory->getLastWayPoint();
      auto next=state.getGlobalLinkTransform(&link)*offset;
      const Eigen::Vector3d delta=target.translation()-next.translation();
      const double fraction=std::min(1.,.003/std::max(1e-9,delta.norm()));
      next.translation()+=fraction*delta;
      next.linear()=Eigen::Quaterniond(next.linear()).slerp(fraction,Eigen::Quaterniond(target.linear())).toRotationMatrix();
      if(!state.setFromIK(group,next*offset.inverse(),link.getName(),.02))
        result.message+="; diagnostic_next_state=IK_UNAVAILABLE";
      else {
        state.update();CollisionValidator validator;std::string error;
        if(validator.configure(from,CollisionParams{},error)) {
          auto report=validator.check(state,group->getName());
          result.message+="; diagnostic_next_state="+(report.collision?report.reason:std::string("VALID_OR_DIFFERENT_IK_BRANCH"));
        }
      }
    }
    if(!result) RCLCPP_WARN(rclcpp::get_logger("transport_mtc_cartesian"),"%s",result.message.c_str());
    return result;
  }
};

class Planner {
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  std::atomic<bool> busy_{false};
  std::thread worker_;
  std::mutex mutex_;
  mtc::Task* active_{nullptr};
  using Revalidate=astribot_transport_msgs::srv::RevalidateManipulation;
  using RevalidatePayload=astribot_transport_msgs::srv::RevalidatePayloadTransition;
  using CachedStage=astribot_s1_transport_mtc::CachedStage;
  rclcpp::Service<Revalidate>::SharedPtr revalidate_service_;
  rclcpp::Service<RevalidatePayload>::SharedPtr payload_revalidate_service_;
  astribot_s1_transport_mtc::PayloadTransitionBinding cached_payload_;
  std::vector<CachedStage> cached_stages_;
  std::string cached_context_;
  octomap_msgs::msg::OctomapWithPose cached_octomap_metadata_;
  std::chrono::steady_clock::time_point cached_at_;
  double velocity_scaling_{.1}, acceleration_scaling_{.1};
  double joint_margin_{.1};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  static void flatten(const mtc::SolutionBase& solution,std::vector<const mtc::SubTrajectory*>& out) {
    if(auto s=dynamic_cast<const mtc::SubTrajectory*>(&solution)) out.push_back(s);
    else if(auto s=dynamic_cast<const mtc::SolutionSequence*>(&solution))
      for(auto child:s->solutions()) flatten(*child,out);
    else if(auto s=dynamic_cast<const mtc::WrappedSolution*>(&solution)) flatten(*s->wrapped(),out);
    else throw std::runtime_error("UNSUPPORTED_SOLUTION_TYPE");
  }

  void revalidate(const Revalidate::Request& request,Revalidate::Response& response) {
    response.context_id=request.context_id;
    bool free=false;
    if(!busy_.compare_exchange_strong(free,true)) {response.reason="MTC_REVALIDATION_BUSY";return;}
    struct Release {std::atomic<bool>& busy;~Release(){busy=false;}} release{busy_};
    try {
      if(request.context_id.empty() || request.scene.is_diff || request.scene.robot_state.is_diff)
        throw std::runtime_error("MTC_REVALIDATION_FULL_SCENE_REQUIRED");
      std::vector<CachedStage> stages;
      octomap_msgs::msg::OctomapWithPose metadata;
      std::chrono::steady_clock::time_point expires;
      {
        std::lock_guard<std::mutex> guard(mutex_);
        if(cached_context_!=request.context_id || cached_stages_.empty())
          throw std::runtime_error("MTC_REVALIDATION_CONTEXT_UNKNOWN");
        expires=cached_at_+std::chrono::seconds(120);
        if(std::chrono::steady_clock::now()>=expires)
          throw std::runtime_error("MTC_REVALIDATION_CONTEXT_EXPIRED");
        if(request.start_index>=cached_stages_.size())
          throw std::runtime_error("MTC_REVALIDATION_INDEX_INVALID");
        stages=cached_stages_;
        metadata=cached_octomap_metadata_;
      }
      const auto& map=request.scene.world.octomap;
      if(map.header.frame_id!="astribot_torso_base")
        throw std::runtime_error("MTC_REVALIDATION_OCTOMAP_FRAME_INVALID");
      if(map.header.frame_id!=metadata.header.frame_id || map.origin!=metadata.origin ||
         map.octomap.header.frame_id!=metadata.octomap.header.frame_id ||
         map.octomap.id!=metadata.octomap.id || map.octomap.resolution!=metadata.octomap.resolution ||
         map.octomap.binary!=metadata.octomap.binary)
        throw std::runtime_error("MTC_REVALIDATION_OCTOMAP_METADATA_CHANGED");
      const auto& p=map.origin.position;const auto& q=map.origin.orientation;
      if(!std::isfinite(p.x+p.y+p.z+q.x+q.y+q.z+q.w) ||
         std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.001)
        throw std::runtime_error("MTC_REVALIDATION_OCTOMAP_ORIGIN_INVALID");
      const auto& octomap=map.octomap;
      const std::string bytes(octomap.data.begin(),octomap.data.end());
      (void)astribot::transport::canonical_octomap(bytes,octomap.binary,octomap.resolution,octomap.id);
      const auto deadline=std::min(expires,std::chrono::steady_clock::now()+std::chrono::seconds(10));
      for(std::size_t index=request.start_index;index<stages.size();++index) {
        const auto& stage=stages[index];
        if(!stage.trajectory) continue; // Attachment transitions have no motion.
        auto scene=planning_scene::PlanningScene::clone(stage.scene);
        scene->processOctomapMsg(map);
        robot_trajectory::RobotTrajectory trajectory(*stage.trajectory,true);
        std::string reason;
        if(!validateExternalTrajectory(scene,trajectory,reason,velocity_scaling_,acceleration_scaling_))
          throw std::runtime_error("MTC_REVALIDATION:"+stage.id+":"+reason);
        if(std::chrono::steady_clock::now()>=deadline)
          throw std::runtime_error("MTC_REVALIDATION_BUDGET_EXHAUSTED");
      }
      response.success=true;response.reason="MTC_REMAINING_SEQUENCE_REVALIDATED";
    } catch(const std::exception& error) {response.success=false;response.reason=error.what();}
  }

  void run(const std::shared_ptr<Handle>& handle) {
    auto result=std::make_shared<Action::Result>();
    const auto goal=handle->get_goal();result->context_id=goal->context_id;
    {std::lock_guard<std::mutex> guard(mutex_);cached_context_.clear();cached_stages_.clear();cached_payload_={};}
    try {
      if((goal->operation!="PICK" && goal->operation!="PLACE") || goal->object_id.empty() ||
         goal->context_id.empty() || goal->exit_targets.empty() || goal->exit_targets.size()>3 ||
         !std::isfinite(goal->timeout_s) || goal->timeout_s<=0 || goal->timeout_s>60 ||
         !std::isfinite(goal->grasp_width_m) || goal->grasp_width_m<=0 || goal->touch_links.empty())
        throw std::runtime_error("INVALID_REQUEST");
      auto pose_valid=[](const geometry_msgs::msg::PoseStamped& p) {
        const auto& v=p.pose.position;const auto& q=p.pose.orientation;
        return p.header.frame_id=="astribot_torso_base" && std::isfinite(v.x+v.y+v.z) &&
          std::isfinite(q.x+q.y+q.z+q.w) && std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)<.001;
      };
      if(!pose_valid(goal->target)||!pose_valid(goal->pre_target))throw std::runtime_error("INVALID_POSE");
      for(const auto& p:goal->exit_targets)if(!pose_valid(p))throw std::runtime_error("INVALID_EXIT_POSE");
      const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(goal->timeout_s);
      // Replan the complete sequence when a safety validator rejects a candidate.
      // No trajectory or scene mutation is sent to an executor by this node.
      for(unsigned attempt=1;attempt<=8 && !handle->is_canceling();++attempt) {
        const double remaining=std::chrono::duration<double>(deadline-std::chrono::steady_clock::now()).count();
        if(remaining<=0)break;
        mtc::Task task("transport_mtc",true);
        task.setName(goal->operation);task.loadRobotModel(node_);task.setTimeout(remaining);
        {std::lock_guard<std::mutex> guard(mutex_);active_=&task;}
        // Always clear active_ before task destruction, including exceptional exits.
        struct Clear {Planner* p;~Clear(){std::lock_guard<std::mutex> g(p->mutex_);p->active_=nullptr;}} clear{this};
        auto scene=std::make_shared<planning_scene::PlanningScene>(task.getRobotModel());
        scene->setPlanningSceneMsg(goal->scene);
        if(scene->getPlanningFrame()!="astribot_torso_base") throw std::runtime_error("FIXED_BASE_FRAME_REQUIRED");
        const bool attached=scene->getCurrentState().hasAttachedBody(goal->object_id);
        if((goal->operation=="PLACE")!=attached ||
           (!attached && !scene->getWorld()->hasObject(goal->object_id)))throw std::runtime_error("OBJECT_STATE_MISMATCH");
        for(const auto& link:goal->touch_links)if(!task.getRobotModel()->hasLinkModel(link))throw std::runtime_error("UNKNOWN_TOUCH_LINK");
        auto snapshot=std::make_unique<mtc::stages::FixedState>("SNAPSHOT",scene);
        auto* snapshot_stage=snapshot.get();task.add(std::move(snapshot));
        auto pipeline=std::make_shared<mtc::solvers::PipelinePlanner>(node_);
        pipeline->setPlannerId("RRTConnectConfig");pipeline->setTimeout(3.);
        auto cartesian=std::make_shared<AuditedCartesianPath>();
        cartesian->setStepSize(.003);cartesian->setMinFraction(1.);cartesian->setJumpThreshold(1.5);
        auto joints=std::make_shared<mtc::solvers::JointInterpolationPlanner>();
        moveit_msgs::msg::Constraints arm_constraints;
        const auto* arm_group=task.getRobotModel()->getJointModelGroup("arm_left");
        for (const auto& name:arm_group->getVariableNames()) {
          const auto& bounds=task.getRobotModel()->getVariableBounds(name);
          if (!bounds.position_bounded_) continue;
          const auto interval=astribot_s1_transport_mtc::planningInterval(
            bounds.min_position_,bounds.max_position_,joint_margin_);
          if (!interval.contains(scene->getCurrentState().getVariablePosition(name)))
            throw std::runtime_error("MTC_START_OUTSIDE_PLANNING_MARGIN:"+name);
          moveit_msgs::msg::JointConstraint constraint;
          constraint.joint_name=name;constraint.position=(interval.lower+interval.upper)/2.;
          constraint.tolerance_above=constraint.tolerance_below=(interval.upper-interval.lower)/2.;
          constraint.weight=1.;arm_constraints.joint_constraints.push_back(constraint);
        }
        for(const auto& solver:std::vector<mtc::solvers::PlannerInterfacePtr>{pipeline,cartesian,joints}) {
          solver->setMaxVelocityScalingFactor(velocity_scaling_);
          solver->setMaxAccelerationScalingFactor(acceleration_scaling_);
        }
        GripperCommander gripper(node_);GripperConfig gc;
        gc.group_name="gripper_left";gc.action_name="/gripper_left_controller/follow_joint_trajectory";
        gc.tcp_link="astribot_arm_left_tcp_link";gc.left_pad_link="astribot_gripper_left_Link_L11";
        gc.right_pad_link="astribot_gripper_left_Link_R11";
        std::string error;
        if(gripper.configureForPlanning(task.getRobotModel(),gc,error)!=PlanErrorCode::kSuccess)throw std::runtime_error(error);
        double grasp_angle;
        if(gripper.graspAngleForWidth(goal->grasp_width_m,grasp_angle,error)!=PlanErrorCode::kSuccess)throw std::runtime_error(error);
        auto sequence=std::make_unique<mtc::SerialContainer>(goal->operation+"_SEQUENCE");
        {
          // Solve pose IK with the joint margin first. Combining a Cartesian
          // goal directly with all joint path constraints can select OMPL's
          // joint sampler, which does not solve the Cartesian target.
          const std::string prefix=goal->operation=="PICK"?"PREGRASP":"PREPLACE";
          auto connect=std::make_unique<mtc::stages::Connect>(prefix,
            mtc::stages::Connect::GroupPlannerVector{{"arm_left",pipeline}});
          connect->setTimeout(3.);connect->setPathConstraints(arm_constraints);task.add(std::move(connect));
          auto pose=std::make_unique<mtc::stages::GeneratePose>("_"+prefix+"_POSE");
          pose->setPose(goal->pre_target);pose->setMonitoredStage(snapshot_stage);
          auto ik=std::make_unique<mtc::stages::ComputeIK>("_"+prefix+"_IK",std::move(pose));
          ik->setGroup("arm_left");ik->setIKFrame("astribot_arm_left_tcp_link");
          // The model's 10 ms single-query IK default is too short to sample
          // alternative branches for an entire approach/release/retreat chain.
          ik->setTimeout(2.);
          ik->setMaxIKSolutions(32);ik->setMinSolutionDistance(.05);ik->setIgnoreCollisions(false);
          ik->setProperty("constraints",arm_constraints);
          ik->properties().configureInitFrom(mtc::Stage::INTERFACE,{"target_pose"});
          sequence->insert(std::move(ik));
        }
        auto arm=[&](const std::string& name,const geometry_msgs::msg::PoseStamped& pose,
                     const mtc::solvers::PlannerInterfacePtr& solver) {
          auto stage=std::make_unique<mtc::stages::MoveTo>(name,solver);stage->setGroup("arm_left");
          stage->setPathConstraints(arm_constraints);
          stage->setTimeout(3.);
          stage->setIKFrame("astribot_arm_left_tcp_link");stage->setGoal(pose);sequence->insert(std::move(stage));
        };
        auto hand=[&](const std::string& name,double angle) {
          auto stage=std::make_unique<mtc::stages::MoveTo>(name,joints);stage->setGroup("gripper_left");
          stage->setGoal(std::map<std::string,double>{{gripper.jointName(),angle}});sequence->insert(std::move(stage));
        };
        auto attachment=[&](bool attach) {
          auto stage=std::make_unique<mtc::stages::ModifyPlanningScene>(attach?"ATTACH_CONFIRM":"DETACH_CONFIRM");
          stage->setCallback([goal,attach](const planning_scene::PlanningScenePtr& state,const mtc::PropertyMap&) {
            moveit_msgs::msg::AttachedCollisionObject object;object.link_name="astribot_arm_left_tcp_link";
            object.touch_links=goal->touch_links;object.object.id=goal->object_id;
            object.object.operation=attach?moveit_msgs::msg::CollisionObject::ADD:moveit_msgs::msg::CollisionObject::REMOVE;
            if(!state->processAttachedCollisionObjectMsg(object))throw std::runtime_error("PREDICTED_ATTACHMENT_FAILED");
          });sequence->insert(std::move(stage));
        };
        if(goal->operation=="PICK") {
          arm("GRASP_APPROACH",goal->target,cartesian);
          // Intentional grasp contact is limited to this payload and the two
          // finger pads, plus payload/support contact during lift-off.
          // It exists only in the predicted planning scene, never
          // as a global collision-disable request to the running MoveIt scene.
          const auto original_acm=scene->getAllowedCollisionMatrix();
          auto contact=std::make_unique<mtc::stages::ModifyPlanningScene>("_CONTACT_ALLOW");
          contact->allowCollisions(goal->object_id,
            std::vector<std::string>{gc.left_pad_link,gc.right_pad_link},true);
          contact->allowCollisions(goal->object_id,goal->object_id+"_pick_station",true);
          sequence->insert(std::move(contact));
          hand("GRASP_CONFIRM",grasp_angle);attachment(true);
          arm("LIFT",goal->exit_targets.front(),cartesian);
          auto restore=std::make_unique<mtc::stages::ModifyPlanningScene>("_CONTACT_RESTORE");
          restore->setCallback([original_acm](const planning_scene::PlanningScenePtr& state,const mtc::PropertyMap&) {
            state->getAllowedCollisionMatrixNonConst()=original_acm;
          });sequence->insert(std::move(restore));
        } else {
          const auto original_acm=scene->getAllowedCollisionMatrix();
          auto support=std::make_unique<mtc::stages::ModifyPlanningScene>("_SUPPORT_ALLOW");
          support->allowCollisions(goal->object_id,goal->object_id+"_place_station",true);
          sequence->insert(std::move(support));
          arm("PLACE_APPROACH",goal->target,cartesian);
          hand("RELEASE",gripper.openAngle());attachment(false);
          auto restore=std::make_unique<mtc::stages::ModifyPlanningScene>("_SUPPORT_RESTORE");
          restore->setCallback([original_acm](const planning_scene::PlanningScenePtr& state,const mtc::PropertyMap&) {
            state->getAllowedCollisionMatrixNonConst()=original_acm;
          });sequence->insert(std::move(restore));
          auto exits=std::make_unique<mtc::Alternatives>("RETREAT_ALTERNATIVES");
          unsigned i=0;
          for(const auto& target:goal->exit_targets) {
            auto stage=std::make_unique<mtc::stages::MoveTo>("RETREAT:"+std::to_string(i++),cartesian);
            stage->setGroup("arm_left");stage->setIKFrame("astribot_arm_left_tcp_link");stage->setGoal(target);
            stage->setPathConstraints(arm_constraints);
            exits->insert(std::move(stage));
          }
          sequence->insert(std::move(exits));
        }
        auto compact=std::make_unique<mtc::stages::MoveTo>(goal->operation=="PICK"?"TRANSPORT_POSTURE":"STOW",pipeline);
        compact->setGroup("arm_left");
        compact->setPathConstraints(arm_constraints);
        compact->setTimeout(3.);
        const auto* group=task.getRobotModel()->getJointModelGroup("arm_left");
        const std::vector<double> values{0.,0.,-.6,1.,0.,0.,0.};std::map<std::string,double> target;
        if(group->getVariableCount()!=values.size())throw std::runtime_error("ARM_MODEL_MISMATCH");
        for(size_t i=0;i<values.size();++i)target[group->getVariableNames()[i]]=values[i];
        compact->setGoal(target);sequence->insert(std::move(compact));task.add(std::move(sequence));
        auto feedback=std::make_shared<Action::Feedback>();feedback->stage_path=goal->operation;feedback->attempt=attempt;
        feedback->diagnostic="PLAN_COMPLETE_OPERATION";handle->publish_feedback(feedback);
        task.init();if(handle->is_canceling())break;task.plan(12);
        if(handle->is_canceling())break;
        for(const auto& candidate:task.solutions()) {
          std::vector<const mtc::SubTrajectory*> leaves;flatten(*candidate,leaves);std::vector<Segment> segments;bool valid=true;
          std::vector<CachedStage> cached_candidate;
          for(const auto* sub:leaves) {
            if(handle->is_canceling()){valid=false;break;}
            const auto name=sub->creator()->name();
            if(name=="SNAPSHOT" || name.rfind("_CONTACT_",0)==0 || name.rfind("_SUPPORT_",0)==0 ||
               name.rfind("_PREPLACE_",0)==0 || name.rfind("_PREGRASP_",0)==0)continue;
            Segment segment;segment.stage_id=name;
            CachedStage cached_stage;cached_stage.id=name;
            cached_stage.scene=planning_scene::PlanningScene::clone(sub->start()->scene());
            moveit::core::robotStateToRobotStateMsg(sub->start()->scene()->getCurrentState(),segment.expected_start);
            if(sub->trajectory() && sub->trajectory()->getWayPointCount()) {
              robot_trajectory::RobotTrajectory trajectory(*sub->trajectory(),true);
              if(!validateExternalTrajectory(sub->start()->scene(),trajectory,error,
                  velocity_scaling_,acceleration_scaling_)) {
                valid=false;result->reason=name+":"+error;feedback->diagnostic=result->reason;
                feedback->stage_path=name;handle->publish_feedback(feedback);break;
              }
              // Check emitted timed waypoints as well as solver constraints;
              // an IK branch at a hard stop must never reach the executor.
              if (trajectory.getGroupName()=="arm_left") {
                for (size_t i=0;i<trajectory.getWayPointCount() && valid;++i)
                  for (const auto& joint:arm_group->getVariableNames()) {
                    const auto& bounds=task.getRobotModel()->getVariableBounds(joint);
                    if (bounds.position_bounded_ && !astribot_s1_transport_mtc::planningInterval(
                        bounds.min_position_,bounds.max_position_,joint_margin_).contains(
                        trajectory.getWayPoint(i).getVariablePosition(joint))) {
                      valid=false;result->reason=name+":MTC_JOINT_PLANNING_MARGIN:"+joint;break;
                    }
                  }
                if (!valid) {feedback->diagnostic=result->reason;handle->publish_feedback(feedback);break;}
              }
              trajectory.getRobotTrajectoryMsg(segment.trajectory);
              segment.kind=trajectory.getGroupName()=="gripper_left"?"GRIPPER":"ARM";
              cached_stage.trajectory=std::make_shared<robot_trajectory::RobotTrajectory>(trajectory,true);
            } else if(name=="ATTACH_CONFIRM")segment.kind="ATTACH";
            else if(name=="DETACH_CONFIRM")segment.kind="DETACH";
            else {valid=false;result->reason="UNSUPPORTED_EMPTY_STAGE:"+name;break;}
            segments.push_back(segment);
            cached_candidate.push_back(std::move(cached_stage));
          }
          if(valid && !segments.empty()) {
            task.introspection().publishSolution(*candidate);
            result->stages=std::move(segments);result->success=true;result->reason="MTC_SEQUENCE_VALIDATED";
            {std::lock_guard<std::mutex> guard(mutex_);cached_stages_=std::move(cached_candidate);
             cached_context_=goal->context_id;cached_at_=std::chrono::steady_clock::now();
             cached_payload_.object_id=goal->object_id;cached_payload_.operation=goal->operation;
             cached_payload_.input_scene=goal->scene;
             cached_octomap_metadata_=goal->scene.world.octomap;cached_octomap_metadata_.octomap.data.clear();}
            break;
          }
        }
        if(result->success)break;
        if(result->reason.empty()) {std::ostringstream details;task.explainFailure(details);result->reason="MTC_NO_SOLUTION:"+details.str();}
      }
    } catch(const std::exception& e) {result->success=false;result->stages.clear();result->reason=e.what();}
    if(handle->is_canceling() || !result->success) {
      std::lock_guard<std::mutex> guard(mutex_);cached_context_.clear();cached_stages_.clear();cached_payload_={};
    }
    if(handle->is_canceling()){result->success=false;result->stages.clear();result->reason="PLANNING_CANCELED";handle->canceled(result);}
    else if(result->success)handle->succeed(result);
    else {result->stages.clear();if(result->reason.empty())result->reason="PLANNING_BUDGET_EXHAUSTED";handle->abort(result);}
    busy_=false;
  }
public:
  explicit Planner(rclcpp::Node::SharedPtr node):node_(std::move(node)) {
    if(!node_->get_parameter("use_sim_time").as_bool())throw std::runtime_error("SIMULATION_ONLY");
    for(const auto& name:{"max_velocity_scaling","max_acceleration_scaling"})
      if(!node_->has_parameter(name))node_->declare_parameter(name,.1);
    velocity_scaling_=node_->get_parameter("max_velocity_scaling").as_double();
    acceleration_scaling_=node_->get_parameter("max_acceleration_scaling").as_double();
    if(!node_->has_parameter("joint_limit_margin_rad"))node_->declare_parameter("joint_limit_margin_rad",.1);
    joint_margin_=node_->get_parameter("joint_limit_margin_rad").as_double();
    (void)astribot_s1_transport_mtc::planningInterval(-1.,1.,joint_margin_);
    if(!std::isfinite(velocity_scaling_) || !std::isfinite(acceleration_scaling_) ||
       velocity_scaling_<=0 || velocity_scaling_>1 || acceleration_scaling_<=0 || acceleration_scaling_>1)
      throw std::runtime_error("INVALID_MOTION_SCALING");
    parameters_callback_=node_->add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>& values) {
      rcl_interfaces::msg::SetParametersResult result;result.successful=true;
      for(const auto& value:values)
        if(value.get_name()=="joint_limit_margin_rad" || value.get_name()=="max_velocity_scaling" ||
           value.get_name()=="max_acceleration_scaling") {
          result.successful=false;result.reason="PLANNING_PARAMETERS_REQUIRE_RESTART";break;
        }
      return result;
    });
    revalidate_service_=node_->create_service<Revalidate>("/transport/revalidate_manipulation",
      [this](const std::shared_ptr<Revalidate::Request> request,std::shared_ptr<Revalidate::Response> response) {
        revalidate(*request,*response);
      });
    payload_revalidate_service_=node_->create_service<RevalidatePayload>("/transport/revalidate_payload_transition",
      [this](const std::shared_ptr<RevalidatePayload::Request> request,std::shared_ptr<RevalidatePayload::Response> response) {
        response->context_id=request->context_id;response->transaction_id=request->transaction_id;
        bool free=false;
        if(!busy_.compare_exchange_strong(free,true)){response->reason="MTC_REVALIDATION_BUSY";return;}
        struct Release {std::atomic<bool>& busy;~Release(){busy=false;}} release{busy_};
        try {
          std::lock_guard<std::mutex> guard(mutex_);
          auto stages=astribot_s1_transport_mtc::revalidatePayloadTransition(*request,cached_context_,
            cached_payload_,cached_stages_,cached_at_,velocity_scaling_,acceleration_scaling_);
          // Prepare the binding before committing either half. An exception
          // leaves the original cache and its original expiry intact.
          auto binding=cached_payload_;binding.transaction_id=request->transaction_id;
          binding.confirmed_scene=request->scene;
          cached_stages_.swap(stages);std::swap(cached_payload_,binding);
          response->success=true;response->reason="MTC_PAYLOAD_REMAINING_SEQUENCE_REVALIDATED";
        } catch(const std::exception& error){response->success=false;response->reason=error.what();}
      });
    server_=rclcpp_action::create_server<Action>(node_,"/transport/plan_manipulation",
      [this](const rclcpp_action::GoalUUID&,std::shared_ptr<const Action::Goal>) {
        bool free=false;return busy_.compare_exchange_strong(free,true)?rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE:rclcpp_action::GoalResponse::REJECT;
      },[this](const std::shared_ptr<Handle>) {
        std::lock_guard<std::mutex> guard(mutex_);if(active_)active_->preempt();return rclcpp_action::CancelResponse::ACCEPT;
      },[this](const std::shared_ptr<Handle> goal) {
        if(worker_.joinable())worker_.join();worker_=std::thread([this,goal]{run(goal);});
      });
  }
  ~Planner(){ {std::lock_guard<std::mutex> g(mutex_);if(active_)active_->preempt();}if(worker_.joinable())worker_.join();}
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("transport_mtc_planner",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  try {Planner planner(node);rclcpp::executors::MultiThreadedExecutor executor;executor.add_node(node);executor.spin();}
  catch(const std::exception& e){RCLCPP_ERROR(node->get_logger(),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
