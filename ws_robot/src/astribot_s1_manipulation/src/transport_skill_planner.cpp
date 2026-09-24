// Plans only. The task owner executes/cancels trajectories and confirms measured state.
#include <thread>
#include <cmath>
#include <astribot_transport_msgs/srv/plan_skill.hpp>
#include "astribot_s1_manipulation/dual_arm_planner.hpp"
#include "astribot_s1_manipulation/gripper_commander.hpp"
#include "astribot_s1_manipulation/external_trajectory_validator.hpp"
#include <moveit_msgs/srv/get_planning_scene.hpp>

using namespace astribot_s1_manipulation;
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("transport_skill_planner",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&] {executor.spin();});
  int code = 0;
  try {
    if (!node->get_parameter("use_sim_time").as_bool()) {
      throw std::runtime_error("SIMULATION_ONLY");
    }
    DualArmPlanner planner(node);
    DualArmPlannerParams params;
    params.planner_id = "RRTConnectConfig";
    params.planning_attempts = 1;
    params.closed_chain.leader_tcp_link = "astribot_arm_left_tcp_link";
    params.closed_chain.follower_tcp_link = "astribot_arm_right_tcp_link";
    params.time_optimizer.optimized_velocity_scaling = 0.3;
    params.time_optimizer.optimized_acceleration_scaling = 0.3;
    params.time_optimizer.enable_optimization = false;
    std::string error;
    if (!planner.initialize(params, error)) {throw std::runtime_error(error);}
    GripperCommander gripper(node);
    GripperConfig gc;
    gc.group_name = "gripper_left";
    gc.action_name = "/gripper_left_controller/follow_joint_trajectory";
    gc.tcp_link = "astribot_arm_left_tcp_link";
    gc.left_pad_link = "astribot_gripper_left_Link_L11";
    gc.right_pad_link = "astribot_gripper_left_Link_R11";
    if (gripper.configureForPlanning(planner.getRobotModel(), gc, error) != PlanErrorCode::kSuccess) {
      throw std::runtime_error(error);
    }
    // A separate mutually exclusive callback group serializes planner access while
    // the executor continues servicing MoveIt action responses and joint states.
    auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto scene_client=node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    auto service = node->create_service<astribot_transport_msgs::srv::PlanSkill>(
      "/transport/plan_skill",
      [&](const std::shared_ptr<astribot_transport_msgs::srv::PlanSkill::Request> req,
          std::shared_ptr<astribot_transport_msgs::srv::PlanSkill::Response> res) {
        try {
          if(req->operation=="joints" && req->group=="head") {
            const auto* head=planner.getRobotModel()->getJointModelGroup("head");
            if(req->joint_target.size()!=head->getVariableCount())throw std::runtime_error("HEAD_TARGET_SIZE");
            for(double value:req->joint_target)if(!std::isfinite(value))throw std::runtime_error("HEAD_TARGET_INVALID");
            moveit::core::RobotStatePtr current;
            if(!planner.getCurrentState(current,error))throw std::runtime_error(error);
            auto request=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();request->components.components=1023;
            if(!scene_client->wait_for_service(std::chrono::seconds(2)))throw std::runtime_error("SCENE_UNAVAILABLE");
            auto pending=scene_client->async_send_request(request);
            if(pending.wait_for(std::chrono::seconds(3))!=std::future_status::ready) {
              scene_client->remove_pending_request(pending);throw std::runtime_error("SCENE_TIMEOUT");
            }
            auto scene=std::make_shared<planning_scene::PlanningScene>(planner.getRobotModel());
            scene->setPlanningSceneMsg(pending.get()->scene);scene->setCurrentState(*current);
            moveit::core::RobotState target(*current);target.setJointGroupPositions(head,req->joint_target);target.update();
            if(!target.satisfiesBounds(head))throw std::runtime_error("HEAD_TARGET_OUT_OF_BOUNDS");
            double travel=0.;
            for(auto index:head->getVariableIndexList())
              travel=std::max(travel,std::abs(current->getVariablePositions()[index]-target.getVariablePositions()[index]));
            if(travel<1e-4) {res->success=true;res->reason="HEAD_ALREADY_AT_TARGET";return;}
            robot_trajectory::RobotTrajectory trajectory(planner.getRobotModel(),"head");
            const int steps=std::max(1,int(std::ceil(travel/.02)));
            for(int i=0;i<=steps;++i) {moveit::core::RobotState sample(*current);current->interpolate(target,double(i)/steps,sample);sample.update();trajectory.addSuffixWayPoint(sample,0.);}
            res->success=validateExternalTrajectory(scene,trajectory,res->reason);
            if(res->success){trajectory.getRobotTrajectoryMsg(res->trajectory);res->reason="HEAD_PATH_VALIDATED";}
            return;
          }
          if (req->operation == "open" || req->operation == "close") {
            double angle = gripper.openAngle();
            if (req->operation == "close") {
              if (!std::isfinite(req->grasp_width_m) || req->grasp_width_m <= 0 ||
                  gripper.graspAngleForWidth(req->grasp_width_m, angle, res->reason) !=
                  PlanErrorCode::kSuccess) {return;}
            }
            auto & t = res->trajectory.joint_trajectory;
            t.joint_names = {gripper.jointName()};
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = {angle}; point.time_from_start.sec = 2;
            t.points.push_back(point);
            res->success = true; res->reason = "GRIPPER_GEOMETRY_VALIDATED";
            return;
          }
          if ((req->group != "arm_left" && req->group != "arm_right") ||
              (req->operation != "named" && req->operation != "pose")) {
            res->reason = "UNSUPPORTED_SKILL"; return;
          }
          SingleArmPlanRequest request;
          request.group = req->group;
          request.named_target = req->operation == "named" ? req->named_target : "";
          if (request.named_target == "transport_compact") {
            // Keep the elbow bent while removing the ready pose's shoulder
            // abduction. This target still passes the full planner validation.
            request.named_target.clear();
            request.joint_target = {0., 0., req->group == "arm_left" ? -0.6 : 0.6,
                                    1., 0., 0., 0.};
          }
          request.use_pose_target = req->operation == "pose";
          request.pose_target = req->target;
          request.tcp_link = "astribot_" + req->group + "_tcp_link";
          auto plan = planner.planSingleArm(request);
          res->success = plan.succeeded() || plan.noActionNeeded();
          res->reason = toString(plan.code);
          if (!plan.message.empty()) {res->reason += ": " + plan.message;}
          res->trajectory = plan.trajectory;
        } catch (const std::exception & e) {res->reason = e.what();}
      }, rmw_qos_profile_services_default, group);
    RCLCPP_INFO(node->get_logger(), "Validated transport skill planner ready");
    while (rclcpp::ok()) {std::this_thread::sleep_for(std::chrono::milliseconds(100));}
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what()); code = 1;
  }
  executor.cancel(); spinner.join(); rclcpp::shutdown(); return code;
}
