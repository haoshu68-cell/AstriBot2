#!/usr/bin/env bash
# Optional simulation dependencies; no sudo, system installs, or live-stack changes.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
deps_dir="${1:-$repo_dir/ws_robot/deps/hunav_sim}"
mkdir -p "$deps_dir"
deps_dir="$(cd -- "$deps_dir" && pwd)"
python3 "$repo_dir/tools/social_navigation/fetch_dependencies.py" --destination "$deps_dir"
set +u
source /opt/ros/humble/setup.bash
set -u
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
mkdir -p "$deps_dir/install/include/lightsfm"
cp "$deps_dir"/source/lightsfm/include/* "$deps_dir/install/include/lightsfm/"

# Keep the upstream bytes in source/ immutable; record compatibility edits separately.
python3 - "$deps_dir" <<'PY'
import difflib, pathlib, shutil, sys
root = pathlib.Path(sys.argv[1])
patches = []
for name in ('hunav_sim', 'hunav_gazebo_fortress_wrapper', 'people'):
    shutil.copytree(root / 'source' / name, root / 'build_source' / name, dirs_exist_ok=True)

def edit(relative, old, new):
    path = root / 'build_source' / relative
    before = path.read_text()
    if old not in before:
        raise RuntimeError('Pinned patch no longer applies: ' + relative)
    after = before.replace(old, new)
    path.write_text(after)
    patches.extend(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
                                      fromfile=relative, tofile=relative))

manager = 'hunav_sim/hunav_agent_manager/'
# This declared dependency is unused. Linking Nav2 into the human simulator mixes BT 3/4.
for path in (root / 'build_source' / manager / 'src').glob('*.cpp'):
    if '#include "nav2_behavior_tree/' in path.read_text():
        raise RuntimeError('Review new Nav2 dependency before building HuNav')
edit(manager + 'CMakeLists.txt', 'find_package(nav2_behavior_tree REQUIRED)', '')
edit(manager + 'CMakeLists.txt', 'visualization_msgs nav2_behavior_tree tf2', 'visualization_msgs tf2')
edit(manager + 'CMakeLists.txt', 'tf2 tf2_ros behaviortree_cpp)', 'tf2 tf2_ros tf2_geometry_msgs behaviortree_cpp)')
edit(manager + 'package.xml', '<depend>nav2_behavior_tree</depend>', '')
edit(manager + 'CMakeLists.txt', '  /usr/local/include #to find lightsfm just in case',
     '  "' + str(root / 'install/include') + '"')
edit(manager + 'src/bt_node.cpp', 'std::string src_dir = share_to_src_path(share_dir);\n        bt_dir_base_ = src_dir + "/behavior_trees";',
     'bt_dir_base_ = this->declare_parameter<std::string>("behavior_tree_directory", share_dir + "/behavior_trees");')
edit(manager + 'src/bt_node.cpp', '    BT::NodeStatus status;', '    BT::NodeStatus status = BT::NodeStatus::SUCCESS;')
edit(manager + 'src/bt_node.cpp', '    if (_agent.id == 1)', '    if (_agent.id == 1 && this->get_parameter("enable_groot").as_bool())')
edit(manager + 'src/bt_node.cpp', '  initialized_ = false;',
     '  initialized_ = false;\n  this->declare_parameter<bool>("enable_groot", false);')
edit(manager + 'src/agent_manager.cpp', '  a.type = agents_[id].type;',
     '  a.type = agents_[id].type;\n'
     '  a.radius = agents_[id].sfmAgent.radius;\n'
     '  a.group_id = agents_[id].sfmAgent.groupId;\n'
     '  a.desired_velocity = agents_[id].sfmAgent.desiredVelocity;\n'
     '  a.cyclic_goals = agents_[id].sfmAgent.cyclicGoals;\n'
     '  if (!agents_[id].sfmAgent.goals.empty()) a.goal_radius = agents_[id].sfmAgent.goals.front().radius;')
edit(manager + 'src/agent_manager.cpp', 'void AgentManager::init()\n{',
     'void AgentManager::init()\n{\n  agents_.clear(); override_goals_data_.clear(); orig_desired_vels_.clear(); last_interaction_time_.clear();')
edit(manager + 'include/hunav_agent_manager/bt_functions.hpp', '  void init();',
     '  void init();\n  void resetSimulation() { agent_manager_.init(); }')
edit(manager + 'src/bt_node.cpp', '    btfunc_.updateAllAgents(ro, ag);',
     '    if (initialized_ && rclcpp::Time(ag->header.stamp) < prev_time_) {\n'
     '      for (auto &entry : trees_) entry.second.haltTree();\n'
     '      publisher_.reset(); trees_.clear(); btfunc_.resetSimulation(); initialized_ = false;\n'
     '    }\n    btfunc_.updateAllAgents(ro, ag);')
wrapper = 'hunav_gazebo_fortress_wrapper/'
edit(wrapper + 'CMakeLists.txt', '  LIBRARY DESTINATION ${ignition-gazebo6_PLUGIN_PATH}', '')
plugin = wrapper + 'src/HuNavSystemPlugin_fortress.cpp'
edit(plugin, 'this->rosnode_ = std::make_shared<rclcpp::Node>(nodename.c_str());',
     'this->rosnode_ = std::make_shared<rclcpp::Node>(nodename.c_str(), rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));')
edit(plugin, 'pose.Rot().Z()', 'pose.Rot().Yaw()')
edit(plugin, '        newPose.Pos().Y(0);',
     '        newPose.Pos().Y(0);\n        newPose.Pos().Z(0);\n        newPose.Rot() = gz::math::Quaterniond::Identity;')
edit(plugin, 'auto npose = worldPose(agentEntity, _ecm);', 'auto npose = curr_pose;')
edit(plugin, 'curr_pose.Pos().Z(0.35);', 'curr_pose.Pos().Z(sdf_->Get<double>("actor_ground_offset", 1.06178).first);')
edit(plugin, 'actorPose.Pos().Z(0.8);', 'actorPose.Pos().Z(sdf_->Get<double>("actor_ground_offset", 1.06178).first);')
# Rendering follows the physical heading with a bounded turn rate, never a fixed 1% chase.
edit(plugin, 'yaw = normalizeAngle(currAngle + (diff * 0.01));  // 0.01, 0.005',
     'yaw = normalizeAngle(currAngle + std::max(-2.0 * update_rate_secs_, std::min(2.0 * update_rate_secs_, diff)));')
edit(plugin, 'if (std::fabs(diff) > IGN_DTOR(10)) //25 degrees to rads',
     'if (std::fabs(diff) > 2.0 * update_rate_secs_)')
edit(plugin, 'double animationFactor = 5.0; //0.005; //Noé',
     'double animationFactor = 1.0;')
edit(plugin, '    RCLCPP_ERROR(this->rosnode_->get_logger(), "Error initializing robot. We will try again...");',
     '    RCLCPP_WARN_THROTTLE(this->rosnode_->get_logger(), *this->rosnode_->get_clock(), 2000, "Waiting for robot spawn");')
edit(plugin, '    initializeAgents(_ecm);\n  }',
     '    initializeAgents(_ecm);\n    if (!agentsInitialized_) return;\n  }')
edit(plugin, '  lastUpdate_ = _info.simTime;\n\n  // Convert',
     '  if (dt < 0) { agentsInitialized_ = false; lastUpdate_ = _info.simTime; return; }\n'
     '  if (dt < update_rate_secs_) return;\n'
     '  lastUpdate_ = _info.simTime;\n\n  // Convert')
edit(plugin, '  getObstacles(_ecm);', '  if (sdf_->Get<bool>("use_gazebo_obs", false).first) getObstacles(_ecm);')
edit(plugin, '  agents.header.stamp = this->rosnode_->get_clock()->now(); //ros_time;', '  agents.header.stamp = ros_time;')
# Service loss must not indefinitely block Gazebo's physics loop.
path = root / 'build_source' / plugin
text = path.read_text()
for client, wait, ending in [('rosSrvGetAgentsClient_', '5s', '  RCLCPP_INFO(rosnode_->get_logger(), "Service /get_agents is available.'),
                             ('rosSrvClient_', '1s', '  hunav_msgs::msg::Agents agents;')]:
    start = text.index('  while (!' + client + '->wait_for_service(' + wait + '))')
    end = text.index(ending, start)
    old = text[start:end]
    edit(plugin, old, '  if (!' + client + '->service_is_ready()) return;\n\n')
    text = path.read_text()
edit(plugin, '  if (rclcpp::spin_until_future_complete(this->rosnode_, result) ==\n    rclcpp::FutureReturnCode::SUCCESS)',
     '  if (rclcpp::spin_until_future_complete(this->rosnode_, result, 200ms) != rclcpp::FutureReturnCode::SUCCESS) {\n'
     '    rosSrvGetAgentsClient_->remove_pending_request(result); return;\n  }\n  if (result.valid())')
edit(plugin, '    RCLCPP_ERROR(rosnode_->get_logger(), "Failed to call service /compute_agents");',
     '    rosSrvClient_->remove_pending_request(result);\n    RCLCPP_ERROR(rosnode_->get_logger(), "Failed to call service /compute_agents");')
edit(wrapper + 'include/hunav_gazebo_fortress_wrapper/HuNavSystemPlugin_fortress.h',
     '  rclcpp::Node::SharedPtr rosnode_;',
     '  rclcpp::Node::SharedPtr rosnode_;\n  rclcpp::Publisher<hunav_msgs::msg::Agents>::SharedPtr actor_states_;')
edit(plugin, '  worldEntity_ = _entity;',
     '  actor_states_ = rosnode_->create_publisher<hunav_msgs::msg::Agents>("/simulation/hunav_actor_states", 10);\n  worldEntity_ = _entity;')
edit(plugin, '    updateGazeboPedestrians(_ecm, _info, updated_agents);',
     '    updateGazeboPedestrians(_ecm, _info, updated_agents);\n'
     '    for (auto &a : updated_agents.agents) {\n'
     '      auto e = _ecm.EntityByComponents(gz::sim::components::Name(a.name));\n'
     '      auto pose = _ecm.Component<gz::sim::components::TrajectoryPose>(e);\n'
     '      if (!pose) continue;\n'
     '      double previous_yaw = pedestrians_.at(e).yaw;\n'
     '      a.yaw = pose->Data().Rot().Yaw();\n'
     '      tf2::Quaternion q; q.setRPY(0, 0, a.yaw); a.position.orientation = tf2::toMsg(q);\n'
     '      a.angular_vel = normalizeAngle(a.yaw - previous_yaw) / dt; a.velocity.angular.z = a.angular_vel;\n'
     '    }\n'
     '    updated_agents.header.stamp = ros_time; updated_agents.header.frame_id = globalFrame_;\n'
     '    actor_states_->publish(updated_agents);')
# Explicit, simulation-only episode control; no robot goal or velocity publisher.
edit(wrapper + 'include/hunav_gazebo_fortress_wrapper/HuNavSystemPlugin_fortress.h',
     '  rclcpp::Publisher<hunav_msgs::msg::Agents>::SharedPtr actor_states_;',
     '  rclcpp::Publisher<hunav_msgs::msg::Agents>::SharedPtr actor_states_;\n'
     '  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr episode_command_;\n'
     '  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr episode_state_;')
edit(plugin, '  reset_ = false;\n  //rostime_',
     '  if (waitForGoal_) {\n'
     '    episode_state_ = rosnode_->create_publisher<std_msgs::msg::String>("/social_sim/episode_state", rclcpp::QoS(1).transient_local());\n'
     '    episode_command_ = rosnode_->create_subscription<std_msgs::msg::String>("/social_sim/episode", 10,\n'
     '      [this](std_msgs::msg::String::ConstSharedPtr msg) {\n'
     '        if (msg->data != "start" && msg->data != "pause") return;\n'
     '        goalReceived_ = msg->data == "start"; episode_state_->publish(*msg);\n'
     '      });\n'
     '    std_msgs::msg::String state; state.data = "pause"; episode_state_->publish(state);\n'
     '  }\n  reset_ = false;\n  //rostime_')
edit(plugin, '  agents.agents = vector;',
     '  agents.agents = vector;\n'
     '  rclcpp::spin_some(rosnode_);\n'
     '  if (waitForGoal_ && !goalReceived_) {\n'
     '    for (auto &a : agents.agents) { a.velocity = geometry_msgs::msg::Twist(); a.linear_vel = 0.; a.angular_vel = 0.; }\n'
     '    if (rosSrvResetClient_->service_is_ready()) {\n'
     '      auto reset = std::make_shared<hunav_msgs::srv::ResetAgents::Request>();\n'
     '      reset->current_agents = agents; reset->robot = robotAgent_;\n'
     '      auto result = rosSrvResetClient_->async_send_request(reset);\n'
     '      if (rclcpp::spin_until_future_complete(rosnode_, result, 150ms) != rclcpp::FutureReturnCode::SUCCESS)\n'
     '        rosSrvResetClient_->remove_pending_request(result);\n'
     '    }\n'
     '    actor_states_->publish(agents); return;\n'
     '  }')
edit(plugin, 'if (dt < 0) { agentsInitialized_ = false;',
     'if (dt < 0) { goalReceived_ = !waitForGoal_; agentsInitialized_ = false;')
edit(manager + 'src/bt_node.cpp', '    response->ok = true;',
     '    prev_time_ = rclcpp::Time(ag->header.stamp);\n    response->ok = true;')

for name in ('maps',):
    (root / 'build_source' / wrapper / name).mkdir(exist_ok=True)
(root / 'compatibility.patch').write_text(''.join(patches))
PY

colcon --log-base "$deps_dir/log" build --base-paths \
  "$deps_dir/source/behaviortree_cpp" "$deps_dir/build_source" \
  --build-base "$deps_dir/build" --install-base "$deps_dir/install" --merge-install \
  --executor sequential --packages-select behaviortree_cpp people_msgs hunav_msgs \
  hunav_agent_manager hunav_gazebo_fortress_wrapper \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DBTCPP_UNIT_TESTS=OFF -DBTCPP_EXAMPLES=OFF -DBTCPP_BUILD_TOOLS=OFF
printf 'HuNav simulation overlay: %s/install/local_setup.bash\n' "$deps_dir"
