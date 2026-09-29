#include <cassert>
#include <iostream>
#include <string>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_behavior_tree/behavior_tree_engine.hpp>

class ControlledChild : public BT::StatefulActionNode
{
public:
  using BT::StatefulActionNode::StatefulActionNode;
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override {return result();}
  BT::NodeStatus onRunning() override {return result();}
  void onHalted() override {}
  BT::NodeStatus result()
  {
    return config().blackboard->get<BT::NodeStatus>("child_result");
  }
};

int main(int argc, char ** argv)
{
  assert(argc == 2);
  nav2_behavior_tree::BehaviorTreeEngine engine({});
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(argv[1]);
  factory.registerNodeType<ControlledChild>("ControlledChild");
  auto bb = BT::Blackboard::create();
  geometry_msgs::msg::PoseStamped goal;
  bb->set("goal", goal);
  bb->set("child_result", BT::NodeStatus::RUNNING);
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
    "<PolicyExecution session='{session}' goal='{goal}'>"
    "<ControlledChild/></PolicyExecution></BehaviorTree></root>", bb);
  assert(tree.tickRoot() == BT::NodeStatus::RUNNING);
  auto first = bb->get<std::string>("session");
  assert(!first.empty());
  assert(tree.tickRoot() == BT::NodeStatus::RUNNING);
  assert(bb->get<std::string>("session") == first);

  // Use Nav2's actual cancellation entry point, not Tree::haltTree(),
  // which resets root status and masks the lifecycle regression.
  engine.haltAllActions(tree.rootNode());
  assert(tree.tickRoot() == BT::NodeStatus::RUNNING);
  auto restarted = bb->get<std::string>("session");
  if (restarted == first) {
    std::cerr << "same goal reused a canceled execution session\n";
    return 1;
  }
  assert(tree.tickRoot() == BT::NodeStatus::RUNNING);
  assert(bb->get<std::string>("session") == restarted);

  goal.pose.position.x = 1.;
  bb->set("goal", goal);
  assert(tree.tickRoot() == BT::NodeStatus::RUNNING);
  assert(bb->get<std::string>("session") != restarted);

  for (auto terminal : {BT::NodeStatus::SUCCESS, BT::NodeStatus::FAILURE}) {
    bb->set("child_result", terminal);
    assert(tree.tickRoot() == terminal);
    const auto completed = bb->get<std::string>("session");
    // Completion must invalidate identity even without an intervening halt.
    bb->set("child_result", BT::NodeStatus::RUNNING);
    assert(tree.tickRoot() == BT::NodeStatus::RUNNING);
    assert(bb->get<std::string>("session") != completed);
    engine.haltAllActions(tree.rootNode());
  }
  std::cout << "stable active session; cancel, changed goal and terminal restart get new identities\n";
}
