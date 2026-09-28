// Real RequireNavigationEnvelope plugin and DDS publisher; only the downstream
// action and the test node's ROS clock are controlled. No motion is commanded.
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "astribot_navigation_msgs/msg/navigation_envelope_v2.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"

using Message = astribot_navigation_msgs::msg::NavigationEnvelopeV2;
using Status = BT::NodeStatus;
using Wall = std::chrono::steady_clock;
using namespace std::chrono_literals;

struct Probe {int ticks = 0, halts = 0;};

class ControlledChild : public BT::StatefulActionNode
{
public:
  using BT::StatefulActionNode::StatefulActionNode;
  static BT::PortsList providedPorts() {return {};}
  Status onStart() override {return onRunning();}
  Status onRunning() override {
    ++config().blackboard->get<Probe *>("probe")->ticks;
    return Status::RUNNING;
  }
  void onHalted() override {++config().blackboard->get<Probe *>("probe")->halts;}
};

struct Fixture
{
  Probe probe;
  rclcpp::Node::SharedPtr node, source;
  rclcpp::Publisher<Message>::SharedPtr publisher;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr board;
  BT::Tree tree;

  explicit Fixture(const char * library) {
    node = std::make_shared<rclcpp::Node>("envelope_bt_test_guard",
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("navigation_geometry_mode", "fixed_v2"),
        rclcpp::Parameter("use_sim_time", true)}));
    source = std::make_shared<rclcpp::Node>("envelope_bt_test_publisher");
    publisher = source->create_publisher<Message>("/navigation/envelope_v2", 10);
    factory.registerFromPlugin(library);
    factory.registerNodeType<ControlledChild>("ControlledChild");
    board = BT::Blackboard::create();
    board->set("node", node);board->set("probe", &probe);
    tree = factory.createTreeFromText(
      "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
      "<RequireNavigationEnvelope reason='{reason}'><ControlledChild/>"
      "</RequireNavigationEnvelope></BehaviorTree></root>", board);
    if (rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()) != RCL_RET_OK)
      throw std::runtime_error("cannot control fixture ROS clock");
    const auto deadline = Wall::now() + 3s;
    while (publisher->get_subscription_count() != 1) {
      if (Wall::now() >= deadline) throw std::runtime_error("isolated guard subscription not discovered");
      std::this_thread::sleep_for(5ms);
    }
  }
  ~Fixture() {tree.haltTree();}

  Message heartbeat(int stamp_ms, int until_ms) const {
    Message message;
    message.header.stamp = rclcpp::Time(626000000000LL + int64_t(stamp_ms) * 1000000LL);
    message.header.frame_id = "astribot_torso_base";
    message.valid_until = rclcpp::Time(626000000000LL + int64_t(until_ms) * 1000000LL);
    message.limits.stamp = message.header.stamp;
    message.limits.lease_s = .3;
    message.navigation_allowed = message.limits.transport_ready = true;
    message.mode = Message::FIXED_POSTURE;
    message.coordinator_session_id = "fixture_session";
    message.epoch = 7;message.clock_epoch = 1;
    message.installed_geometry_hash = "fixture_geometry";
    message.hold_id = "fixture_hold";message.request_id = "fixture_request";
    message.attachment_revision = "attachment_1";message.model_revision = "model_1";
    message.source_state_sequence = stamp_ms + 1;
    return message;
  }
  void publish(const Message & message) {
    publisher->publish(message);
    if (!publisher->wait_for_all_acked(1s)) throw std::runtime_error("DDS sample not acknowledged");
  }
  Status tick(int now_ms) {
    if (rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),
      626000000000LL + int64_t(now_ms) * 1000000LL) != RCL_RET_OK)
      throw std::runtime_error("fixture clock update failed");
    return tree.tickRoot();
  }
  void bind() {
    publish(heartbeat(0, 500));
    if (tick(200) != Status::RUNNING || probe.ticks != 1)
      throw std::runtime_error("valid initial heartbeat did not bind");
  }
  std::string reason() const {return board->get<std::string>("reason");}
};

int main(int argc, char ** argv)
{
  if (argc != 2) {std::cerr << "usage: envelope_bt_test <BT plugin library>\n";return 2;}
  rclcpp::init(argc, argv);
  int failures = 0;
  const auto report = [&failures](const char * name, bool ok, Status status, const Fixture & f) {
      std::cout << (ok ? "PASS " : "FAIL ") << name << " status=" << BT::toStr(status)
                << " reason=" << f.reason() << " child_ticks=" << f.probe.ticks
                << " child_halts=" << f.probe.halts << std::endl;
      if (!ok) ++failures;
    };
  try {
    {
      Fixture f(argv[1]);
      // front55's last depth-10 positive samples before rejection, retaining
      // exact source/expiry times and sequences from the closed capture. All
      // are acknowledged while IDLE. Tick one can bind the oldest valid frame;
      // tick two must use the current valid heartbeat instead of latching an
      // expired middle frame. This also exercises non-monotonic expiry times.
      const struct {int stamp_ms;int64_t until_ns;uint64_t sequence;} captured[] = {
        {12, 159246556, 7162}, {33, 130441239, 7162}, {59, 106327678, 7162},
        {61, 106327678, 7163}, {80, 277429496, 7163}, {100, 277429496, 7163},
        {105, 252064295, 7163}, {120, 254416228, 7164},
        {124, 349699383, 7164}, {150, 325455136, 7164}};
      for (const auto & sample : captured) {
        auto message = f.heartbeat(sample.stamp_ms, 0);
        message.valid_until = rclcpp::Time(626000000000LL + sample.until_ns);
        message.source_state_sequence = sample.sequence;f.publish(message);
      }
      if (f.tick(150) != Status::RUNNING || f.probe.ticks != 1)
        throw std::runtime_error("idle backlog did not bind on first tick");
      const auto status = f.tick(154);
      report("front55_idle_depth10_latest_valid", status == Status::RUNNING &&
        f.reason().empty() && f.probe.ticks == 2 && f.probe.halts == 0, status, f);
    }
    {
      Fixture f(argv[1]);f.bind();
      auto revoked = f.heartbeat(250, 600);
      revoked.navigation_allowed = false;revoked.reason = "ARM_HOLD_UNCONFIRMED";
      f.publish(revoked);f.publish(f.heartbeat(280, 600));
      const auto first = f.tick(300);
      const auto second = f.tick(310);
      report("revocation_not_erased_by_positive", first == Status::FAILURE &&
        second == Status::FAILURE && f.reason() == "ENVELOPE_REVOKED: ARM_HOLD_UNCONFIRMED" &&
        f.probe.ticks == 1 && f.probe.halts == 1, second, f);
    }
    {
      Fixture f(argv[1]);f.bind();f.publish(f.heartbeat(100, 300));
      const auto status = f.tick(350);
      report("latest_positive_survives_old_deadline", status == Status::RUNNING &&
        f.reason().empty() && f.probe.ticks == 2 && f.probe.halts == 0, status, f);
    }
    {
      Fixture f(argv[1]);f.bind();auto changed = f.heartbeat(250, 600);
      changed.request_id = "different_execution";
      f.publish(changed);f.publish(f.heartbeat(280, 600));
      const auto first = f.tick(300);
      const auto second = f.tick(310);
      report("identity_change_not_erased_by_positive", first == Status::FAILURE &&
        second == Status::FAILURE && f.reason() == "NAVIGATION_ENVELOPE_CHANGED" &&
        f.probe.ticks == 1 && f.probe.halts == 1, second, f);
    }
    {
      Fixture f(argv[1]);f.bind();auto changed = f.heartbeat(250, 600);
      changed.mode = Message::HOLD;
      f.publish(changed);f.publish(f.heartbeat(280, 600));
      const auto first = f.tick(300);
      const auto second = f.tick(310);
      report("mode_hold_not_erased_by_positive", first == Status::FAILURE &&
        second == Status::FAILURE && f.reason() == "ENVELOPE_MODE_MISMATCH" &&
        f.probe.ticks == 1 && f.probe.halts == 1, second, f);
    }
    {
      Fixture f(argv[1]);f.bind();auto old = f.heartbeat(250, 600);
      --old.epoch;old.mode = Message::HOLD;old.navigation_allowed = false;
      f.publish(old);f.publish(f.heartbeat(280, 600));
      const auto first = f.tick(300);
      const auto second = f.tick(310);
      report("lower_epoch_still_ignored", first == Status::RUNNING &&
        second == Status::RUNNING && f.reason().empty() &&
        f.probe.ticks == 3 && f.probe.halts == 0, second, f);
    }
  } catch (const std::exception & error) {
    std::cerr << "FIXTURE_ERROR " << error.what() << std::endl;
    rclcpp::shutdown();return 2;
  }
  rclcpp::shutdown();
  std::cout << "cases=6 failures=" << failures << std::endl;
  return failures ? 1 : 0;
}
