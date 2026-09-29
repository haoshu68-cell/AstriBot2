#pragma once
#include "astribot_s1_transport_native/payload_command.hpp"
#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose.pb.h>
#include <functional>
#include <memory>
#include <optional>
namespace astribot::transport {
// Simulation-only, owned by the same executor/resource lease as the arm task.
// The caller provides a registered model and a validated physical-parent/local
// pose (attach) or world pose (detach). No shell or extra controller gateway.
class PayloadClient {
public:
 using Receipt=PayloadCommand::Receipt;
 PayloadClient(std::string model,std::function<int64_t()> ros_clock,
   const ignition::transport::NodeOptions &options={});
 ~PayloadClient();
 PayloadClient(const PayloadClient&)=delete;
 PayloadClient& operator=(const PayloadClient&)=delete;
 void begin(bool attach,ignition::msgs::Pose request,double radius,
   const nlohmann::json &inventory,Receipt receipt,int64_t ros,int64_t steady);
 bool applied(const nlohmann::json &inventory,Receipt receipt,int64_t ros,int64_t steady);
 // Latest ordered model-world observation; querying never sends a command.
 std::optional<nlohmann::json> observation(int64_t ros,int64_t steady);
 // Failure/timeout is not proof of a terminal physical command. The owning
 // executor must quarantine unresolved transactions instead of releasing them.
 bool unresolved() const{return unresolved_;}
 uint32_t id() const{return command_->id();}
private:
 struct Inbox;
 std::string model_;
 std::function<int64_t()> clock_;
 std::shared_ptr<Inbox> inbox_;
 ignition::transport::Node node_;
 std::unique_ptr<PayloadCommand> command_;
 bool attach_=false,unresolved_=false;
 double radius_=0;
 int64_t started_=0;
};
}
