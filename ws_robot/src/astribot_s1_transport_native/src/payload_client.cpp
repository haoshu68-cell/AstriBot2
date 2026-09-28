#include "astribot_s1_transport_native/payload_client.hpp"
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <optional>
#include <stdexcept>
namespace astribot::transport {
namespace {
int64_t wall(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
void require(bool okay,const char *reason){if(!okay)throw std::runtime_error(reason);}
}
struct PayloadClient::Inbox {
 std::mutex mutex;
 std::optional<nlohmann::json> state;
 Receipt received{};
 std::string failure;
 uint64_t generation=0;
};
PayloadClient::PayloadClient(std::string model,std::function<int64_t()> clock,
 const ignition::transport::NodeOptions &options)
 :model_(std::move(model)),clock_(std::move(clock)),inbox_(std::make_shared<Inbox>()),node_(options) {
 require(!model_.empty()&&model_.find('/')==std::string::npos,"PAYLOAD_MODEL_NAME_INVALID");
 const std::weak_ptr<Inbox> weak=inbox_;
 std::function<void(const ignition::msgs::StringMsg&)> callback=[weak,clock=clock_](const auto &message) {
  auto inbox=weak.lock();if(!inbox)return;
  const Receipt received{clock(),wall()};std::lock_guard<std::mutex> lock(inbox->mutex);
  try {
   const auto value=nlohmann::json::parse(message.data());
   const auto &stamp=value.at("stamp_ns");
   if(!stamp.is_number_integer()||stamp.template get<int64_t>()<=0)throw std::runtime_error("PAYLOAD_STATE_STAMP_INVALID");
   if(inbox->state) {
    const auto previous=inbox->state->at("stamp_ns").template get<int64_t>();
    if(stamp.template get<int64_t>()<previous)return;
    if(stamp.template get<int64_t>()==previous) {
     require(value==*inbox->state,"PAYLOAD_STATE_CAPTURE_CONFLICT");
     return; // Keep the first receipt even before the executor's first poll.
    }
   }
   const auto error=value.at("error").template get<std::string>();
   if(!error.empty()){inbox->failure="PAYLOAD_PHYSICAL_ERROR:"+error;return;}
   inbox->state=value;inbox->received=received;
  }catch(const std::exception &error){inbox->failure=std::string("PAYLOAD_STATE_INVALID:")+error.what();}
 };
 require(node_.Subscribe("/model/"+model_+"/kinematic_attachment/state",callback),"PAYLOAD_STATE_SUBSCRIPTION_FAILED");
}
PayloadClient::~PayloadClient()=default;
void PayloadClient::begin(bool attach,ignition::msgs::Pose request,double radius,
 const nlohmann::json &inventory,Receipt receipt,int64_t ros,int64_t steady) {
 require(!unresolved_,"PAYLOAD_PREVIOUS_COMMAND_UNRESOLVED");
 const auto &p=request.position();const auto &q=request.orientation();
 require(std::isfinite(p.x())&&std::isfinite(p.y())&&std::isfinite(p.z())&&
   std::isfinite(q.x())&&std::isfinite(q.y())&&std::isfinite(q.z())&&std::isfinite(q.w())&&
   std::abs(q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w()-1.)<=.001&&
   std::isfinite(radius)&&radius>0&&attach==!request.name().empty(),"PAYLOAD_REQUEST_INVALID");
 command_=std::make_unique<PayloadCommand>(model_,attach,inventory,receipt,ros,steady);
 request.set_id(command_->id());attach_=attach;radius_=radius;started_=steady;
 uint64_t generation;
 {std::lock_guard<std::mutex> lock(inbox_->mutex);generation=++inbox_->generation;inbox_->failure.clear();inbox_->state.reset();}
 const std::weak_ptr<Inbox> weak=inbox_;
 std::function<void(const ignition::msgs::Boolean&,bool)> callback=[weak,generation](const auto &reply,bool okay) {
  auto inbox=weak.lock();if(!inbox)return;std::lock_guard<std::mutex> lock(inbox->mutex);
  if(generation!=inbox->generation)return;
  if(!okay||!reply.data())inbox->failure=okay?"PAYLOAD_COMMAND_REJECTED":"PAYLOAD_COMMAND_ACK_UNCONFIRMED";
  // A successful queue ACK deliberately changes no applied/terminal state.
 };
 unresolved_=true;
 require(node_.Request("/model/"+model_+"/kinematic_attachment/command",request,callback),"PAYLOAD_COMMAND_SEND_UNCONFIRMED");
}
std::optional<nlohmann::json> PayloadClient::observation(int64_t ros,int64_t steady) {
 (void)ros;(void)steady;
 std::optional<nlohmann::json> state;
 {std::lock_guard<std::mutex> lock(inbox_->mutex);
  if(!inbox_->failure.empty())throw std::runtime_error(inbox_->failure);
  state=inbox_->state;
 }
 if(!state)return {};
 try {
  const auto &counter=state->at("command_id");
  require(counter.is_number_integer()&&counter.get<int64_t>()>=0&&counter.get<uint64_t>()<=UINT32_MAX,"PAYLOAD_STATE_COUNTER_INVALID");
  const auto pose=state->at("actual_world_xyzw").get<std::vector<double>>();
  require(pose.size()==7,"PAYLOAD_WORLD_OBJECT_POSE_INVALID");
  for(double value:pose)require(std::isfinite(value),"PAYLOAD_WORLD_OBJECT_POSE_INVALID");
  require(std::abs(pose[3]*pose[3]+pose[4]*pose[4]+pose[5]*pose[5]+pose[6]*pose[6]-1.)<1e-6,"PAYLOAD_WORLD_OBJECT_POSE_INVALID");
  return state;
 }catch(const nlohmann::json::exception &error){throw std::runtime_error(std::string("PAYLOAD_STATE_INVALID:")+error.what());}
}
bool PayloadClient::applied(const nlohmann::json &inventory,Receipt receipt,int64_t ros,int64_t steady) {
 require(bool(command_),"PAYLOAD_COMMAND_NOT_STARTED");
 // Queue ACK remains insufficient, and the original 2 s application deadline
 // is independent of the ongoing physical observation lifetime.
 require(steady>=started_&&steady-started_<2000000000,"PAYLOAD_APPLICATION_TIMEOUT");
 auto state=observation(ros,steady);
 const bool complete=command_->applied(inventory,receipt,ros,steady);
 if(!state)return false;
 try {
  const auto &counter=state->at("command_id");
  require(counter.get<uint32_t>()<=command_->id(),"FOREIGN_PAYLOAD_COMMAND");
  if(counter.get<uint32_t>()<command_->id())return false;
  require(state->at("attached").get<bool>()==attach_,"PAYLOAD_PHYSICAL_STATE_CONFLICT");
  const double position=state->at("position_error_m").get<double>(),rotation=state->at("rotation_error_rad").get<double>();
  require(std::isfinite(position)&&std::isfinite(rotation)&&position>=0&&rotation>=0,"PAYLOAD_PHYSICAL_ERROR_INVALID");
  require(position+radius_*rotation<=.006,"PAYLOAD_PHYSICAL_TRACKING_ERROR");
  if(!complete)return false;
  const auto completed=wall();
  require(completed>=started_&&completed-started_<2000000000,"PAYLOAD_APPLICATION_TIMEOUT");
  unresolved_=false;return true;
 }catch(const nlohmann::json::exception &error){throw std::runtime_error(std::string("PAYLOAD_STATE_INVALID:")+error.what());}
}
}
