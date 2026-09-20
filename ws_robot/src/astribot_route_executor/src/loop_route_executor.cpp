#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <astribot_operator_msgs/srv/start_loop_route.hpp>
#include <astribot_operator_msgs/srv/cancel_loop_route.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <cmath>
#include <chrono>
#include <random>
#include <sstream>
#include <unordered_set>
#include <csignal>
#include <thread>

namespace astribot_route_executor {
using Start = astribot_operator_msgs::srv::StartLoopRoute;
using Cancel = astribot_operator_msgs::srv::CancelLoopRoute;
using Nav = nav2_msgs::action::NavigateToPose;
using Handle = rclcpp_action::ClientGoalHandle<Nav>;
using Clock = std::chrono::steady_clock;
using Json = nlohmann::json;
class LoopRouteExecutor : public rclcpp::Node {
  rclcpp_action::Client<Nav>::SharedPtr nav_;
  Handle::SharedPtr handle_;
  rclcpp::Service<Start>::SharedPtr start_;
  rclcpp::Service<Cancel>::SharedPtr cancel_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string boot_, route_id_, request_id_, state_{"IDLE"}, reason_, stop_reason_;
  Start::Request route_;
  std::unordered_set<std::string> seen_requests_;
  Json points_=Json::array();
  bool shutting_down_{false}, active_{false}, outstanding_{false}, stopping_{false}, user_cancel_{false};
  size_t index_{0}; uint64_t cycles_{0}, leg_{0};
  double goal_timeout_, server_timeout_, cancel_timeout_;
  Clock::time_point deadline_, cancel_deadline_, heartbeat_{};
  int64_t max_points_;
  void publish() {
    std_msgs::msg::String msg;
    msg.data = Json({{"schema_version",1},{"boot_id",boot_},{"route_id",route_id_},
      {"state",state_},{"reason",reason_},{"active",active_},{"index",index_},
      {"completed_cycles",cycles_},{"dwell_sec",route_.dwell_sec},{"waypoints",points_},
      {"outstanding_goal",outstanding_}}).dump();
    status_->publish(msg);
  }
  void state(const std::string & value, const std::string & reason) {
    state_=value; reason_=reason;
    RCLCPP_INFO(get_logger(), "route=%s state=%s point=%zu cycles=%lu reason=%s",
      route_id_.c_str(), value.c_str(), index_+1, static_cast<unsigned long>(cycles_), reason.c_str());
    publish();
  }
  void finish(const std::string & value, const std::string & reason) {
    active_=false; outstanding_=false; handle_.reset(); state(value,reason);
  }
  void stoppedResult() {finish(user_cancel_ ? "CANCELED" : "FAILED",stop_reason_);}
  void cancelOwned() {
    if(!handle_) return;
    try {nav_->async_cancel_goal(handle_);}
    catch(const std::exception & e) {state("CANCEL_UNCONFIRMED",stop_reason_+"; cancel RPC failed: "+e.what());}
  }
  void stop(const std::string & reason, bool user) {
    if (!active_) return;
    if (!stopping_) {
      stopping_=true; user_cancel_=user; stop_reason_=reason;
      cancel_deadline_=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(cancel_timeout_));
      state("CANCELING",reason);
      cancelOwned();
    }
    else if(state_=="CANCEL_UNCONFIRMED" && handle_) {
      cancel_deadline_=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(cancel_timeout_));
      state("CANCELING",stop_reason_);cancelOwned();
    }
    if (!outstanding_) stoppedResult();
  }
  void dispatch() {
    outstanding_=true; ++leg_;
    const auto leg=leg_;
    Nav::Goal goal; goal.pose=route_.waypoints[index_]; goal.pose.header.stamp=builtin_interfaces::msg::Time();
    state("DISPATCHING","Waiting for goal acceptance");
    deadline_=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(goal_timeout_));
    rclcpp_action::Client<Nav>::SendGoalOptions options;
    options.goal_response_callback=[this,leg](Handle::SharedPtr handle) {
      if (leg!=leg_) {if(handle) nav_->async_cancel_goal(handle); return;}
      if (!handle) {
        outstanding_=false;
        if(stopping_) stoppedResult(); else finish("FAILED","Navigation rejected route waypoint");
        return;
      }
      handle_=handle;
      if(stopping_) cancelOwned();
      else state("NAVIGATING","Executing through route priority");
    };
    options.result_callback=[this,leg](const Handle::WrappedResult & result) {
      if(leg!=leg_ || !outstanding_) return;
      outstanding_=false; handle_.reset();
      if(stopping_) {stoppedResult(); return;}
      if(result.code!=rclcpp_action::ResultCode::SUCCEEDED) {
        finish("FAILED","Navigation failed/canceled/preempted; result_code="+std::to_string(static_cast<int>(result.code))); return;
      }
      // Only a successful terminal result advances the sequence. No local arrival approximation.
      deadline_=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(route_.dwell_sec));
      state("DWELL","Waypoint succeeded; waiting before next point");
    };
    try {nav_->async_send_goal(goal,options);}
    catch(const std::exception & e) {
      // Dispatch outcome is unknown; retain ownership and never send another goal.
      stop(std::string("Dispatch exception; outcome unknown: ")+e.what(),false);
    }
  }
  void tick() {
    if(Clock::now()>=heartbeat_) {publish(); heartbeat_=Clock::now()+std::chrono::seconds(1);}
    if(!active_) return;
    if(stopping_) {
      if(outstanding_ && Clock::now()>cancel_deadline_ && state_!="CANCEL_UNCONFIRMED")
        state("CANCEL_UNCONFIRMED",stop_reason_+"; terminal result not received; new routes blocked");
      return;
    }
    if(state_=="WAIT_SERVER") {
      if(nav_->action_server_is_ready()) dispatch();
      else if(Clock::now()>deadline_) finish("FAILED","Route navigation action unavailable");
    } else if(state_=="DWELL" && Clock::now()>=deadline_) {
      if(++index_==route_.waypoints.size()) {index_=0; ++cycles_;}
      deadline_=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(server_timeout_));
      state("WAIT_SERVER","Next waypoint");
    } else if((state_=="DISPATCHING" || state_=="NAVIGATING") && Clock::now()>deadline_)
      stop("Waypoint deadline exceeded",false);
  }
public:
  explicit LoopRouteExecutor(const rclcpp::NodeOptions & options=rclcpp::NodeOptions()) : Node("loop_route_executor",options) {
    goal_timeout_=declare_parameter("goal_timeout_sec",300.0);
    server_timeout_=declare_parameter("server_timeout_sec",10.0);
    cancel_timeout_=declare_parameter("cancel_timeout_sec",10.0);
    max_points_=declare_parameter("max_points",int64_t(200));
    for(double v:{goal_timeout_,server_timeout_,cancel_timeout_})
      if(!std::isfinite(v)||v<=0) throw std::invalid_argument("Timeouts must be positive and finite");
    if(max_points_<2 || max_points_>10000) throw std::invalid_argument("Invalid max_points");
    std::random_device random; std::ostringstream id; id<<std::hex<<random()<<random(); boot_=id.str();
    nav_=rclcpp_action::create_client<Nav>(this,declare_parameter("navigation_action",std::string("/route/navigate_to_pose")));
    status_=create_publisher<std_msgs::msg::String>("~/status",rclcpp::QoS(1).transient_local());
    start_=create_service<Start>("~/start",[this](Start::Request::SharedPtr req,Start::Response::SharedPtr res) {
      res->boot_id=boot_;res->active=active_;res->state=state_;
      if(req->expected_boot_id!=boot_) {res->reason="Executor instance mismatch; refresh status and explicitly confirm a new request";return;}
      if(!request_id_.empty() && req->request_id==request_id_) {
        res->accepted=req->waypoints==route_.waypoints && req->dwell_sec==route_.dwell_sec;
        res->route_id=route_id_;res->reason=res->accepted?"Duplicate request; "+state_:"Request ID reused with different contents";return;
      }
      if(seen_requests_.count(req->request_id)) {res->reason="Request ID belongs to a retired route; not restarting it";return;}
      if(seen_requests_.size()>=10000) {res->reason="Request history full; restart an idle executor before new routes";return;}
      if(shutting_down_) {res->reason="Executor shutting down";return;}
      if(active_) {res->reason="Route active or cancellation unconfirmed";return;}
      if(req->request_id.empty() || req->request_id.size()>128 || req->waypoints.size()<2 ||
        req->waypoints.size()>static_cast<size_t>(max_points_) || !std::isfinite(req->dwell_sec) || req->dwell_sec<0 || req->dwell_sec>3600) {
        res->reason="Require request ID, 2..max_points points, and dwell in [0,3600] seconds";return;
      }
      Json points=Json::array();const auto frame=req->waypoints.front().header.frame_id;
      if(frame.empty() || frame.front()=='/') {res->reason="Invalid common frame";return;}
      for(const auto & p:req->waypoints) {
        const auto & q=p.pose.orientation; const auto & v=p.pose.position;
        const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
        if(p.header.frame_id!=frame || !std::isfinite(v.x)||!std::isfinite(v.y)||!std::isfinite(v.z)||
          !std::isfinite(norm)||std::abs(norm-1)>1e-3||std::abs(v.z)>1e-6||std::abs(q.x)>1e-6||std::abs(q.y)>1e-6) {
          res->reason="Waypoints must be finite planar poses in one frame with unit quaternion";return;
        }
        points.push_back({{"frame",frame},{"x",v.x},{"y",v.y},{"yaw",2*std::atan2(q.z,q.w)}});
      }
      route_=*req;points_=points;request_id_=req->request_id;seen_requests_.insert(request_id_);route_id_=boot_+"/"+request_id_;
      index_=0;cycles_=0;active_=true;stopping_=false;user_cancel_=false;
      deadline_=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(server_timeout_));
      res->accepted=true;res->route_id=route_id_;res->reason="Accepted; loops until canceled or navigation fails";
      state("WAIT_SERVER","Route accepted");res->active=active_;res->state=state_;
    });
    cancel_=create_service<Cancel>("~/cancel",[this](Cancel::Request::SharedPtr req,Cancel::Response::SharedPtr res) {
      if(req->route_id.empty() || req->route_id!=route_id_) {res->reason="Route ID mismatch";return;}
      res->accepted=true;res->reason="Cancellation requested; verify terminal status";
      stop("Canceled by operator",true);
    });
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{tick();}); publish();
  }
  void shutdownRoute() {shutting_down_=true;stop("Executor shutting down",true);}
  bool active() const {return active_;}
};
}
#ifndef ASTRIBOT_ROUTE_EXECUTOR_NO_MAIN
namespace {volatile std::sig_atomic_t shutdown_requested=0;void request_shutdown(int){shutdown_requested=1;}}
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,request_shutdown);std::signal(SIGTERM,request_shutdown);
  auto node=std::make_shared<astribot_route_executor::LoopRouteExecutor>();
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  while(rclcpp::ok()&&!shutdown_requested) {executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));}
  node->shutdownRoute();auto end=std::chrono::steady_clock::now()+std::chrono::seconds(10);
  while(rclcpp::ok()&&node->active()&&std::chrono::steady_clock::now()<end) {executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));}
  const bool pending=node->active();
  if(pending) RCLCPP_ERROR(node->get_logger(),"Shutdown: navigation cancellation is NOT confirmed");
  rclcpp::shutdown();return pending?2:0;
}
#endif
