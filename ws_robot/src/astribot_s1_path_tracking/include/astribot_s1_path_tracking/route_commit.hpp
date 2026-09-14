#ifndef ASTRIBOT_S1_PATH_TRACKING__ROUTE_COMMIT_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__ROUTE_COMMIT_HPP_
#include <chrono>
#include "behaviortree_cpp_v3/exceptions.h"
#include "astribot_navigation_msgs/srv/resolve_route.hpp"
#include "rclcpp/rclcpp.hpp"
namespace astribot_s1_path_tracking {
// Used only on the BT tick thread. Futures have no authority after reset().
class RouteCommit {
  using Clock=std::chrono::steady_clock;
  using Service=astribot_navigation_msgs::srv::ResolveRoute;
public:
  RouteCommit(rclcpp::Node::SharedPtr node,rclcpp::CallbackGroup::SharedPtr group):node_(node) {
    client_=node->create_client<Service>("navigation_policy/resolve_route",rmw_qos_profile_services_default,group);
  }
  ~RouteCommit() {reset();}
  void reset() {
    if(pending_) {client_->remove_pending_request(request_id_);pending_=false;}
    next_=Clock::time_point();last_good_=Clock::now();
  }
  bool update(const std::string & session,nav_msgs::msg::Path & path,
              const geometry_msgs::msg::PoseStamped & goal,bool allow_detour,std::string & reason) {
    auto now=Clock::now();
    if(session!=session_ || path!=reference_) {reset();session_=session;reference_=path;}
    if(pending_ && future_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
      auto result=future_.get();pending_=false;
      const double age=(node_->now()-rclcpp::Time(result->evaluated_at,node_->get_clock()->get_clock_type())).seconds();
      if(age>=0 && age<=.15 && now-sent_<=std::chrono::milliseconds(300)) {
        last_good_=now;
        if(result->disposition==Service::Response::BLOCKED) {throw BT::RuntimeError(result->reason);}
        if(result->disposition==Service::Response::COMMIT && !result->path.poses.empty() &&
           result->path.header.frame_id==path.header.frame_id && result->path.poses.back().pose==goal.pose) {
          path=result->path;reference_=path;reason=result->reason;next_=now;
          return true;
        }
      }
      next_=now+std::chrono::milliseconds(100);
    }
    if(now-last_good_>std::chrono::seconds(2)) {reset();throw BT::RuntimeError("POLICY_ROUTE_UNAVAILABLE");}
    if(!pending_ && now>=next_ && client_->service_is_ready()) {
      auto req=std::make_shared<Service::Request>();req->allow_detour=allow_detour;req->session_id=session;req->reference_path=path;req->goal=goal;
      auto result=client_->async_send_request(req);future_=result.future.share();request_id_=result.request_id;
      pending_=true;sent_=now;
    }
    return false;
  }
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Service>::SharedPtr client_;
  std::shared_future<Service::Response::SharedPtr> future_;
  bool pending_{false};int64_t request_id_{0};
  Clock::time_point next_{},last_good_{Clock::now()},sent_{};
  std::string session_;nav_msgs::msg::Path reference_;
};
}
#endif
