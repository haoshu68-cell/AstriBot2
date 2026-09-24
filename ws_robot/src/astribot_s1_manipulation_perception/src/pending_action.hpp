#pragma once
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>

namespace astribot::perception_planning::detail {
template<class Action> struct Pending {
  using Client=rclcpp_action::Client<Action>;
  using Handle=rclcpp_action::ClientGoalHandle<Action>;
  typename Client::SharedPtr client;
  std::shared_future<typename Handle::SharedPtr> response;
  std::shared_future<typename Handle::WrappedResult> result;
  typename Handle::SharedPtr handle;
  bool sent{false},rejected{false},cancel_sent{false};
  void send(const typename Action::Goal &g) {response=client->async_send_goal(g);sent=true;}
  void poll() {
    if(sent&&!handle&&!rejected&&response.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
      handle=response.get();
      if(handle)result=client->async_get_result(handle);else rejected=true;
    }
  }
  bool received() const {return result.valid()&&result.wait_for(std::chrono::seconds(0))==std::future_status::ready;}
  bool terminal() const {
    if(!sent||rejected)return true;
    if(!received())return false;
    const auto code=result.get().code;
    return code==rclcpp_action::ResultCode::SUCCEEDED||code==rclcpp_action::ResultCode::ABORTED||
      code==rclcpp_action::ResultCode::CANCELED;
  }
  void cancel() {
    poll();
    if(handle&&!terminal()&&!cancel_sent) {client->async_cancel_goal(handle);cancel_sent=true;}
  }
};
} // namespace astribot::perception_planning::detail
