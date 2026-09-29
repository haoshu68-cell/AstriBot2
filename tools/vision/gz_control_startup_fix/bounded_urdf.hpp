#pragma once
#include <algorithm>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>

namespace astribot {
// Startup only; the caller's executor must already be spinning. No commands are
// published. Each timed-out request is removed so a late reply cannot be reused.
inline std::string fetch_urdf(
  const rclcpp::Node::SharedPtr& node, const std::string& remote,
  const std::string& parameter, std::chrono::milliseconds budget,
  std::chrono::milliseconds attempt)
{
  using Clock=std::chrono::steady_clock;
  if (budget.count()<=0 || attempt.count()<=0) return {};
  auto client=node->create_client<rcl_interfaces::srv::GetParameters>(remote+"/get_parameters");
  const auto deadline=Clock::now()+budget;
  unsigned requests=0;
  while (rclcpp::ok(node->get_node_base_interface()->get_context()) && Clock::now()<deadline) {
    const auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-Clock::now());
    if (!client->wait_for_service(std::min(remaining,std::chrono::milliseconds(100)))) continue;
    auto request=std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names={parameter};
    auto pending=client->async_send_request(request);++requests;
    const auto wait=std::min(attempt,std::chrono::duration_cast<std::chrono::milliseconds>(deadline-Clock::now()));
    if (pending.wait_for(wait)!=std::future_status::ready) {
      client->remove_pending_request(pending);
      RCLCPP_WARN(node->get_logger(),"URDF request %u timed out; bounded retry",requests);
      continue;
    }
    const auto reply=pending.get();
    if (reply && reply->values.size()==1 &&
        reply->values[0].type==rcl_interfaces::msg::ParameterType::PARAMETER_STRING &&
        !reply->values[0].string_value.empty()) {
      RCLCPP_INFO(node->get_logger(),"Received URDF after %u bounded request(s)",requests);
      return reply->values[0].string_value;
    }
    // A present but invalid parameter is a configuration error, not a reason
    // to spin or to accept an old cached robot model.
    RCLCPP_ERROR(node->get_logger(),"URDF parameter missing, empty or wrong type");
    return {};
  }
  RCLCPP_ERROR(node->get_logger(),"URDF_STARTUP_DEADLINE: model unavailable; control setup rejected");
  return {};
}
}
