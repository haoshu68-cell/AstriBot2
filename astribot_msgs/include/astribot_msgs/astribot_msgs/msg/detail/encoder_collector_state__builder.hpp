// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/EncoderCollectorState.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ENCODER_COLLECTOR_STATE__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ENCODER_COLLECTOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/encoder_collector_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_EncoderCollectorState_rx_sequence_count
{
public:
  explicit Init_EncoderCollectorState_rx_sequence_count(::astribot_msgs::msg::EncoderCollectorState & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::EncoderCollectorState rx_sequence_count(::astribot_msgs::msg::EncoderCollectorState::_rx_sequence_count_type arg)
  {
    msg_.rx_sequence_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::EncoderCollectorState msg_;
};

class Init_EncoderCollectorState_acceleration_rpss
{
public:
  explicit Init_EncoderCollectorState_acceleration_rpss(::astribot_msgs::msg::EncoderCollectorState & msg)
  : msg_(msg)
  {}
  Init_EncoderCollectorState_rx_sequence_count acceleration_rpss(::astribot_msgs::msg::EncoderCollectorState::_acceleration_rpss_type arg)
  {
    msg_.acceleration_rpss = std::move(arg);
    return Init_EncoderCollectorState_rx_sequence_count(msg_);
  }

private:
  ::astribot_msgs::msg::EncoderCollectorState msg_;
};

class Init_EncoderCollectorState_velocity_rps
{
public:
  explicit Init_EncoderCollectorState_velocity_rps(::astribot_msgs::msg::EncoderCollectorState & msg)
  : msg_(msg)
  {}
  Init_EncoderCollectorState_acceleration_rpss velocity_rps(::astribot_msgs::msg::EncoderCollectorState::_velocity_rps_type arg)
  {
    msg_.velocity_rps = std::move(arg);
    return Init_EncoderCollectorState_acceleration_rpss(msg_);
  }

private:
  ::astribot_msgs::msg::EncoderCollectorState msg_;
};

class Init_EncoderCollectorState_position_rad
{
public:
  Init_EncoderCollectorState_position_rad()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EncoderCollectorState_velocity_rps position_rad(::astribot_msgs::msg::EncoderCollectorState::_position_rad_type arg)
  {
    msg_.position_rad = std::move(arg);
    return Init_EncoderCollectorState_velocity_rps(msg_);
  }

private:
  ::astribot_msgs::msg::EncoderCollectorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::EncoderCollectorState>()
{
  return astribot_msgs::msg::builder::Init_EncoderCollectorState_position_rad();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ENCODER_COLLECTOR_STATE__BUILDER_HPP_
