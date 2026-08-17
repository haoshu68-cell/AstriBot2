// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/MotorState.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/motor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorState_j_a
{
public:
  explicit Init_MotorState_j_a(::astribot_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::MotorState j_a(::astribot_msgs::msg::MotorState::_j_a_type arg)
  {
    msg_.j_a = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::MotorState msg_;
};

class Init_MotorState_j_v
{
public:
  explicit Init_MotorState_j_v(::astribot_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_j_a j_v(::astribot_msgs::msg::MotorState::_j_v_type arg)
  {
    msg_.j_v = std::move(arg);
    return Init_MotorState_j_a(msg_);
  }

private:
  ::astribot_msgs::msg::MotorState msg_;
};

class Init_MotorState_j_p
{
public:
  explicit Init_MotorState_j_p(::astribot_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_j_v j_p(::astribot_msgs::msg::MotorState::_j_p_type arg)
  {
    msg_.j_p = std::move(arg);
    return Init_MotorState_j_v(msg_);
  }

private:
  ::astribot_msgs::msg::MotorState msg_;
};

class Init_MotorState_j_s
{
public:
  explicit Init_MotorState_j_s(::astribot_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_j_p j_s(::astribot_msgs::msg::MotorState::_j_s_type arg)
  {
    msg_.j_s = std::move(arg);
    return Init_MotorState_j_p(msg_);
  }

private:
  ::astribot_msgs::msg::MotorState msg_;
};

class Init_MotorState_can_error
{
public:
  explicit Init_MotorState_can_error(::astribot_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_j_s can_error(::astribot_msgs::msg::MotorState::_can_error_type arg)
  {
    msg_.can_error = std::move(arg);
    return Init_MotorState_j_s(msg_);
  }

private:
  ::astribot_msgs::msg::MotorState msg_;
};

class Init_MotorState_can_count_last
{
public:
  explicit Init_MotorState_can_count_last(::astribot_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_can_error can_count_last(::astribot_msgs::msg::MotorState::_can_count_last_type arg)
  {
    msg_.can_count_last = std::move(arg);
    return Init_MotorState_can_error(msg_);
  }

private:
  ::astribot_msgs::msg::MotorState msg_;
};

class Init_MotorState_can_count
{
public:
  explicit Init_MotorState_can_count(::astribot_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_can_count_last can_count(::astribot_msgs::msg::MotorState::_can_count_type arg)
  {
    msg_.can_count = std::move(arg);
    return Init_MotorState_can_count_last(msg_);
  }

private:
  ::astribot_msgs::msg::MotorState msg_;
};

class Init_MotorState_acc
{
public:
  explicit Init_MotorState_acc(::astribot_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_can_count acc(::astribot_msgs::msg::MotorState::_acc_type arg)
  {
    msg_.acc = std::move(arg);
    return Init_MotorState_can_count(msg_);
  }

private:
  ::astribot_msgs::msg::MotorState msg_;
};

class Init_MotorState_vol
{
public:
  explicit Init_MotorState_vol(::astribot_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_acc vol(::astribot_msgs::msg::MotorState::_vol_type arg)
  {
    msg_.vol = std::move(arg);
    return Init_MotorState_acc(msg_);
  }

private:
  ::astribot_msgs::msg::MotorState msg_;
};

class Init_MotorState_c
{
public:
  explicit Init_MotorState_c(::astribot_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_vol c(::astribot_msgs::msg::MotorState::_c_type arg)
  {
    msg_.c = std::move(arg);
    return Init_MotorState_vol(msg_);
  }

private:
  ::astribot_msgs::msg::MotorState msg_;
};

class Init_MotorState_v
{
public:
  explicit Init_MotorState_v(::astribot_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_c v(::astribot_msgs::msg::MotorState::_v_type arg)
  {
    msg_.v = std::move(arg);
    return Init_MotorState_c(msg_);
  }

private:
  ::astribot_msgs::msg::MotorState msg_;
};

class Init_MotorState_p
{
public:
  Init_MotorState_p()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorState_v p(::astribot_msgs::msg::MotorState::_p_type arg)
  {
    msg_.p = std::move(arg);
    return Init_MotorState_v(msg_);
  }

private:
  ::astribot_msgs::msg::MotorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::MotorState>()
{
  return astribot_msgs::msg::builder::Init_MotorState_p();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
