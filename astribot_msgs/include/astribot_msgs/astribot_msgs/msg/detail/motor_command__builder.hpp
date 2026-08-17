// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/motor_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorCommand_motor_id_list
{
public:
  explicit Init_MotorCommand_motor_id_list(::astribot_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::MotorCommand motor_id_list(::astribot_msgs::msg::MotorCommand::_motor_id_list_type arg)
  {
    msg_.motor_id_list = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_vel_slave
{
public:
  explicit Init_MotorCommand_vel_slave(::astribot_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_motor_id_list vel_slave(::astribot_msgs::msg::MotorCommand::_vel_slave_type arg)
  {
    msg_.vel_slave = std::move(arg);
    return Init_MotorCommand_motor_id_list(msg_);
  }

private:
  ::astribot_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_kd_slave
{
public:
  explicit Init_MotorCommand_kd_slave(::astribot_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_vel_slave kd_slave(::astribot_msgs::msg::MotorCommand::_kd_slave_type arg)
  {
    msg_.kd_slave = std::move(arg);
    return Init_MotorCommand_vel_slave(msg_);
  }

private:
  ::astribot_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_break_relase
{
public:
  explicit Init_MotorCommand_break_relase(::astribot_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_kd_slave break_relase(::astribot_msgs::msg::MotorCommand::_break_relase_type arg)
  {
    msg_.break_relase = std::move(arg);
    return Init_MotorCommand_kd_slave(msg_);
  }

private:
  ::astribot_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_t_limit
{
public:
  explicit Init_MotorCommand_t_limit(::astribot_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_break_relase t_limit(::astribot_msgs::msg::MotorCommand::_t_limit_type arg)
  {
    msg_.t_limit = std::move(arg);
    return Init_MotorCommand_break_relase(msg_);
  }

private:
  ::astribot_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_t_ff
{
public:
  explicit Init_MotorCommand_t_ff(::astribot_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_t_limit t_ff(::astribot_msgs::msg::MotorCommand::_t_ff_type arg)
  {
    msg_.t_ff = std::move(arg);
    return Init_MotorCommand_t_limit(msg_);
  }

private:
  ::astribot_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_v
{
public:
  explicit Init_MotorCommand_v(::astribot_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_t_ff v(::astribot_msgs::msg::MotorCommand::_v_type arg)
  {
    msg_.v = std::move(arg);
    return Init_MotorCommand_t_ff(msg_);
  }

private:
  ::astribot_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_p
{
public:
  explicit Init_MotorCommand_p(::astribot_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_v p(::astribot_msgs::msg::MotorCommand::_p_type arg)
  {
    msg_.p = std::move(arg);
    return Init_MotorCommand_v(msg_);
  }

private:
  ::astribot_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_kd
{
public:
  explicit Init_MotorCommand_kd(::astribot_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_p kd(::astribot_msgs::msg::MotorCommand::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return Init_MotorCommand_p(msg_);
  }

private:
  ::astribot_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_kp
{
public:
  Init_MotorCommand_kp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorCommand_kd kp(::astribot_msgs::msg::MotorCommand::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_MotorCommand_kd(msg_);
  }

private:
  ::astribot_msgs::msg::MotorCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::MotorCommand>()
{
  return astribot_msgs::msg::builder::Init_MotorCommand_kp();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
