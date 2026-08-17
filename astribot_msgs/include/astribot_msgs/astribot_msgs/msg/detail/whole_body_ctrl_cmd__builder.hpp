// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/WholeBodyCtrlCmd.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__WHOLE_BODY_CTRL_CMD__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__WHOLE_BODY_CTRL_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/whole_body_ctrl_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_WholeBodyCtrlCmd_smooth_duration
{
public:
  explicit Init_WholeBodyCtrlCmd_smooth_duration(::astribot_msgs::msg::WholeBodyCtrlCmd & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::WholeBodyCtrlCmd smooth_duration(::astribot_msgs::msg::WholeBodyCtrlCmd::_smooth_duration_type arg)
  {
    msg_.smooth_duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::WholeBodyCtrlCmd msg_;
};

class Init_WholeBodyCtrlCmd_smooth_t
{
public:
  explicit Init_WholeBodyCtrlCmd_smooth_t(::astribot_msgs::msg::WholeBodyCtrlCmd & msg)
  : msg_(msg)
  {}
  Init_WholeBodyCtrlCmd_smooth_duration smooth_t(::astribot_msgs::msg::WholeBodyCtrlCmd::_smooth_t_type arg)
  {
    msg_.smooth_t = std::move(arg);
    return Init_WholeBodyCtrlCmd_smooth_duration(msg_);
  }

private:
  ::astribot_msgs::msg::WholeBodyCtrlCmd msg_;
};

class Init_WholeBodyCtrlCmd_enable_collision_avoidance
{
public:
  explicit Init_WholeBodyCtrlCmd_enable_collision_avoidance(::astribot_msgs::msg::WholeBodyCtrlCmd & msg)
  : msg_(msg)
  {}
  Init_WholeBodyCtrlCmd_smooth_t enable_collision_avoidance(::astribot_msgs::msg::WholeBodyCtrlCmd::_enable_collision_avoidance_type arg)
  {
    msg_.enable_collision_avoidance = std::move(arg);
    return Init_WholeBodyCtrlCmd_smooth_t(msg_);
  }

private:
  ::astribot_msgs::msg::WholeBodyCtrlCmd msg_;
};

class Init_WholeBodyCtrlCmd_torso_open_loop
{
public:
  explicit Init_WholeBodyCtrlCmd_torso_open_loop(::astribot_msgs::msg::WholeBodyCtrlCmd & msg)
  : msg_(msg)
  {}
  Init_WholeBodyCtrlCmd_enable_collision_avoidance torso_open_loop(::astribot_msgs::msg::WholeBodyCtrlCmd::_torso_open_loop_type arg)
  {
    msg_.torso_open_loop = std::move(arg);
    return Init_WholeBodyCtrlCmd_enable_collision_avoidance(msg_);
  }

private:
  ::astribot_msgs::msg::WholeBodyCtrlCmd msg_;
};

class Init_WholeBodyCtrlCmd_pose_world_to_chassis_current
{
public:
  explicit Init_WholeBodyCtrlCmd_pose_world_to_chassis_current(::astribot_msgs::msg::WholeBodyCtrlCmd & msg)
  : msg_(msg)
  {}
  Init_WholeBodyCtrlCmd_torso_open_loop pose_world_to_chassis_current(::astribot_msgs::msg::WholeBodyCtrlCmd::_pose_world_to_chassis_current_type arg)
  {
    msg_.pose_world_to_chassis_current = std::move(arg);
    return Init_WholeBodyCtrlCmd_torso_open_loop(msg_);
  }

private:
  ::astribot_msgs::msg::WholeBodyCtrlCmd msg_;
};

class Init_WholeBodyCtrlCmd_pose_world_to_right_arm_desired
{
public:
  explicit Init_WholeBodyCtrlCmd_pose_world_to_right_arm_desired(::astribot_msgs::msg::WholeBodyCtrlCmd & msg)
  : msg_(msg)
  {}
  Init_WholeBodyCtrlCmd_pose_world_to_chassis_current pose_world_to_right_arm_desired(::astribot_msgs::msg::WholeBodyCtrlCmd::_pose_world_to_right_arm_desired_type arg)
  {
    msg_.pose_world_to_right_arm_desired = std::move(arg);
    return Init_WholeBodyCtrlCmd_pose_world_to_chassis_current(msg_);
  }

private:
  ::astribot_msgs::msg::WholeBodyCtrlCmd msg_;
};

class Init_WholeBodyCtrlCmd_pose_world_to_left_arm_desired
{
public:
  explicit Init_WholeBodyCtrlCmd_pose_world_to_left_arm_desired(::astribot_msgs::msg::WholeBodyCtrlCmd & msg)
  : msg_(msg)
  {}
  Init_WholeBodyCtrlCmd_pose_world_to_right_arm_desired pose_world_to_left_arm_desired(::astribot_msgs::msg::WholeBodyCtrlCmd::_pose_world_to_left_arm_desired_type arg)
  {
    msg_.pose_world_to_left_arm_desired = std::move(arg);
    return Init_WholeBodyCtrlCmd_pose_world_to_right_arm_desired(msg_);
  }

private:
  ::astribot_msgs::msg::WholeBodyCtrlCmd msg_;
};

class Init_WholeBodyCtrlCmd_pose_world_to_torso_desired
{
public:
  explicit Init_WholeBodyCtrlCmd_pose_world_to_torso_desired(::astribot_msgs::msg::WholeBodyCtrlCmd & msg)
  : msg_(msg)
  {}
  Init_WholeBodyCtrlCmd_pose_world_to_left_arm_desired pose_world_to_torso_desired(::astribot_msgs::msg::WholeBodyCtrlCmd::_pose_world_to_torso_desired_type arg)
  {
    msg_.pose_world_to_torso_desired = std::move(arg);
    return Init_WholeBodyCtrlCmd_pose_world_to_left_arm_desired(msg_);
  }

private:
  ::astribot_msgs::msg::WholeBodyCtrlCmd msg_;
};

class Init_WholeBodyCtrlCmd_torso_twist
{
public:
  explicit Init_WholeBodyCtrlCmd_torso_twist(::astribot_msgs::msg::WholeBodyCtrlCmd & msg)
  : msg_(msg)
  {}
  Init_WholeBodyCtrlCmd_pose_world_to_torso_desired torso_twist(::astribot_msgs::msg::WholeBodyCtrlCmd::_torso_twist_type arg)
  {
    msg_.torso_twist = std::move(arg);
    return Init_WholeBodyCtrlCmd_pose_world_to_torso_desired(msg_);
  }

private:
  ::astribot_msgs::msg::WholeBodyCtrlCmd msg_;
};

class Init_WholeBodyCtrlCmd_right_arm_twist
{
public:
  explicit Init_WholeBodyCtrlCmd_right_arm_twist(::astribot_msgs::msg::WholeBodyCtrlCmd & msg)
  : msg_(msg)
  {}
  Init_WholeBodyCtrlCmd_torso_twist right_arm_twist(::astribot_msgs::msg::WholeBodyCtrlCmd::_right_arm_twist_type arg)
  {
    msg_.right_arm_twist = std::move(arg);
    return Init_WholeBodyCtrlCmd_torso_twist(msg_);
  }

private:
  ::astribot_msgs::msg::WholeBodyCtrlCmd msg_;
};

class Init_WholeBodyCtrlCmd_left_arm_twist
{
public:
  explicit Init_WholeBodyCtrlCmd_left_arm_twist(::astribot_msgs::msg::WholeBodyCtrlCmd & msg)
  : msg_(msg)
  {}
  Init_WholeBodyCtrlCmd_right_arm_twist left_arm_twist(::astribot_msgs::msg::WholeBodyCtrlCmd::_left_arm_twist_type arg)
  {
    msg_.left_arm_twist = std::move(arg);
    return Init_WholeBodyCtrlCmd_right_arm_twist(msg_);
  }

private:
  ::astribot_msgs::msg::WholeBodyCtrlCmd msg_;
};

class Init_WholeBodyCtrlCmd_header
{
public:
  Init_WholeBodyCtrlCmd_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WholeBodyCtrlCmd_left_arm_twist header(::astribot_msgs::msg::WholeBodyCtrlCmd::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_WholeBodyCtrlCmd_left_arm_twist(msg_);
  }

private:
  ::astribot_msgs::msg::WholeBodyCtrlCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::WholeBodyCtrlCmd>()
{
  return astribot_msgs::msg::builder::Init_WholeBodyCtrlCmd_header();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__WHOLE_BODY_CTRL_CMD__BUILDER_HPP_
