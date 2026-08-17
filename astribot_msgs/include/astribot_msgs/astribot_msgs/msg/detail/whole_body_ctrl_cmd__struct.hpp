// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:msg/WholeBodyCtrlCmd.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__WHOLE_BODY_CTRL_CMD__STRUCT_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__WHOLE_BODY_CTRL_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "astribot_msgs/msg/detail/astribot_header__struct.hpp"
// Member 'left_arm_twist'
// Member 'right_arm_twist'
// Member 'torso_twist'
#include "geometry_msgs/msg/detail/twist__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__astribot_msgs__msg__WholeBodyCtrlCmd __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__msg__WholeBodyCtrlCmd __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WholeBodyCtrlCmd_
{
  using Type = WholeBodyCtrlCmd_<ContainerAllocator>;

  explicit WholeBodyCtrlCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    left_arm_twist(_init),
    right_arm_twist(_init),
    torso_twist(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->torso_open_loop = false;
      this->enable_collision_avoidance = false;
      this->smooth_t = 0.0;
      this->smooth_duration = 0.0;
    }
  }

  explicit WholeBodyCtrlCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    left_arm_twist(_alloc, _init),
    right_arm_twist(_alloc, _init),
    torso_twist(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->torso_open_loop = false;
      this->enable_collision_avoidance = false;
      this->smooth_t = 0.0;
      this->smooth_duration = 0.0;
    }
  }

  // field types and members
  using _header_type =
    astribot_msgs::msg::AstribotHeader_<ContainerAllocator>;
  _header_type header;
  using _left_arm_twist_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _left_arm_twist_type left_arm_twist;
  using _right_arm_twist_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _right_arm_twist_type right_arm_twist;
  using _torso_twist_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _torso_twist_type torso_twist;
  using _pose_world_to_torso_desired_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _pose_world_to_torso_desired_type pose_world_to_torso_desired;
  using _pose_world_to_left_arm_desired_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _pose_world_to_left_arm_desired_type pose_world_to_left_arm_desired;
  using _pose_world_to_right_arm_desired_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _pose_world_to_right_arm_desired_type pose_world_to_right_arm_desired;
  using _pose_world_to_chassis_current_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _pose_world_to_chassis_current_type pose_world_to_chassis_current;
  using _torso_open_loop_type =
    bool;
  _torso_open_loop_type torso_open_loop;
  using _enable_collision_avoidance_type =
    bool;
  _enable_collision_avoidance_type enable_collision_avoidance;
  using _smooth_t_type =
    double;
  _smooth_t_type smooth_t;
  using _smooth_duration_type =
    double;
  _smooth_duration_type smooth_duration;

  // setters for named parameter idiom
  Type & set__header(
    const astribot_msgs::msg::AstribotHeader_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__left_arm_twist(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->left_arm_twist = _arg;
    return *this;
  }
  Type & set__right_arm_twist(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->right_arm_twist = _arg;
    return *this;
  }
  Type & set__torso_twist(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->torso_twist = _arg;
    return *this;
  }
  Type & set__pose_world_to_torso_desired(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->pose_world_to_torso_desired = _arg;
    return *this;
  }
  Type & set__pose_world_to_left_arm_desired(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->pose_world_to_left_arm_desired = _arg;
    return *this;
  }
  Type & set__pose_world_to_right_arm_desired(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->pose_world_to_right_arm_desired = _arg;
    return *this;
  }
  Type & set__pose_world_to_chassis_current(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->pose_world_to_chassis_current = _arg;
    return *this;
  }
  Type & set__torso_open_loop(
    const bool & _arg)
  {
    this->torso_open_loop = _arg;
    return *this;
  }
  Type & set__enable_collision_avoidance(
    const bool & _arg)
  {
    this->enable_collision_avoidance = _arg;
    return *this;
  }
  Type & set__smooth_t(
    const double & _arg)
  {
    this->smooth_t = _arg;
    return *this;
  }
  Type & set__smooth_duration(
    const double & _arg)
  {
    this->smooth_duration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::msg::WholeBodyCtrlCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::msg::WholeBodyCtrlCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::msg::WholeBodyCtrlCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::msg::WholeBodyCtrlCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::WholeBodyCtrlCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::WholeBodyCtrlCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::WholeBodyCtrlCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::WholeBodyCtrlCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::msg::WholeBodyCtrlCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::msg::WholeBodyCtrlCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__msg__WholeBodyCtrlCmd
    std::shared_ptr<astribot_msgs::msg::WholeBodyCtrlCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__msg__WholeBodyCtrlCmd
    std::shared_ptr<astribot_msgs::msg::WholeBodyCtrlCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WholeBodyCtrlCmd_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->left_arm_twist != other.left_arm_twist) {
      return false;
    }
    if (this->right_arm_twist != other.right_arm_twist) {
      return false;
    }
    if (this->torso_twist != other.torso_twist) {
      return false;
    }
    if (this->pose_world_to_torso_desired != other.pose_world_to_torso_desired) {
      return false;
    }
    if (this->pose_world_to_left_arm_desired != other.pose_world_to_left_arm_desired) {
      return false;
    }
    if (this->pose_world_to_right_arm_desired != other.pose_world_to_right_arm_desired) {
      return false;
    }
    if (this->pose_world_to_chassis_current != other.pose_world_to_chassis_current) {
      return false;
    }
    if (this->torso_open_loop != other.torso_open_loop) {
      return false;
    }
    if (this->enable_collision_avoidance != other.enable_collision_avoidance) {
      return false;
    }
    if (this->smooth_t != other.smooth_t) {
      return false;
    }
    if (this->smooth_duration != other.smooth_duration) {
      return false;
    }
    return true;
  }
  bool operator!=(const WholeBodyCtrlCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WholeBodyCtrlCmd_

// alias to use template instance with default allocator
using WholeBodyCtrlCmd =
  astribot_msgs::msg::WholeBodyCtrlCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__WHOLE_BODY_CTRL_CMD__STRUCT_HPP_
