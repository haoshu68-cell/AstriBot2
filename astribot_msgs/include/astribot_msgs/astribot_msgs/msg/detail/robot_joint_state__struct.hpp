// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:msg/RobotJointState.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_JOINT_STATE__STRUCT_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_JOINT_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__astribot_msgs__msg__RobotJointState __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__msg__RobotJointState __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotJointState_
{
  using Type = RobotJointState_<ContainerAllocator>;

  explicit RobotJointState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
    }
  }

  explicit RobotJointState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _mode_type =
    int8_t;
  _mode_type mode;
  using _name_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _name_type name;
  using _position_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _position_type position;
  using _velocity_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _velocity_type velocity;
  using _acceleration_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _acceleration_type acceleration;
  using _torque_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _torque_type torque;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__mode(
    const int8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__name(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__position(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__acceleration(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }
  Type & set__torque(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->torque = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::msg::RobotJointState_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::msg::RobotJointState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::msg::RobotJointState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::msg::RobotJointState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::RobotJointState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::RobotJointState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::RobotJointState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::RobotJointState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::msg::RobotJointState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::msg::RobotJointState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__msg__RobotJointState
    std::shared_ptr<astribot_msgs::msg::RobotJointState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__msg__RobotJointState
    std::shared_ptr<astribot_msgs::msg::RobotJointState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotJointState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->acceleration != other.acceleration) {
      return false;
    }
    if (this->torque != other.torque) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotJointState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotJointState_

// alias to use template instance with default allocator
using RobotJointState =
  astribot_msgs::msg::RobotJointState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_JOINT_STATE__STRUCT_HPP_
