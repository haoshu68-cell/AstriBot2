// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:msg/RobotCartesianStates.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_CARTESIAN_STATES__STRUCT_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_CARTESIAN_STATES__STRUCT_HPP_

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
// Member 'states'
#include "astribot_msgs/msg/detail/robot_cartesian_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__astribot_msgs__msg__RobotCartesianStates __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__msg__RobotCartesianStates __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotCartesianStates_
{
  using Type = RobotCartesianStates_<ContainerAllocator>;

  explicit RobotCartesianStates_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit RobotCartesianStates_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _names_type names;
  using _states_type =
    std::vector<astribot_msgs::msg::RobotCartesianState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<astribot_msgs::msg::RobotCartesianState_<ContainerAllocator>>>;
  _states_type states;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->names = _arg;
    return *this;
  }
  Type & set__states(
    const std::vector<astribot_msgs::msg::RobotCartesianState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<astribot_msgs::msg::RobotCartesianState_<ContainerAllocator>>> & _arg)
  {
    this->states = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::msg::RobotCartesianStates_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::msg::RobotCartesianStates_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::msg::RobotCartesianStates_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::msg::RobotCartesianStates_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::RobotCartesianStates_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::RobotCartesianStates_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::RobotCartesianStates_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::RobotCartesianStates_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::msg::RobotCartesianStates_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::msg::RobotCartesianStates_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__msg__RobotCartesianStates
    std::shared_ptr<astribot_msgs::msg::RobotCartesianStates_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__msg__RobotCartesianStates
    std::shared_ptr<astribot_msgs::msg::RobotCartesianStates_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotCartesianStates_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->names != other.names) {
      return false;
    }
    if (this->states != other.states) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotCartesianStates_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotCartesianStates_

// alias to use template instance with default allocator
using RobotCartesianStates =
  astribot_msgs::msg::RobotCartesianStates_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_CARTESIAN_STATES__STRUCT_HPP_
