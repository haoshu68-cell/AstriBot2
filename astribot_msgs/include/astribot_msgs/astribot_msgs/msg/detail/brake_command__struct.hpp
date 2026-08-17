// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:msg/BrakeCommand.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__BRAKE_COMMAND__STRUCT_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__BRAKE_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__astribot_msgs__msg__BrakeCommand __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__msg__BrakeCommand __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BrakeCommand_
{
  using Type = BrakeCommand_<ContainerAllocator>;

  explicit BrakeCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->brake = 0l;
    }
  }

  explicit BrakeCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->brake = 0l;
    }
  }

  // field types and members
  using _brake_type =
    int32_t;
  _brake_type brake;

  // setters for named parameter idiom
  Type & set__brake(
    const int32_t & _arg)
  {
    this->brake = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int32_t OPEN_BREAK_CMD =
    1;
  static constexpr int32_t CLOSE_BREAK_CMD =
    0;

  // pointer types
  using RawPtr =
    astribot_msgs::msg::BrakeCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::msg::BrakeCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::msg::BrakeCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::msg::BrakeCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::BrakeCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::BrakeCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::BrakeCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::BrakeCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::msg::BrakeCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::msg::BrakeCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__msg__BrakeCommand
    std::shared_ptr<astribot_msgs::msg::BrakeCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__msg__BrakeCommand
    std::shared_ptr<astribot_msgs::msg::BrakeCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BrakeCommand_ & other) const
  {
    if (this->brake != other.brake) {
      return false;
    }
    return true;
  }
  bool operator!=(const BrakeCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BrakeCommand_

// alias to use template instance with default allocator
using BrakeCommand =
  astribot_msgs::msg::BrakeCommand_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t BrakeCommand_<ContainerAllocator>::OPEN_BREAK_CMD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t BrakeCommand_<ContainerAllocator>::CLOSE_BREAK_CMD;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__BRAKE_COMMAND__STRUCT_HPP_
