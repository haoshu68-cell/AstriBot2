// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:msg/LaunchRequest.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__LAUNCH_REQUEST__STRUCT_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__LAUNCH_REQUEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__astribot_msgs__msg__LaunchRequest __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__msg__LaunchRequest __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LaunchRequest_
{
  using Type = LaunchRequest_<ContainerAllocator>;

  explicit LaunchRequest_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->launch_target = 0;
      this->launch_action = 0;
    }
  }

  explicit LaunchRequest_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->launch_target = 0;
      this->launch_action = 0;
    }
  }

  // field types and members
  using _launch_target_type =
    uint8_t;
  _launch_target_type launch_target;
  using _launch_action_type =
    uint8_t;
  _launch_action_type launch_action;

  // setters for named parameter idiom
  Type & set__launch_target(
    const uint8_t & _arg)
  {
    this->launch_target = _arg;
    return *this;
  }
  Type & set__launch_action(
    const uint8_t & _arg)
  {
    this->launch_action = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t ENUM_UNKNOWN =
    0u;
  static constexpr uint8_t ENUM_ASTRIBOT_CAMERA =
    1u;
  static constexpr uint8_t ENUM_ASTRIBOT_JOY =
    2u;
  static constexpr uint8_t ENUM_ASTRIBOT_VR =
    3u;
  static constexpr uint8_t ENUM_ASTRIBOT_MIC =
    4u;
  static constexpr uint8_t ENUM_ASTRIBOT_LIDAR =
    5u;
  static constexpr uint8_t ENUM_ASTRIBOT_TELEOP =
    6u;
  static constexpr uint8_t ENUM_DEACTIVE =
    1u;
  static constexpr uint8_t ENUM_ACTIVE =
    2u;

  // pointer types
  using RawPtr =
    astribot_msgs::msg::LaunchRequest_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::msg::LaunchRequest_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::msg::LaunchRequest_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::msg::LaunchRequest_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::LaunchRequest_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::LaunchRequest_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::LaunchRequest_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::LaunchRequest_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::msg::LaunchRequest_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::msg::LaunchRequest_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__msg__LaunchRequest
    std::shared_ptr<astribot_msgs::msg::LaunchRequest_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__msg__LaunchRequest
    std::shared_ptr<astribot_msgs::msg::LaunchRequest_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LaunchRequest_ & other) const
  {
    if (this->launch_target != other.launch_target) {
      return false;
    }
    if (this->launch_action != other.launch_action) {
      return false;
    }
    return true;
  }
  bool operator!=(const LaunchRequest_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LaunchRequest_

// alias to use template instance with default allocator
using LaunchRequest =
  astribot_msgs::msg::LaunchRequest_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LaunchRequest_<ContainerAllocator>::ENUM_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LaunchRequest_<ContainerAllocator>::ENUM_ASTRIBOT_CAMERA;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LaunchRequest_<ContainerAllocator>::ENUM_ASTRIBOT_JOY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LaunchRequest_<ContainerAllocator>::ENUM_ASTRIBOT_VR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LaunchRequest_<ContainerAllocator>::ENUM_ASTRIBOT_MIC;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LaunchRequest_<ContainerAllocator>::ENUM_ASTRIBOT_LIDAR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LaunchRequest_<ContainerAllocator>::ENUM_ASTRIBOT_TELEOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LaunchRequest_<ContainerAllocator>::ENUM_DEACTIVE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LaunchRequest_<ContainerAllocator>::ENUM_ACTIVE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__LAUNCH_REQUEST__STRUCT_HPP_
