// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:msg/EncoderCollectorState.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ENCODER_COLLECTOR_STATE__STRUCT_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ENCODER_COLLECTOR_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__astribot_msgs__msg__EncoderCollectorState __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__msg__EncoderCollectorState __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EncoderCollectorState_
{
  using Type = EncoderCollectorState_<ContainerAllocator>;

  explicit EncoderCollectorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->position_rad = 0.0f;
      this->velocity_rps = 0.0f;
      this->acceleration_rpss = 0.0f;
      this->rx_sequence_count = 0;
    }
  }

  explicit EncoderCollectorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->position_rad = 0.0f;
      this->velocity_rps = 0.0f;
      this->acceleration_rpss = 0.0f;
      this->rx_sequence_count = 0;
    }
  }

  // field types and members
  using _position_rad_type =
    float;
  _position_rad_type position_rad;
  using _velocity_rps_type =
    float;
  _velocity_rps_type velocity_rps;
  using _acceleration_rpss_type =
    float;
  _acceleration_rpss_type acceleration_rpss;
  using _rx_sequence_count_type =
    uint8_t;
  _rx_sequence_count_type rx_sequence_count;

  // setters for named parameter idiom
  Type & set__position_rad(
    const float & _arg)
  {
    this->position_rad = _arg;
    return *this;
  }
  Type & set__velocity_rps(
    const float & _arg)
  {
    this->velocity_rps = _arg;
    return *this;
  }
  Type & set__acceleration_rpss(
    const float & _arg)
  {
    this->acceleration_rpss = _arg;
    return *this;
  }
  Type & set__rx_sequence_count(
    const uint8_t & _arg)
  {
    this->rx_sequence_count = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::msg::EncoderCollectorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::msg::EncoderCollectorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::msg::EncoderCollectorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::msg::EncoderCollectorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::EncoderCollectorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::EncoderCollectorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::EncoderCollectorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::EncoderCollectorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::msg::EncoderCollectorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::msg::EncoderCollectorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__msg__EncoderCollectorState
    std::shared_ptr<astribot_msgs::msg::EncoderCollectorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__msg__EncoderCollectorState
    std::shared_ptr<astribot_msgs::msg::EncoderCollectorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EncoderCollectorState_ & other) const
  {
    if (this->position_rad != other.position_rad) {
      return false;
    }
    if (this->velocity_rps != other.velocity_rps) {
      return false;
    }
    if (this->acceleration_rpss != other.acceleration_rpss) {
      return false;
    }
    if (this->rx_sequence_count != other.rx_sequence_count) {
      return false;
    }
    return true;
  }
  bool operator!=(const EncoderCollectorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EncoderCollectorState_

// alias to use template instance with default allocator
using EncoderCollectorState =
  astribot_msgs::msg::EncoderCollectorState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ENCODER_COLLECTOR_STATE__STRUCT_HPP_
