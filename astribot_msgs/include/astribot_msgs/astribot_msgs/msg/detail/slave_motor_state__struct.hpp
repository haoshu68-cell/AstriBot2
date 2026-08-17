// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:msg/SlaveMotorState.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_STATE__STRUCT_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_STATE__STRUCT_HPP_

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
// Member 'slave_motor_state'
#include "astribot_msgs/msg/detail/motor_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__astribot_msgs__msg__SlaveMotorState __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__msg__SlaveMotorState __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SlaveMotorState_
{
  using Type = SlaveMotorState_<ContainerAllocator>;

  explicit SlaveMotorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->slave_motor_state.fill(astribot_msgs::msg::MotorState_<ContainerAllocator>{_init});
    }
  }

  explicit SlaveMotorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    slave_motor_state(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->slave_motor_state.fill(astribot_msgs::msg::MotorState_<ContainerAllocator>{_alloc, _init});
    }
  }

  // field types and members
  using _header_type =
    astribot_msgs::msg::AstribotHeader_<ContainerAllocator>;
  _header_type header;
  using _slave_motor_state_type =
    std::array<astribot_msgs::msg::MotorState_<ContainerAllocator>, 3>;
  _slave_motor_state_type slave_motor_state;

  // setters for named parameter idiom
  Type & set__header(
    const astribot_msgs::msg::AstribotHeader_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__slave_motor_state(
    const std::array<astribot_msgs::msg::MotorState_<ContainerAllocator>, 3> & _arg)
  {
    this->slave_motor_state = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MAX_SLAVE_MOTOR =
    3u;

  // pointer types
  using RawPtr =
    astribot_msgs::msg::SlaveMotorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::msg::SlaveMotorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::msg::SlaveMotorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::msg::SlaveMotorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::SlaveMotorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::SlaveMotorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::SlaveMotorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::SlaveMotorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::msg::SlaveMotorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::msg::SlaveMotorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__msg__SlaveMotorState
    std::shared_ptr<astribot_msgs::msg::SlaveMotorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__msg__SlaveMotorState
    std::shared_ptr<astribot_msgs::msg::SlaveMotorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SlaveMotorState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->slave_motor_state != other.slave_motor_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const SlaveMotorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SlaveMotorState_

// alias to use template instance with default allocator
using SlaveMotorState =
  astribot_msgs::msg::SlaveMotorState_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SlaveMotorState_<ContainerAllocator>::MAX_SLAVE_MOTOR;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_STATE__STRUCT_HPP_
