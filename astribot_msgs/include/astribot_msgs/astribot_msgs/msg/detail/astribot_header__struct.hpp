// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:msg/AstribotHeader.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEADER__STRUCT_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEADER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__astribot_msgs__msg__AstribotHeader __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__msg__AstribotHeader __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AstribotHeader_
{
  using Type = AstribotHeader_<ContainerAllocator>;

  explicit AstribotHeader_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->seq = 0ull;
      this->time_meas = 0ull;
      this->time_pub = 0ull;
    }
  }

  explicit AstribotHeader_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->seq = 0ull;
      this->time_meas = 0ull;
      this->time_pub = 0ull;
    }
  }

  // field types and members
  using _seq_type =
    uint64_t;
  _seq_type seq;
  using _time_meas_type =
    uint64_t;
  _time_meas_type time_meas;
  using _time_pub_type =
    uint64_t;
  _time_pub_type time_pub;

  // setters for named parameter idiom
  Type & set__seq(
    const uint64_t & _arg)
  {
    this->seq = _arg;
    return *this;
  }
  Type & set__time_meas(
    const uint64_t & _arg)
  {
    this->time_meas = _arg;
    return *this;
  }
  Type & set__time_pub(
    const uint64_t & _arg)
  {
    this->time_pub = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::msg::AstribotHeader_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::msg::AstribotHeader_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::msg::AstribotHeader_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::msg::AstribotHeader_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::AstribotHeader_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::AstribotHeader_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::AstribotHeader_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::AstribotHeader_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::msg::AstribotHeader_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::msg::AstribotHeader_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__msg__AstribotHeader
    std::shared_ptr<astribot_msgs::msg::AstribotHeader_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__msg__AstribotHeader
    std::shared_ptr<astribot_msgs::msg::AstribotHeader_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AstribotHeader_ & other) const
  {
    if (this->seq != other.seq) {
      return false;
    }
    if (this->time_meas != other.time_meas) {
      return false;
    }
    if (this->time_pub != other.time_pub) {
      return false;
    }
    return true;
  }
  bool operator!=(const AstribotHeader_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AstribotHeader_

// alias to use template instance with default allocator
using AstribotHeader =
  astribot_msgs::msg::AstribotHeader_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEADER__STRUCT_HPP_
