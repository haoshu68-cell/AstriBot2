// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:msg/AstribotHeartbeat.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEARTBEAT__STRUCT_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEARTBEAT__STRUCT_HPP_

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
// Member 'node_name'
#include "std_msgs/msg/detail/string__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__astribot_msgs__msg__AstribotHeartbeat __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__msg__AstribotHeartbeat __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AstribotHeartbeat_
{
  using Type = AstribotHeartbeat_<ContainerAllocator>;

  explicit AstribotHeartbeat_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    node_name(_init)
  {
    (void)_init;
  }

  explicit AstribotHeartbeat_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    node_name(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    astribot_msgs::msg::AstribotHeader_<ContainerAllocator>;
  _header_type header;
  using _node_name_type =
    std_msgs::msg::String_<ContainerAllocator>;
  _node_name_type node_name;

  // setters for named parameter idiom
  Type & set__header(
    const astribot_msgs::msg::AstribotHeader_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__node_name(
    const std_msgs::msg::String_<ContainerAllocator> & _arg)
  {
    this->node_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::msg::AstribotHeartbeat_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::msg::AstribotHeartbeat_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::msg::AstribotHeartbeat_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::msg::AstribotHeartbeat_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::AstribotHeartbeat_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::AstribotHeartbeat_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::AstribotHeartbeat_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::AstribotHeartbeat_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::msg::AstribotHeartbeat_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::msg::AstribotHeartbeat_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__msg__AstribotHeartbeat
    std::shared_ptr<astribot_msgs::msg::AstribotHeartbeat_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__msg__AstribotHeartbeat
    std::shared_ptr<astribot_msgs::msg::AstribotHeartbeat_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AstribotHeartbeat_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->node_name != other.node_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const AstribotHeartbeat_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AstribotHeartbeat_

// alias to use template instance with default allocator
using AstribotHeartbeat =
  astribot_msgs::msg::AstribotHeartbeat_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEARTBEAT__STRUCT_HPP_
