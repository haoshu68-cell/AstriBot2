// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:msg/AstribotSerializedProtoMsg.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_SERIALIZED_PROTO_MSG__STRUCT_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_SERIALIZED_PROTO_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__astribot_msgs__msg__AstribotSerializedProtoMsg __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__msg__AstribotSerializedProtoMsg __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AstribotSerializedProtoMsg_
{
  using Type = AstribotSerializedProtoMsg_<ContainerAllocator>;

  explicit AstribotSerializedProtoMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit AstribotSerializedProtoMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::msg::AstribotSerializedProtoMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::msg::AstribotSerializedProtoMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::msg::AstribotSerializedProtoMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::msg::AstribotSerializedProtoMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::AstribotSerializedProtoMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::AstribotSerializedProtoMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::AstribotSerializedProtoMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::AstribotSerializedProtoMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::msg::AstribotSerializedProtoMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::msg::AstribotSerializedProtoMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__msg__AstribotSerializedProtoMsg
    std::shared_ptr<astribot_msgs::msg::AstribotSerializedProtoMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__msg__AstribotSerializedProtoMsg
    std::shared_ptr<astribot_msgs::msg::AstribotSerializedProtoMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AstribotSerializedProtoMsg_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const AstribotSerializedProtoMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AstribotSerializedProtoMsg_

// alias to use template instance with default allocator
using AstribotSerializedProtoMsg =
  astribot_msgs::msg::AstribotSerializedProtoMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_SERIALIZED_PROTO_MSG__STRUCT_HPP_
