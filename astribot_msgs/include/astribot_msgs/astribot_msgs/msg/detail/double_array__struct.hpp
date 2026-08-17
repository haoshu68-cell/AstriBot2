// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:msg/DoubleArray.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__DOUBLE_ARRAY__STRUCT_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__DOUBLE_ARRAY__STRUCT_HPP_

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
# define DEPRECATED__astribot_msgs__msg__DoubleArray __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__msg__DoubleArray __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DoubleArray_
{
  using Type = DoubleArray_<ContainerAllocator>;

  explicit DoubleArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit DoubleArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _data_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::msg::DoubleArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::msg::DoubleArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::msg::DoubleArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::msg::DoubleArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::DoubleArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::DoubleArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::DoubleArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::DoubleArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::msg::DoubleArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::msg::DoubleArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__msg__DoubleArray
    std::shared_ptr<astribot_msgs::msg::DoubleArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__msg__DoubleArray
    std::shared_ptr<astribot_msgs::msg::DoubleArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DoubleArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const DoubleArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DoubleArray_

// alias to use template instance with default allocator
using DoubleArray =
  astribot_msgs::msg::DoubleArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__DOUBLE_ARRAY__STRUCT_HPP_
