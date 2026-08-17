// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:srv/DoubleArrayRequest.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__DOUBLE_ARRAY_REQUEST__STRUCT_HPP_
#define ASTRIBOT_MSGS__SRV__DETAIL__DOUBLE_ARRAY_REQUEST__STRUCT_HPP_

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
# define DEPRECATED__astribot_msgs__srv__DoubleArrayRequest_Request __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__srv__DoubleArrayRequest_Request __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DoubleArrayRequest_Request_
{
  using Type = DoubleArrayRequest_Request_<ContainerAllocator>;

  explicit DoubleArrayRequest_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit DoubleArrayRequest_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    astribot_msgs::srv::DoubleArrayRequest_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::srv::DoubleArrayRequest_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::srv::DoubleArrayRequest_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::srv::DoubleArrayRequest_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::srv::DoubleArrayRequest_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::srv::DoubleArrayRequest_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::srv::DoubleArrayRequest_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::srv::DoubleArrayRequest_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::srv::DoubleArrayRequest_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::srv::DoubleArrayRequest_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__srv__DoubleArrayRequest_Request
    std::shared_ptr<astribot_msgs::srv::DoubleArrayRequest_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__srv__DoubleArrayRequest_Request
    std::shared_ptr<astribot_msgs::srv::DoubleArrayRequest_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DoubleArrayRequest_Request_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const DoubleArrayRequest_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DoubleArrayRequest_Request_

// alias to use template instance with default allocator
using DoubleArrayRequest_Request =
  astribot_msgs::srv::DoubleArrayRequest_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace astribot_msgs


// Include directives for member types
// Member 'header'
// already included above
// #include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__astribot_msgs__srv__DoubleArrayRequest_Response __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__srv__DoubleArrayRequest_Response __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DoubleArrayRequest_Response_
{
  using Type = DoubleArrayRequest_Response_<ContainerAllocator>;

  explicit DoubleArrayRequest_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit DoubleArrayRequest_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    astribot_msgs::srv::DoubleArrayRequest_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::srv::DoubleArrayRequest_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::srv::DoubleArrayRequest_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::srv::DoubleArrayRequest_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::srv::DoubleArrayRequest_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::srv::DoubleArrayRequest_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::srv::DoubleArrayRequest_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::srv::DoubleArrayRequest_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::srv::DoubleArrayRequest_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::srv::DoubleArrayRequest_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__srv__DoubleArrayRequest_Response
    std::shared_ptr<astribot_msgs::srv::DoubleArrayRequest_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__srv__DoubleArrayRequest_Response
    std::shared_ptr<astribot_msgs::srv::DoubleArrayRequest_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DoubleArrayRequest_Response_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const DoubleArrayRequest_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DoubleArrayRequest_Response_

// alias to use template instance with default allocator
using DoubleArrayRequest_Response =
  astribot_msgs::srv::DoubleArrayRequest_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace astribot_msgs

namespace astribot_msgs
{

namespace srv
{

struct DoubleArrayRequest
{
  using Request = astribot_msgs::srv::DoubleArrayRequest_Request;
  using Response = astribot_msgs::srv::DoubleArrayRequest_Response;
};

}  // namespace srv

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__DOUBLE_ARRAY_REQUEST__STRUCT_HPP_
