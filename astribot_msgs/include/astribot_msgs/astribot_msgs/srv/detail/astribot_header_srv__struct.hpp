// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:srv/AstribotHeaderSrv.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__ASTRIBOT_HEADER_SRV__STRUCT_HPP_
#define ASTRIBOT_MSGS__SRV__DETAIL__ASTRIBOT_HEADER_SRV__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__astribot_msgs__srv__AstribotHeaderSrv_Request __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__srv__AstribotHeaderSrv_Request __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct AstribotHeaderSrv_Request_
{
  using Type = AstribotHeaderSrv_Request_<ContainerAllocator>;

  explicit AstribotHeaderSrv_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit AstribotHeaderSrv_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    astribot_msgs::msg::AstribotHeader_<ContainerAllocator>;
  _header_type header;

  // setters for named parameter idiom
  Type & set__header(
    const astribot_msgs::msg::AstribotHeader_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::srv::AstribotHeaderSrv_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::srv::AstribotHeaderSrv_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::srv::AstribotHeaderSrv_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::srv::AstribotHeaderSrv_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::srv::AstribotHeaderSrv_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::srv::AstribotHeaderSrv_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::srv::AstribotHeaderSrv_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::srv::AstribotHeaderSrv_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::srv::AstribotHeaderSrv_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::srv::AstribotHeaderSrv_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__srv__AstribotHeaderSrv_Request
    std::shared_ptr<astribot_msgs::srv::AstribotHeaderSrv_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__srv__AstribotHeaderSrv_Request
    std::shared_ptr<astribot_msgs::srv::AstribotHeaderSrv_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AstribotHeaderSrv_Request_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    return true;
  }
  bool operator!=(const AstribotHeaderSrv_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AstribotHeaderSrv_Request_

// alias to use template instance with default allocator
using AstribotHeaderSrv_Request =
  astribot_msgs::srv::AstribotHeaderSrv_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace astribot_msgs


// Include directives for member types
// Member 'header'
// already included above
// #include "astribot_msgs/msg/detail/astribot_header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__astribot_msgs__srv__AstribotHeaderSrv_Response __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__srv__AstribotHeaderSrv_Response __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct AstribotHeaderSrv_Response_
{
  using Type = AstribotHeaderSrv_Response_<ContainerAllocator>;

  explicit AstribotHeaderSrv_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit AstribotHeaderSrv_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    astribot_msgs::msg::AstribotHeader_<ContainerAllocator>;
  _header_type header;

  // setters for named parameter idiom
  Type & set__header(
    const astribot_msgs::msg::AstribotHeader_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::srv::AstribotHeaderSrv_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::srv::AstribotHeaderSrv_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::srv::AstribotHeaderSrv_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::srv::AstribotHeaderSrv_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::srv::AstribotHeaderSrv_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::srv::AstribotHeaderSrv_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::srv::AstribotHeaderSrv_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::srv::AstribotHeaderSrv_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::srv::AstribotHeaderSrv_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::srv::AstribotHeaderSrv_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__srv__AstribotHeaderSrv_Response
    std::shared_ptr<astribot_msgs::srv::AstribotHeaderSrv_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__srv__AstribotHeaderSrv_Response
    std::shared_ptr<astribot_msgs::srv::AstribotHeaderSrv_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AstribotHeaderSrv_Response_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    return true;
  }
  bool operator!=(const AstribotHeaderSrv_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AstribotHeaderSrv_Response_

// alias to use template instance with default allocator
using AstribotHeaderSrv_Response =
  astribot_msgs::srv::AstribotHeaderSrv_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace astribot_msgs

namespace astribot_msgs
{

namespace srv
{

struct AstribotHeaderSrv
{
  using Request = astribot_msgs::srv::AstribotHeaderSrv_Request;
  using Response = astribot_msgs::srv::AstribotHeaderSrv_Response;
};

}  // namespace srv

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__ASTRIBOT_HEADER_SRV__STRUCT_HPP_
