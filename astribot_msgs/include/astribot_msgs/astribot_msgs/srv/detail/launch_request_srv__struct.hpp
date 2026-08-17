// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:srv/LaunchRequestSrv.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__LAUNCH_REQUEST_SRV__STRUCT_HPP_
#define ASTRIBOT_MSGS__SRV__DETAIL__LAUNCH_REQUEST_SRV__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'request'
#include "astribot_msgs/msg/detail/launch_request__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__astribot_msgs__srv__LaunchRequestSrv_Request __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__srv__LaunchRequestSrv_Request __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LaunchRequestSrv_Request_
{
  using Type = LaunchRequestSrv_Request_<ContainerAllocator>;

  explicit LaunchRequestSrv_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : request(_init)
  {
    (void)_init;
  }

  explicit LaunchRequestSrv_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : request(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _request_type =
    astribot_msgs::msg::LaunchRequest_<ContainerAllocator>;
  _request_type request;

  // setters for named parameter idiom
  Type & set__request(
    const astribot_msgs::msg::LaunchRequest_<ContainerAllocator> & _arg)
  {
    this->request = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::srv::LaunchRequestSrv_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::srv::LaunchRequestSrv_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::srv::LaunchRequestSrv_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::srv::LaunchRequestSrv_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::srv::LaunchRequestSrv_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::srv::LaunchRequestSrv_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::srv::LaunchRequestSrv_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::srv::LaunchRequestSrv_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::srv::LaunchRequestSrv_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::srv::LaunchRequestSrv_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__srv__LaunchRequestSrv_Request
    std::shared_ptr<astribot_msgs::srv::LaunchRequestSrv_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__srv__LaunchRequestSrv_Request
    std::shared_ptr<astribot_msgs::srv::LaunchRequestSrv_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LaunchRequestSrv_Request_ & other) const
  {
    if (this->request != other.request) {
      return false;
    }
    return true;
  }
  bool operator!=(const LaunchRequestSrv_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LaunchRequestSrv_Request_

// alias to use template instance with default allocator
using LaunchRequestSrv_Request =
  astribot_msgs::srv::LaunchRequestSrv_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace astribot_msgs


#ifndef _WIN32
# define DEPRECATED__astribot_msgs__srv__LaunchRequestSrv_Response __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__srv__LaunchRequestSrv_Response __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LaunchRequestSrv_Response_
{
  using Type = LaunchRequestSrv_Response_<ContainerAllocator>;

  explicit LaunchRequestSrv_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response = false;
      this->message = "";
    }
  }

  explicit LaunchRequestSrv_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response = false;
      this->message = "";
    }
  }

  // field types and members
  using _response_type =
    bool;
  _response_type response;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__response(
    const bool & _arg)
  {
    this->response = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::srv::LaunchRequestSrv_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::srv::LaunchRequestSrv_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::srv::LaunchRequestSrv_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::srv::LaunchRequestSrv_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::srv::LaunchRequestSrv_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::srv::LaunchRequestSrv_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::srv::LaunchRequestSrv_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::srv::LaunchRequestSrv_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::srv::LaunchRequestSrv_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::srv::LaunchRequestSrv_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__srv__LaunchRequestSrv_Response
    std::shared_ptr<astribot_msgs::srv::LaunchRequestSrv_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__srv__LaunchRequestSrv_Response
    std::shared_ptr<astribot_msgs::srv::LaunchRequestSrv_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LaunchRequestSrv_Response_ & other) const
  {
    if (this->response != other.response) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const LaunchRequestSrv_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LaunchRequestSrv_Response_

// alias to use template instance with default allocator
using LaunchRequestSrv_Response =
  astribot_msgs::srv::LaunchRequestSrv_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace astribot_msgs

namespace astribot_msgs
{

namespace srv
{

struct LaunchRequestSrv
{
  using Request = astribot_msgs::srv::LaunchRequestSrv_Request;
  using Response = astribot_msgs::srv::LaunchRequestSrv_Response;
};

}  // namespace srv

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__LAUNCH_REQUEST_SRV__STRUCT_HPP_
