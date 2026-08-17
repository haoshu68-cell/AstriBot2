// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:srv/RawRequest.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__RAW_REQUEST__STRUCT_HPP_
#define ASTRIBOT_MSGS__SRV__DETAIL__RAW_REQUEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__astribot_msgs__srv__RawRequest_Request __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__srv__RawRequest_Request __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RawRequest_Request_
{
  using Type = RawRequest_Request_<ContainerAllocator>;

  explicit RawRequest_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request = "";
    }
  }

  explicit RawRequest_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : request(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request = "";
    }
  }

  // field types and members
  using _request_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _request_type request;

  // setters for named parameter idiom
  Type & set__request(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->request = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::srv::RawRequest_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::srv::RawRequest_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::srv::RawRequest_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::srv::RawRequest_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::srv::RawRequest_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::srv::RawRequest_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::srv::RawRequest_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::srv::RawRequest_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::srv::RawRequest_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::srv::RawRequest_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__srv__RawRequest_Request
    std::shared_ptr<astribot_msgs::srv::RawRequest_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__srv__RawRequest_Request
    std::shared_ptr<astribot_msgs::srv::RawRequest_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RawRequest_Request_ & other) const
  {
    if (this->request != other.request) {
      return false;
    }
    return true;
  }
  bool operator!=(const RawRequest_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RawRequest_Request_

// alias to use template instance with default allocator
using RawRequest_Request =
  astribot_msgs::srv::RawRequest_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace astribot_msgs


#ifndef _WIN32
# define DEPRECATED__astribot_msgs__srv__RawRequest_Response __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__srv__RawRequest_Response __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RawRequest_Response_
{
  using Type = RawRequest_Response_<ContainerAllocator>;

  explicit RawRequest_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response = "";
    }
  }

  explicit RawRequest_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : response(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response = "";
    }
  }

  // field types and members
  using _response_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__response(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::srv::RawRequest_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::srv::RawRequest_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::srv::RawRequest_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::srv::RawRequest_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::srv::RawRequest_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::srv::RawRequest_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::srv::RawRequest_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::srv::RawRequest_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::srv::RawRequest_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::srv::RawRequest_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__srv__RawRequest_Response
    std::shared_ptr<astribot_msgs::srv::RawRequest_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__srv__RawRequest_Response
    std::shared_ptr<astribot_msgs::srv::RawRequest_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RawRequest_Response_ & other) const
  {
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const RawRequest_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RawRequest_Response_

// alias to use template instance with default allocator
using RawRequest_Response =
  astribot_msgs::srv::RawRequest_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace astribot_msgs

namespace astribot_msgs
{

namespace srv
{

struct RawRequest
{
  using Request = astribot_msgs::srv::RawRequest_Request;
  using Response = astribot_msgs::srv::RawRequest_Response;
};

}  // namespace srv

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__RAW_REQUEST__STRUCT_HPP_
