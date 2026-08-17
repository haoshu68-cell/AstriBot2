// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:msg/StorageRequest.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__STORAGE_REQUEST__STRUCT_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__STORAGE_REQUEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__astribot_msgs__msg__StorageRequest __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__msg__StorageRequest __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct StorageRequest_
{
  using Type = StorageRequest_<ContainerAllocator>;

  explicit StorageRequest_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->uuid = "";
      this->storage_type = 0;
      this->tag_content = "";
      this->topic_list = "";
    }
  }

  explicit StorageRequest_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : uuid(_alloc),
    tag_content(_alloc),
    topic_list(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->uuid = "";
      this->storage_type = 0;
      this->tag_content = "";
      this->topic_list = "";
    }
  }

  // field types and members
  using _uuid_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _uuid_type uuid;
  using _storage_type_type =
    uint8_t;
  _storage_type_type storage_type;
  using _tag_content_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _tag_content_type tag_content;
  using _topic_list_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _topic_list_type topic_list;

  // setters for named parameter idiom
  Type & set__uuid(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->uuid = _arg;
    return *this;
  }
  Type & set__storage_type(
    const uint8_t & _arg)
  {
    this->storage_type = _arg;
    return *this;
  }
  Type & set__tag_content(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->tag_content = _arg;
    return *this;
  }
  Type & set__topic_list(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->topic_list = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t TYPE_RECORD_MODE =
    0u;
  static constexpr uint8_t TYPE_START_RECORD =
    1u;
  static constexpr uint8_t TYPE_STOP_RECORD =
    2u;
  static constexpr uint8_t TYPE_SNAPSHOT_MODE =
    3u;
  static constexpr uint8_t TYPE_TRIGGER_SNAPSHOT =
    4u;
  static constexpr uint8_t TYPE_INACTIVE_MODE =
    5u;
  static constexpr uint8_t TYPE_SAVE_DATA =
    6u;
  static constexpr uint8_t TYPE_DELETE_DATA =
    7u;

  // pointer types
  using RawPtr =
    astribot_msgs::msg::StorageRequest_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::msg::StorageRequest_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::msg::StorageRequest_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::msg::StorageRequest_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::StorageRequest_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::StorageRequest_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::StorageRequest_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::StorageRequest_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::msg::StorageRequest_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::msg::StorageRequest_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__msg__StorageRequest
    std::shared_ptr<astribot_msgs::msg::StorageRequest_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__msg__StorageRequest
    std::shared_ptr<astribot_msgs::msg::StorageRequest_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StorageRequest_ & other) const
  {
    if (this->uuid != other.uuid) {
      return false;
    }
    if (this->storage_type != other.storage_type) {
      return false;
    }
    if (this->tag_content != other.tag_content) {
      return false;
    }
    if (this->topic_list != other.topic_list) {
      return false;
    }
    return true;
  }
  bool operator!=(const StorageRequest_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StorageRequest_

// alias to use template instance with default allocator
using StorageRequest =
  astribot_msgs::msg::StorageRequest_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StorageRequest_<ContainerAllocator>::TYPE_RECORD_MODE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StorageRequest_<ContainerAllocator>::TYPE_START_RECORD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StorageRequest_<ContainerAllocator>::TYPE_STOP_RECORD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StorageRequest_<ContainerAllocator>::TYPE_SNAPSHOT_MODE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StorageRequest_<ContainerAllocator>::TYPE_TRIGGER_SNAPSHOT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StorageRequest_<ContainerAllocator>::TYPE_INACTIVE_MODE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StorageRequest_<ContainerAllocator>::TYPE_SAVE_DATA;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StorageRequest_<ContainerAllocator>::TYPE_DELETE_DATA;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__STORAGE_REQUEST__STRUCT_HPP_
