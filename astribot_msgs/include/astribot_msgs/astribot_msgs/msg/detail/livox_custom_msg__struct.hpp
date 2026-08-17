// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:msg/LivoxCustomMsg.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__LIVOX_CUSTOM_MSG__STRUCT_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__LIVOX_CUSTOM_MSG__STRUCT_HPP_

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
// Member 'points'
#include "astribot_msgs/msg/detail/custom_point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__astribot_msgs__msg__LivoxCustomMsg __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__msg__LivoxCustomMsg __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LivoxCustomMsg_
{
  using Type = LivoxCustomMsg_<ContainerAllocator>;

  explicit LivoxCustomMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timebase = 0ull;
      this->point_num = 0ul;
      this->lidar_id = 0;
      std::fill<typename std::array<uint8_t, 3>::iterator, uint8_t>(this->rsvd.begin(), this->rsvd.end(), 0);
    }
  }

  explicit LivoxCustomMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    rsvd(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timebase = 0ull;
      this->point_num = 0ul;
      this->lidar_id = 0;
      std::fill<typename std::array<uint8_t, 3>::iterator, uint8_t>(this->rsvd.begin(), this->rsvd.end(), 0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _timebase_type =
    uint64_t;
  _timebase_type timebase;
  using _point_num_type =
    uint32_t;
  _point_num_type point_num;
  using _lidar_id_type =
    uint8_t;
  _lidar_id_type lidar_id;
  using _rsvd_type =
    std::array<uint8_t, 3>;
  _rsvd_type rsvd;
  using _points_type =
    std::vector<astribot_msgs::msg::CustomPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<astribot_msgs::msg::CustomPoint_<ContainerAllocator>>>;
  _points_type points;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__timebase(
    const uint64_t & _arg)
  {
    this->timebase = _arg;
    return *this;
  }
  Type & set__point_num(
    const uint32_t & _arg)
  {
    this->point_num = _arg;
    return *this;
  }
  Type & set__lidar_id(
    const uint8_t & _arg)
  {
    this->lidar_id = _arg;
    return *this;
  }
  Type & set__rsvd(
    const std::array<uint8_t, 3> & _arg)
  {
    this->rsvd = _arg;
    return *this;
  }
  Type & set__points(
    const std::vector<astribot_msgs::msg::CustomPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<astribot_msgs::msg::CustomPoint_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::msg::LivoxCustomMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::msg::LivoxCustomMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::msg::LivoxCustomMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::msg::LivoxCustomMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::LivoxCustomMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::LivoxCustomMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::LivoxCustomMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::LivoxCustomMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::msg::LivoxCustomMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::msg::LivoxCustomMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__msg__LivoxCustomMsg
    std::shared_ptr<astribot_msgs::msg::LivoxCustomMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__msg__LivoxCustomMsg
    std::shared_ptr<astribot_msgs::msg::LivoxCustomMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LivoxCustomMsg_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->timebase != other.timebase) {
      return false;
    }
    if (this->point_num != other.point_num) {
      return false;
    }
    if (this->lidar_id != other.lidar_id) {
      return false;
    }
    if (this->rsvd != other.rsvd) {
      return false;
    }
    if (this->points != other.points) {
      return false;
    }
    return true;
  }
  bool operator!=(const LivoxCustomMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LivoxCustomMsg_

// alias to use template instance with default allocator
using LivoxCustomMsg =
  astribot_msgs::msg::LivoxCustomMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__LIVOX_CUSTOM_MSG__STRUCT_HPP_
