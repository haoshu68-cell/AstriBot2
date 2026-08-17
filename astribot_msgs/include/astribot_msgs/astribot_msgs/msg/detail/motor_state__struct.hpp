// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:msg/MotorState.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_STATE__STRUCT_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__astribot_msgs__msg__MotorState __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__msg__MotorState __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorState_
{
  using Type = MotorState_<ContainerAllocator>;

  explicit MotorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 9>::iterator, double>(this->p.begin(), this->p.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->v.begin(), this->v.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->c.begin(), this->c.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->vol.begin(), this->vol.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->acc.begin(), this->acc.end(), 0.0);
      std::fill<typename std::array<uint8_t, 9>::iterator, uint8_t>(this->can_count.begin(), this->can_count.end(), 0);
      std::fill<typename std::array<uint8_t, 9>::iterator, uint8_t>(this->can_count_last.begin(), this->can_count_last.end(), 0);
      std::fill<typename std::array<uint8_t, 9>::iterator, uint8_t>(this->can_error.begin(), this->can_error.end(), 0);
      std::fill<typename std::array<bool, 9>::iterator, bool>(this->j_s.begin(), this->j_s.end(), false);
      std::fill<typename std::array<double, 9>::iterator, double>(this->j_p.begin(), this->j_p.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->j_v.begin(), this->j_v.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->j_a.begin(), this->j_a.end(), 0.0);
    }
  }

  explicit MotorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : p(_alloc),
    v(_alloc),
    c(_alloc),
    vol(_alloc),
    acc(_alloc),
    can_count(_alloc),
    can_count_last(_alloc),
    can_error(_alloc),
    j_s(_alloc),
    j_p(_alloc),
    j_v(_alloc),
    j_a(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 9>::iterator, double>(this->p.begin(), this->p.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->v.begin(), this->v.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->c.begin(), this->c.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->vol.begin(), this->vol.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->acc.begin(), this->acc.end(), 0.0);
      std::fill<typename std::array<uint8_t, 9>::iterator, uint8_t>(this->can_count.begin(), this->can_count.end(), 0);
      std::fill<typename std::array<uint8_t, 9>::iterator, uint8_t>(this->can_count_last.begin(), this->can_count_last.end(), 0);
      std::fill<typename std::array<uint8_t, 9>::iterator, uint8_t>(this->can_error.begin(), this->can_error.end(), 0);
      std::fill<typename std::array<bool, 9>::iterator, bool>(this->j_s.begin(), this->j_s.end(), false);
      std::fill<typename std::array<double, 9>::iterator, double>(this->j_p.begin(), this->j_p.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->j_v.begin(), this->j_v.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->j_a.begin(), this->j_a.end(), 0.0);
    }
  }

  // field types and members
  using _p_type =
    std::array<double, 9>;
  _p_type p;
  using _v_type =
    std::array<double, 9>;
  _v_type v;
  using _c_type =
    std::array<double, 9>;
  _c_type c;
  using _vol_type =
    std::array<double, 9>;
  _vol_type vol;
  using _acc_type =
    std::array<double, 9>;
  _acc_type acc;
  using _can_count_type =
    std::array<uint8_t, 9>;
  _can_count_type can_count;
  using _can_count_last_type =
    std::array<uint8_t, 9>;
  _can_count_last_type can_count_last;
  using _can_error_type =
    std::array<uint8_t, 9>;
  _can_error_type can_error;
  using _j_s_type =
    std::array<bool, 9>;
  _j_s_type j_s;
  using _j_p_type =
    std::array<double, 9>;
  _j_p_type j_p;
  using _j_v_type =
    std::array<double, 9>;
  _j_v_type j_v;
  using _j_a_type =
    std::array<double, 9>;
  _j_a_type j_a;

  // setters for named parameter idiom
  Type & set__p(
    const std::array<double, 9> & _arg)
  {
    this->p = _arg;
    return *this;
  }
  Type & set__v(
    const std::array<double, 9> & _arg)
  {
    this->v = _arg;
    return *this;
  }
  Type & set__c(
    const std::array<double, 9> & _arg)
  {
    this->c = _arg;
    return *this;
  }
  Type & set__vol(
    const std::array<double, 9> & _arg)
  {
    this->vol = _arg;
    return *this;
  }
  Type & set__acc(
    const std::array<double, 9> & _arg)
  {
    this->acc = _arg;
    return *this;
  }
  Type & set__can_count(
    const std::array<uint8_t, 9> & _arg)
  {
    this->can_count = _arg;
    return *this;
  }
  Type & set__can_count_last(
    const std::array<uint8_t, 9> & _arg)
  {
    this->can_count_last = _arg;
    return *this;
  }
  Type & set__can_error(
    const std::array<uint8_t, 9> & _arg)
  {
    this->can_error = _arg;
    return *this;
  }
  Type & set__j_s(
    const std::array<bool, 9> & _arg)
  {
    this->j_s = _arg;
    return *this;
  }
  Type & set__j_p(
    const std::array<double, 9> & _arg)
  {
    this->j_p = _arg;
    return *this;
  }
  Type & set__j_v(
    const std::array<double, 9> & _arg)
  {
    this->j_v = _arg;
    return *this;
  }
  Type & set__j_a(
    const std::array<double, 9> & _arg)
  {
    this->j_a = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MAX_MOTOR_NUM =
    9u;

  // pointer types
  using RawPtr =
    astribot_msgs::msg::MotorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::msg::MotorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::msg::MotorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::msg::MotorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::MotorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::MotorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::MotorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::MotorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::msg::MotorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::msg::MotorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__msg__MotorState
    std::shared_ptr<astribot_msgs::msg::MotorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__msg__MotorState
    std::shared_ptr<astribot_msgs::msg::MotorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorState_ & other) const
  {
    if (this->p != other.p) {
      return false;
    }
    if (this->v != other.v) {
      return false;
    }
    if (this->c != other.c) {
      return false;
    }
    if (this->vol != other.vol) {
      return false;
    }
    if (this->acc != other.acc) {
      return false;
    }
    if (this->can_count != other.can_count) {
      return false;
    }
    if (this->can_count_last != other.can_count_last) {
      return false;
    }
    if (this->can_error != other.can_error) {
      return false;
    }
    if (this->j_s != other.j_s) {
      return false;
    }
    if (this->j_p != other.j_p) {
      return false;
    }
    if (this->j_v != other.j_v) {
      return false;
    }
    if (this->j_a != other.j_a) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorState_

// alias to use template instance with default allocator
using MotorState =
  astribot_msgs::msg::MotorState_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MotorState_<ContainerAllocator>::MAX_MOTOR_NUM;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_STATE__STRUCT_HPP_
