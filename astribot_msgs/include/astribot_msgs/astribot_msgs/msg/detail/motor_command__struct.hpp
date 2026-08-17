// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__astribot_msgs__msg__MotorCommand __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__msg__MotorCommand __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorCommand_
{
  using Type = MotorCommand_<ContainerAllocator>;

  explicit MotorCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 9>::iterator, double>(this->kp.begin(), this->kp.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->kd.begin(), this->kd.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->p.begin(), this->p.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->v.begin(), this->v.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->t_ff.begin(), this->t_ff.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->t_limit.begin(), this->t_limit.end(), 0.0);
      std::fill<typename std::array<int8_t, 9>::iterator, int8_t>(this->break_relase.begin(), this->break_relase.end(), 0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->kd_slave.begin(), this->kd_slave.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->vel_slave.begin(), this->vel_slave.end(), 0.0);
    }
  }

  explicit MotorCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : kp(_alloc),
    kd(_alloc),
    p(_alloc),
    v(_alloc),
    t_ff(_alloc),
    t_limit(_alloc),
    break_relase(_alloc),
    kd_slave(_alloc),
    vel_slave(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 9>::iterator, double>(this->kp.begin(), this->kp.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->kd.begin(), this->kd.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->p.begin(), this->p.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->v.begin(), this->v.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->t_ff.begin(), this->t_ff.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->t_limit.begin(), this->t_limit.end(), 0.0);
      std::fill<typename std::array<int8_t, 9>::iterator, int8_t>(this->break_relase.begin(), this->break_relase.end(), 0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->kd_slave.begin(), this->kd_slave.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->vel_slave.begin(), this->vel_slave.end(), 0.0);
    }
  }

  // field types and members
  using _kp_type =
    std::array<double, 9>;
  _kp_type kp;
  using _kd_type =
    std::array<double, 9>;
  _kd_type kd;
  using _p_type =
    std::array<double, 9>;
  _p_type p;
  using _v_type =
    std::array<double, 9>;
  _v_type v;
  using _t_ff_type =
    std::array<double, 9>;
  _t_ff_type t_ff;
  using _t_limit_type =
    std::array<double, 9>;
  _t_limit_type t_limit;
  using _break_relase_type =
    std::array<int8_t, 9>;
  _break_relase_type break_relase;
  using _kd_slave_type =
    std::array<double, 9>;
  _kd_slave_type kd_slave;
  using _vel_slave_type =
    std::array<double, 9>;
  _vel_slave_type vel_slave;
  using _motor_id_list_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _motor_id_list_type motor_id_list;

  // setters for named parameter idiom
  Type & set__kp(
    const std::array<double, 9> & _arg)
  {
    this->kp = _arg;
    return *this;
  }
  Type & set__kd(
    const std::array<double, 9> & _arg)
  {
    this->kd = _arg;
    return *this;
  }
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
  Type & set__t_ff(
    const std::array<double, 9> & _arg)
  {
    this->t_ff = _arg;
    return *this;
  }
  Type & set__t_limit(
    const std::array<double, 9> & _arg)
  {
    this->t_limit = _arg;
    return *this;
  }
  Type & set__break_relase(
    const std::array<int8_t, 9> & _arg)
  {
    this->break_relase = _arg;
    return *this;
  }
  Type & set__kd_slave(
    const std::array<double, 9> & _arg)
  {
    this->kd_slave = _arg;
    return *this;
  }
  Type & set__vel_slave(
    const std::array<double, 9> & _arg)
  {
    this->vel_slave = _arg;
    return *this;
  }
  Type & set__motor_id_list(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->motor_id_list = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MAX_MOTOR_NUM =
    9u;

  // pointer types
  using RawPtr =
    astribot_msgs::msg::MotorCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::msg::MotorCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::msg::MotorCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::msg::MotorCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::MotorCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::MotorCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::MotorCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::MotorCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::msg::MotorCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::msg::MotorCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__msg__MotorCommand
    std::shared_ptr<astribot_msgs::msg::MotorCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__msg__MotorCommand
    std::shared_ptr<astribot_msgs::msg::MotorCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorCommand_ & other) const
  {
    if (this->kp != other.kp) {
      return false;
    }
    if (this->kd != other.kd) {
      return false;
    }
    if (this->p != other.p) {
      return false;
    }
    if (this->v != other.v) {
      return false;
    }
    if (this->t_ff != other.t_ff) {
      return false;
    }
    if (this->t_limit != other.t_limit) {
      return false;
    }
    if (this->break_relase != other.break_relase) {
      return false;
    }
    if (this->kd_slave != other.kd_slave) {
      return false;
    }
    if (this->vel_slave != other.vel_slave) {
      return false;
    }
    if (this->motor_id_list != other.motor_id_list) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorCommand_

// alias to use template instance with default allocator
using MotorCommand =
  astribot_msgs::msg::MotorCommand_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MotorCommand_<ContainerAllocator>::MAX_MOTOR_NUM;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_HPP_
