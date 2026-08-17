// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:msg/AstribotControlCommand.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_CONTROL_COMMAND__STRUCT_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_CONTROL_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__astribot_msgs__msg__AstribotControlCommand __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__msg__AstribotControlCommand __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AstribotControlCommand_
{
  using Type = AstribotControlCommand_<ContainerAllocator>;

  explicit AstribotControlCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->control_way = "";
      this->frame = "";
      this->use_wbc = false;
      this->add_default_torso = false;
    }
  }

  explicit AstribotControlCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : control_way(_alloc),
    frame(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->control_way = "";
      this->frame = "";
      this->use_wbc = false;
      this->add_default_torso = false;
    }
  }

  // field types and members
  using _name_list_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _name_list_type name_list;
  using _dofs_list_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _dofs_list_type dofs_list;
  using _command_list_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _command_list_type command_list;
  using _control_way_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _control_way_type control_way;
  using _frame_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _frame_type frame;
  using _use_wbc_type =
    bool;
  _use_wbc_type use_wbc;
  using _add_default_torso_type =
    bool;
  _add_default_torso_type add_default_torso;

  // setters for named parameter idiom
  Type & set__name_list(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->name_list = _arg;
    return *this;
  }
  Type & set__dofs_list(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->dofs_list = _arg;
    return *this;
  }
  Type & set__command_list(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->command_list = _arg;
    return *this;
  }
  Type & set__control_way(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->control_way = _arg;
    return *this;
  }
  Type & set__frame(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->frame = _arg;
    return *this;
  }
  Type & set__use_wbc(
    const bool & _arg)
  {
    this->use_wbc = _arg;
    return *this;
  }
  Type & set__add_default_torso(
    const bool & _arg)
  {
    this->add_default_torso = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::msg::AstribotControlCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::msg::AstribotControlCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::msg::AstribotControlCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::msg::AstribotControlCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::AstribotControlCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::AstribotControlCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::msg::AstribotControlCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::msg::AstribotControlCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::msg::AstribotControlCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::msg::AstribotControlCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__msg__AstribotControlCommand
    std::shared_ptr<astribot_msgs::msg::AstribotControlCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__msg__AstribotControlCommand
    std::shared_ptr<astribot_msgs::msg::AstribotControlCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AstribotControlCommand_ & other) const
  {
    if (this->name_list != other.name_list) {
      return false;
    }
    if (this->dofs_list != other.dofs_list) {
      return false;
    }
    if (this->command_list != other.command_list) {
      return false;
    }
    if (this->control_way != other.control_way) {
      return false;
    }
    if (this->frame != other.frame) {
      return false;
    }
    if (this->use_wbc != other.use_wbc) {
      return false;
    }
    if (this->add_default_torso != other.add_default_torso) {
      return false;
    }
    return true;
  }
  bool operator!=(const AstribotControlCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AstribotControlCommand_

// alias to use template instance with default allocator
using AstribotControlCommand =
  astribot_msgs::msg::AstribotControlCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_CONTROL_COMMAND__STRUCT_HPP_
