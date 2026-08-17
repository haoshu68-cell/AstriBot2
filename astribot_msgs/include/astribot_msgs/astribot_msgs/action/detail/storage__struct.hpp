// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astribot_msgs:action/Storage.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__ACTION__DETAIL__STORAGE__STRUCT_HPP_
#define ASTRIBOT_MSGS__ACTION__DETAIL__STORAGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__astribot_msgs__action__Storage_Goal __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__action__Storage_Goal __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Storage_Goal_
{
  using Type = Storage_Goal_<ContainerAllocator>;

  explicit Storage_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->topic_list = "";
      this->storage_type = 0l;
      this->trigger_time = 0ll;
      this->archive_path = "";
    }
  }

  explicit Storage_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : topic_list(_alloc),
    archive_path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->topic_list = "";
      this->storage_type = 0l;
      this->trigger_time = 0ll;
      this->archive_path = "";
    }
  }

  // field types and members
  using _topic_list_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _topic_list_type topic_list;
  using _storage_type_type =
    int32_t;
  _storage_type_type storage_type;
  using _trigger_time_type =
    int64_t;
  _trigger_time_type trigger_time;
  using _archive_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _archive_path_type archive_path;

  // setters for named parameter idiom
  Type & set__topic_list(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->topic_list = _arg;
    return *this;
  }
  Type & set__storage_type(
    const int32_t & _arg)
  {
    this->storage_type = _arg;
    return *this;
  }
  Type & set__trigger_time(
    const int64_t & _arg)
  {
    this->trigger_time = _arg;
    return *this;
  }
  Type & set__archive_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->archive_path = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::action::Storage_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::action::Storage_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::action::Storage_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::action::Storage_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::action::Storage_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::action::Storage_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::action::Storage_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::action::Storage_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::action::Storage_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::action::Storage_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__action__Storage_Goal
    std::shared_ptr<astribot_msgs::action::Storage_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__action__Storage_Goal
    std::shared_ptr<astribot_msgs::action::Storage_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Storage_Goal_ & other) const
  {
    if (this->topic_list != other.topic_list) {
      return false;
    }
    if (this->storage_type != other.storage_type) {
      return false;
    }
    if (this->trigger_time != other.trigger_time) {
      return false;
    }
    if (this->archive_path != other.archive_path) {
      return false;
    }
    return true;
  }
  bool operator!=(const Storage_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Storage_Goal_

// alias to use template instance with default allocator
using Storage_Goal =
  astribot_msgs::action::Storage_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace astribot_msgs


#ifndef _WIN32
# define DEPRECATED__astribot_msgs__action__Storage_Result __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__action__Storage_Result __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Storage_Result_
{
  using Type = Storage_Result_<ContainerAllocator>;

  explicit Storage_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dir_path = "";
      this->miss_topic_list = "";
    }
  }

  explicit Storage_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : dir_path(_alloc),
    miss_topic_list(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dir_path = "";
      this->miss_topic_list = "";
    }
  }

  // field types and members
  using _dir_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _dir_path_type dir_path;
  using _miss_topic_list_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _miss_topic_list_type miss_topic_list;

  // setters for named parameter idiom
  Type & set__dir_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->dir_path = _arg;
    return *this;
  }
  Type & set__miss_topic_list(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->miss_topic_list = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::action::Storage_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::action::Storage_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::action::Storage_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::action::Storage_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::action::Storage_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::action::Storage_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::action::Storage_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::action::Storage_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::action::Storage_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::action::Storage_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__action__Storage_Result
    std::shared_ptr<astribot_msgs::action::Storage_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__action__Storage_Result
    std::shared_ptr<astribot_msgs::action::Storage_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Storage_Result_ & other) const
  {
    if (this->dir_path != other.dir_path) {
      return false;
    }
    if (this->miss_topic_list != other.miss_topic_list) {
      return false;
    }
    return true;
  }
  bool operator!=(const Storage_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Storage_Result_

// alias to use template instance with default allocator
using Storage_Result =
  astribot_msgs::action::Storage_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace astribot_msgs


#ifndef _WIN32
# define DEPRECATED__astribot_msgs__action__Storage_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__action__Storage_Feedback __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Storage_Feedback_
{
  using Type = Storage_Feedback_<ContainerAllocator>;

  explicit Storage_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->seconds = 0l;
      this->disk_space = 0l;
      this->miss_topic_list = "";
      this->status = 0ul;
    }
  }

  explicit Storage_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : miss_topic_list(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->seconds = 0l;
      this->disk_space = 0l;
      this->miss_topic_list = "";
      this->status = 0ul;
    }
  }

  // field types and members
  using _seconds_type =
    int32_t;
  _seconds_type seconds;
  using _disk_space_type =
    int32_t;
  _disk_space_type disk_space;
  using _miss_topic_list_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _miss_topic_list_type miss_topic_list;
  using _status_type =
    uint32_t;
  _status_type status;

  // setters for named parameter idiom
  Type & set__seconds(
    const int32_t & _arg)
  {
    this->seconds = _arg;
    return *this;
  }
  Type & set__disk_space(
    const int32_t & _arg)
  {
    this->disk_space = _arg;
    return *this;
  }
  Type & set__miss_topic_list(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->miss_topic_list = _arg;
    return *this;
  }
  Type & set__status(
    const uint32_t & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint32_t STATUS_RECORDING =
    3u;
  static constexpr uint32_t STATUS_UPDATE =
    4u;
  static constexpr uint32_t STATUS_STOP =
    5u;

  // pointer types
  using RawPtr =
    astribot_msgs::action::Storage_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::action::Storage_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::action::Storage_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::action::Storage_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::action::Storage_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::action::Storage_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::action::Storage_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::action::Storage_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::action::Storage_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::action::Storage_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__action__Storage_Feedback
    std::shared_ptr<astribot_msgs::action::Storage_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__action__Storage_Feedback
    std::shared_ptr<astribot_msgs::action::Storage_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Storage_Feedback_ & other) const
  {
    if (this->seconds != other.seconds) {
      return false;
    }
    if (this->disk_space != other.disk_space) {
      return false;
    }
    if (this->miss_topic_list != other.miss_topic_list) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const Storage_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Storage_Feedback_

// alias to use template instance with default allocator
using Storage_Feedback =
  astribot_msgs::action::Storage_Feedback_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t Storage_Feedback_<ContainerAllocator>::STATUS_RECORDING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t Storage_Feedback_<ContainerAllocator>::STATUS_UPDATE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t Storage_Feedback_<ContainerAllocator>::STATUS_STOP;
#endif  // __cplusplus < 201703L

}  // namespace action

}  // namespace astribot_msgs


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "astribot_msgs/action/detail/storage__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__astribot_msgs__action__Storage_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__action__Storage_SendGoal_Request __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Storage_SendGoal_Request_
{
  using Type = Storage_SendGoal_Request_<ContainerAllocator>;

  explicit Storage_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit Storage_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    astribot_msgs::action::Storage_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const astribot_msgs::action::Storage_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::action::Storage_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::action::Storage_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::action::Storage_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::action::Storage_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::action::Storage_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::action::Storage_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::action::Storage_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::action::Storage_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::action::Storage_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::action::Storage_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__action__Storage_SendGoal_Request
    std::shared_ptr<astribot_msgs::action::Storage_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__action__Storage_SendGoal_Request
    std::shared_ptr<astribot_msgs::action::Storage_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Storage_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const Storage_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Storage_SendGoal_Request_

// alias to use template instance with default allocator
using Storage_SendGoal_Request =
  astribot_msgs::action::Storage_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace astribot_msgs


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__astribot_msgs__action__Storage_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__action__Storage_SendGoal_Response __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Storage_SendGoal_Response_
{
  using Type = Storage_SendGoal_Response_<ContainerAllocator>;

  explicit Storage_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit Storage_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::action::Storage_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::action::Storage_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::action::Storage_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::action::Storage_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::action::Storage_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::action::Storage_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::action::Storage_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::action::Storage_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::action::Storage_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::action::Storage_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__action__Storage_SendGoal_Response
    std::shared_ptr<astribot_msgs::action::Storage_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__action__Storage_SendGoal_Response
    std::shared_ptr<astribot_msgs::action::Storage_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Storage_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const Storage_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Storage_SendGoal_Response_

// alias to use template instance with default allocator
using Storage_SendGoal_Response =
  astribot_msgs::action::Storage_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace astribot_msgs

namespace astribot_msgs
{

namespace action
{

struct Storage_SendGoal
{
  using Request = astribot_msgs::action::Storage_SendGoal_Request;
  using Response = astribot_msgs::action::Storage_SendGoal_Response;
};

}  // namespace action

}  // namespace astribot_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__astribot_msgs__action__Storage_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__action__Storage_GetResult_Request __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Storage_GetResult_Request_
{
  using Type = Storage_GetResult_Request_<ContainerAllocator>;

  explicit Storage_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit Storage_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::action::Storage_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::action::Storage_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::action::Storage_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::action::Storage_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::action::Storage_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::action::Storage_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::action::Storage_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::action::Storage_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::action::Storage_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::action::Storage_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__action__Storage_GetResult_Request
    std::shared_ptr<astribot_msgs::action::Storage_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__action__Storage_GetResult_Request
    std::shared_ptr<astribot_msgs::action::Storage_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Storage_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Storage_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Storage_GetResult_Request_

// alias to use template instance with default allocator
using Storage_GetResult_Request =
  astribot_msgs::action::Storage_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace astribot_msgs


// Include directives for member types
// Member 'result'
// already included above
// #include "astribot_msgs/action/detail/storage__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__astribot_msgs__action__Storage_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__action__Storage_GetResult_Response __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Storage_GetResult_Response_
{
  using Type = Storage_GetResult_Response_<ContainerAllocator>;

  explicit Storage_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit Storage_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    astribot_msgs::action::Storage_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const astribot_msgs::action::Storage_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::action::Storage_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::action::Storage_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::action::Storage_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::action::Storage_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::action::Storage_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::action::Storage_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::action::Storage_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::action::Storage_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::action::Storage_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::action::Storage_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__action__Storage_GetResult_Response
    std::shared_ptr<astribot_msgs::action::Storage_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__action__Storage_GetResult_Response
    std::shared_ptr<astribot_msgs::action::Storage_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Storage_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const Storage_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Storage_GetResult_Response_

// alias to use template instance with default allocator
using Storage_GetResult_Response =
  astribot_msgs::action::Storage_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace astribot_msgs

namespace astribot_msgs
{

namespace action
{

struct Storage_GetResult
{
  using Request = astribot_msgs::action::Storage_GetResult_Request;
  using Response = astribot_msgs::action::Storage_GetResult_Response;
};

}  // namespace action

}  // namespace astribot_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "astribot_msgs/action/detail/storage__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__astribot_msgs__action__Storage_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__astribot_msgs__action__Storage_FeedbackMessage __declspec(deprecated)
#endif

namespace astribot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Storage_FeedbackMessage_
{
  using Type = Storage_FeedbackMessage_<ContainerAllocator>;

  explicit Storage_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit Storage_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    astribot_msgs::action::Storage_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const astribot_msgs::action::Storage_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astribot_msgs::action::Storage_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const astribot_msgs::action::Storage_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astribot_msgs::action::Storage_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astribot_msgs::action::Storage_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::action::Storage_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::action::Storage_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astribot_msgs::action::Storage_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astribot_msgs::action::Storage_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astribot_msgs::action::Storage_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astribot_msgs::action::Storage_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astribot_msgs__action__Storage_FeedbackMessage
    std::shared_ptr<astribot_msgs::action::Storage_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astribot_msgs__action__Storage_FeedbackMessage
    std::shared_ptr<astribot_msgs::action::Storage_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Storage_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const Storage_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Storage_FeedbackMessage_

// alias to use template instance with default allocator
using Storage_FeedbackMessage =
  astribot_msgs::action::Storage_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace astribot_msgs

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace astribot_msgs
{

namespace action
{

struct Storage
{
  /// The goal message defined in the action definition.
  using Goal = astribot_msgs::action::Storage_Goal;
  /// The result message defined in the action definition.
  using Result = astribot_msgs::action::Storage_Result;
  /// The feedback message defined in the action definition.
  using Feedback = astribot_msgs::action::Storage_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = astribot_msgs::action::Storage_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = astribot_msgs::action::Storage_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = astribot_msgs::action::Storage_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct Storage Storage;

}  // namespace action

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__ACTION__DETAIL__STORAGE__STRUCT_HPP_
