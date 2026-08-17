// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:action/Storage.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__ACTION__DETAIL__STORAGE__BUILDER_HPP_
#define ASTRIBOT_MSGS__ACTION__DETAIL__STORAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/action/detail/storage__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace action
{

namespace builder
{

class Init_Storage_Goal_archive_path
{
public:
  explicit Init_Storage_Goal_archive_path(::astribot_msgs::action::Storage_Goal & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::action::Storage_Goal archive_path(::astribot_msgs::action::Storage_Goal::_archive_path_type arg)
  {
    msg_.archive_path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::action::Storage_Goal msg_;
};

class Init_Storage_Goal_trigger_time
{
public:
  explicit Init_Storage_Goal_trigger_time(::astribot_msgs::action::Storage_Goal & msg)
  : msg_(msg)
  {}
  Init_Storage_Goal_archive_path trigger_time(::astribot_msgs::action::Storage_Goal::_trigger_time_type arg)
  {
    msg_.trigger_time = std::move(arg);
    return Init_Storage_Goal_archive_path(msg_);
  }

private:
  ::astribot_msgs::action::Storage_Goal msg_;
};

class Init_Storage_Goal_storage_type
{
public:
  explicit Init_Storage_Goal_storage_type(::astribot_msgs::action::Storage_Goal & msg)
  : msg_(msg)
  {}
  Init_Storage_Goal_trigger_time storage_type(::astribot_msgs::action::Storage_Goal::_storage_type_type arg)
  {
    msg_.storage_type = std::move(arg);
    return Init_Storage_Goal_trigger_time(msg_);
  }

private:
  ::astribot_msgs::action::Storage_Goal msg_;
};

class Init_Storage_Goal_topic_list
{
public:
  Init_Storage_Goal_topic_list()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Storage_Goal_storage_type topic_list(::astribot_msgs::action::Storage_Goal::_topic_list_type arg)
  {
    msg_.topic_list = std::move(arg);
    return Init_Storage_Goal_storage_type(msg_);
  }

private:
  ::astribot_msgs::action::Storage_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::action::Storage_Goal>()
{
  return astribot_msgs::action::builder::Init_Storage_Goal_topic_list();
}

}  // namespace astribot_msgs


namespace astribot_msgs
{

namespace action
{

namespace builder
{

class Init_Storage_Result_miss_topic_list
{
public:
  explicit Init_Storage_Result_miss_topic_list(::astribot_msgs::action::Storage_Result & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::action::Storage_Result miss_topic_list(::astribot_msgs::action::Storage_Result::_miss_topic_list_type arg)
  {
    msg_.miss_topic_list = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::action::Storage_Result msg_;
};

class Init_Storage_Result_dir_path
{
public:
  Init_Storage_Result_dir_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Storage_Result_miss_topic_list dir_path(::astribot_msgs::action::Storage_Result::_dir_path_type arg)
  {
    msg_.dir_path = std::move(arg);
    return Init_Storage_Result_miss_topic_list(msg_);
  }

private:
  ::astribot_msgs::action::Storage_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::action::Storage_Result>()
{
  return astribot_msgs::action::builder::Init_Storage_Result_dir_path();
}

}  // namespace astribot_msgs


namespace astribot_msgs
{

namespace action
{

namespace builder
{

class Init_Storage_Feedback_status
{
public:
  explicit Init_Storage_Feedback_status(::astribot_msgs::action::Storage_Feedback & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::action::Storage_Feedback status(::astribot_msgs::action::Storage_Feedback::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::action::Storage_Feedback msg_;
};

class Init_Storage_Feedback_miss_topic_list
{
public:
  explicit Init_Storage_Feedback_miss_topic_list(::astribot_msgs::action::Storage_Feedback & msg)
  : msg_(msg)
  {}
  Init_Storage_Feedback_status miss_topic_list(::astribot_msgs::action::Storage_Feedback::_miss_topic_list_type arg)
  {
    msg_.miss_topic_list = std::move(arg);
    return Init_Storage_Feedback_status(msg_);
  }

private:
  ::astribot_msgs::action::Storage_Feedback msg_;
};

class Init_Storage_Feedback_disk_space
{
public:
  explicit Init_Storage_Feedback_disk_space(::astribot_msgs::action::Storage_Feedback & msg)
  : msg_(msg)
  {}
  Init_Storage_Feedback_miss_topic_list disk_space(::astribot_msgs::action::Storage_Feedback::_disk_space_type arg)
  {
    msg_.disk_space = std::move(arg);
    return Init_Storage_Feedback_miss_topic_list(msg_);
  }

private:
  ::astribot_msgs::action::Storage_Feedback msg_;
};

class Init_Storage_Feedback_seconds
{
public:
  Init_Storage_Feedback_seconds()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Storage_Feedback_disk_space seconds(::astribot_msgs::action::Storage_Feedback::_seconds_type arg)
  {
    msg_.seconds = std::move(arg);
    return Init_Storage_Feedback_disk_space(msg_);
  }

private:
  ::astribot_msgs::action::Storage_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::action::Storage_Feedback>()
{
  return astribot_msgs::action::builder::Init_Storage_Feedback_seconds();
}

}  // namespace astribot_msgs


namespace astribot_msgs
{

namespace action
{

namespace builder
{

class Init_Storage_SendGoal_Request_goal
{
public:
  explicit Init_Storage_SendGoal_Request_goal(::astribot_msgs::action::Storage_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::action::Storage_SendGoal_Request goal(::astribot_msgs::action::Storage_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::action::Storage_SendGoal_Request msg_;
};

class Init_Storage_SendGoal_Request_goal_id
{
public:
  Init_Storage_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Storage_SendGoal_Request_goal goal_id(::astribot_msgs::action::Storage_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Storage_SendGoal_Request_goal(msg_);
  }

private:
  ::astribot_msgs::action::Storage_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::action::Storage_SendGoal_Request>()
{
  return astribot_msgs::action::builder::Init_Storage_SendGoal_Request_goal_id();
}

}  // namespace astribot_msgs


namespace astribot_msgs
{

namespace action
{

namespace builder
{

class Init_Storage_SendGoal_Response_stamp
{
public:
  explicit Init_Storage_SendGoal_Response_stamp(::astribot_msgs::action::Storage_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::action::Storage_SendGoal_Response stamp(::astribot_msgs::action::Storage_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::action::Storage_SendGoal_Response msg_;
};

class Init_Storage_SendGoal_Response_accepted
{
public:
  Init_Storage_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Storage_SendGoal_Response_stamp accepted(::astribot_msgs::action::Storage_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Storage_SendGoal_Response_stamp(msg_);
  }

private:
  ::astribot_msgs::action::Storage_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::action::Storage_SendGoal_Response>()
{
  return astribot_msgs::action::builder::Init_Storage_SendGoal_Response_accepted();
}

}  // namespace astribot_msgs


namespace astribot_msgs
{

namespace action
{

namespace builder
{

class Init_Storage_GetResult_Request_goal_id
{
public:
  Init_Storage_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::astribot_msgs::action::Storage_GetResult_Request goal_id(::astribot_msgs::action::Storage_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::action::Storage_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::action::Storage_GetResult_Request>()
{
  return astribot_msgs::action::builder::Init_Storage_GetResult_Request_goal_id();
}

}  // namespace astribot_msgs


namespace astribot_msgs
{

namespace action
{

namespace builder
{

class Init_Storage_GetResult_Response_result
{
public:
  explicit Init_Storage_GetResult_Response_result(::astribot_msgs::action::Storage_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::action::Storage_GetResult_Response result(::astribot_msgs::action::Storage_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::action::Storage_GetResult_Response msg_;
};

class Init_Storage_GetResult_Response_status
{
public:
  Init_Storage_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Storage_GetResult_Response_result status(::astribot_msgs::action::Storage_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Storage_GetResult_Response_result(msg_);
  }

private:
  ::astribot_msgs::action::Storage_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::action::Storage_GetResult_Response>()
{
  return astribot_msgs::action::builder::Init_Storage_GetResult_Response_status();
}

}  // namespace astribot_msgs


namespace astribot_msgs
{

namespace action
{

namespace builder
{

class Init_Storage_FeedbackMessage_feedback
{
public:
  explicit Init_Storage_FeedbackMessage_feedback(::astribot_msgs::action::Storage_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::action::Storage_FeedbackMessage feedback(::astribot_msgs::action::Storage_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::action::Storage_FeedbackMessage msg_;
};

class Init_Storage_FeedbackMessage_goal_id
{
public:
  Init_Storage_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Storage_FeedbackMessage_feedback goal_id(::astribot_msgs::action::Storage_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Storage_FeedbackMessage_feedback(msg_);
  }

private:
  ::astribot_msgs::action::Storage_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::action::Storage_FeedbackMessage>()
{
  return astribot_msgs::action::builder::Init_Storage_FeedbackMessage_goal_id();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__ACTION__DETAIL__STORAGE__BUILDER_HPP_
