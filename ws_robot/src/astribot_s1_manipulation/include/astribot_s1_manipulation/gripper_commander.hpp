// Copyright 2026 Astribot

#ifndef ASTRIBOT_S1_MANIPULATION__GRIPPER_COMMANDER_HPP_
#define ASTRIBOT_S1_MANIPULATION__GRIPPER_COMMANDER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "astribot_s1_manipulation/error_codes.hpp"

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace astribot_s1_manipulation
{

/// 夹爪配置。全部从 yaml 读，构造时注入。
struct GripperConfig
{
  /// SRDF 里的夹爪规划组名，如 "gripper_left"。开合角从这个组的 group_state 读。
  std::string group_name;
  /// JTC 的 action 名，如 "/gripper_left_controller/follow_joint_trajectory"。
  std::string action_name;
  /// SRDF 中「张开」「闭合」两个 group_state 的名字。
  std::string open_state_name{"open"};
  std::string closed_state_name{"closed"};

  /// 量张口宽度用的两个指垫 link，以及张口方向所在的参考坐标系（TCP）。
  /// 张口方向由 jaw_axis_in_tcp 给出（TCP 系下的单位向量）。
  std::string tcp_link;
  std::string left_pad_link;
  std::string right_pad_link;
  std::vector<double> jaw_axis_in_tcp{1.0, 0.0, 0.0};

  /// 下发轨迹的时长（s）。夹爪行程很短，不需要长时间。
  double move_time_sec{1.2};
  /// 控制器报完成后额外驻留多久再读稳态值（s）。
  ///
  /// 不是保守起见随手加的：实测过「控制器报完成时手臂还在收敛」，
  /// 紧接着下一步规划就拿到移动中的起点并报 start point deviates。
  double settle_time_sec{0.8};
  /// 判「已到位」的角度容差（rad）。
  double converge_tolerance_rad{0.02};
  /// 等 action 服务端 / 等结果的超时（s）。
  double server_wait_sec{10.0};
  double result_timeout_sec{15.0};

  /// 抓取预紧量（m）：闭合到「物体宽度 − 预紧」，让指垫压进去一点而不是刚好贴上。
  /// 给 0 就是理论贴合，实测容易因为几何误差变成没夹到。
  double grasp_preload_m{0.004};

  /// 读实测关节值的话题。
  std::string joint_states_topic{"/joint_states"};
};

/// 一次夹爪动作的实测结果。
struct GripperOutcome
{
  PlanErrorCode code{PlanErrorCode::kNotConfigured};
  /// 下发的目标角（rad）。
  double target_rad{0.0};
  /// 驻留后读到的实测角（rad）。读不到时为 NaN。
  double measured_rad{0.0};
  /// 目标角对应的理论张口宽度（m）。
  double target_width_m{0.0};
  double elapsed_sec{0.0};
  std::string detail;

  bool ok() const noexcept {return code == PlanErrorCode::kSuccess;}
};

/// 夹爪动作器。
class GripperCommander
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  explicit GripperCommander(rclcpp::Node::SharedPtr node);
  ~GripperCommander() = default;

  GripperCommander(const GripperCommander &) = delete;
  GripperCommander & operator=(const GripperCommander &) = delete;

  /// 读 SRDF、校验模型、建 action client 与订阅。
  /// 失败返回具体错误码并把原因写进 detail，不抛异常。
  PlanErrorCode configure(
    const moveit::core::RobotModelConstPtr & model,
    const GripperConfig & config,
    std::string & detail);

  bool isConfigured() const noexcept {return configured_;}

  /// SRDF 里读到的张开角 / 闭合角（rad）。未 configure 时返回 NaN。
  double openAngle() const noexcept {return open_angle_;}
  double closedAngle() const noexcept {return closed_angle_;}
  /// 主动关节名。
  const std::string & jointName() const noexcept {return joint_name_;}

  /// 给定主动关节角，用正解量出两指垫之间的净空（m）。
  ///
  /// 量的是两个指垫**相向面之间的间隙**，不是两个 link 原点距离 ——
  /// 指垫有厚度，原点距离会系统性高估张口。angle 超出 [open, closed]
  /// 会被夹到区间内（并在 detail 里说明），不返回外插值。
  double jawWidthAtAngle(double angle_rad) const;

  /// 反解：要夹住宽 width_m 的物体，主动关节该走到多少角度。
  ///
  /// 目标张口 = width_m − grasp_preload_m。超出量程返回
  /// kGraspWidthUnreachable（确定性结论，重试无意义）。
  PlanErrorCode graspAngleForWidth(
    double width_m, double & angle_rad, std::string & detail) const;

  /// 走到 SRDF 的「张开」位。
  GripperOutcome open();
  /// 走到 SRDF 的「闭合」位（完全闭合，不考虑物体）。
  GripperOutcome close();
  /// 闭合到刚好夹住 width_m 宽的物体。
  GripperOutcome closeToWidth(double width_m);
  /// 走到指定角度。
  GripperOutcome moveTo(double angle_rad);

  /// 读当前实测角（rad）。读不到（还没收到 /joint_states）返回 false。
  bool measuredAngle(double & angle_rad) const;

private:
  /// 采样 [open, closed] 建一张 角度→张口 的表，供 jawWidthAtAngle 插值、
  /// 供 graspAngleForWidth 反查。configure 时建一次。
  PlanErrorCode buildJawTable(std::string & detail);

  /// 真正下发一次 JTC 目标并等结果。不含收敛判定。
  PlanErrorCode sendTrajectory(double angle_rad, std::string & detail);

  void onJointStates(const sensor_msgs::msg::JointState::ConstSharedPtr & msg);

  rclcpp::Node::SharedPtr node_;
  GripperConfig config_;
  moveit::core::RobotModelConstPtr model_;

  bool configured_{false};
  std::string joint_name_;
  double open_angle_{0.0};
  double closed_angle_{0.0};

  /// 角度→张口 采样表（角度单调递增）。
  std::vector<double> table_angle_;
  std::vector<double> table_width_;

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  mutable std::mutex state_mutex_;
  bool have_measured_{false};
  double measured_angle_{0.0};
};

}  // namespace astribot_s1_manipulation

#endif  // ASTRIBOT_S1_MANIPULATION__GRIPPER_COMMANDER_HPP_
