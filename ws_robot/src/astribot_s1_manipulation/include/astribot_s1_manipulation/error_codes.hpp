// Copyright 2026 Astribot

#ifndef ASTRIBOT_S1_MANIPULATION__ERROR_CODES_HPP_
#define ASTRIBOT_S1_MANIPULATION__ERROR_CODES_HPP_

#include <cstdint>
#include <string>

namespace astribot_s1_manipulation
{

/// 规划链路的失败原因。数值稳定，可直接放进 ROS 消息或日志。
enum class PlanErrorCode : std::int32_t
{
  /// 规划成功，输出轨迹合法（已通过限位/碰撞/奇异/闭链四项校验）。
  kSuccess = 0,

  /// 入参非法（空组名、空指针、目标维度与关节数不符等）。
  kInvalidInput = 1,
  /// RobotModel / PlanningScene 不可用（URDF/SRDF 加载失败）。
  kRobotModelUnavailable = 2,
  /// SRDF 里找不到指定的规划组。
  kPlanningGroupNotFound = 3,
  /// 配置未完成就调用了求解接口（configure 没跑或返回失败）。
  kNotConfigured = 4,

  /// 起点已经在目标上（各关节偏差都在 already_at_goal_tolerance_rad 以内）。
  kAlreadyAtGoal = 5,

  /// OMPL 规划器无解或超时。
  kPlannerFailed = 10,
  /// follower 臂 IK 无解（闭链目标位姿超出可达空间）。
  kIkFailed = 11,
  /// 重试次数用尽仍未拿到合法轨迹。
  kRetriesExhausted = 12,

  /// 闭链相对位姿残差超阈值。
  kClosedChainResidualTooLarge = 20,
  /// 轨迹中存在奇异构型（雅可比最小奇异值过小或条件数过大）。
  kSingularConfiguration = 21,
  /// 自碰撞或臂-底盘碰撞。
  kSelfCollision = 22,
  /// 与环境障碍物碰撞。
  kEnvironmentCollision = 23,

  /// 时间参数化失败（IPTP/TOTG 都没能给出时间戳）。
  kTimeParameterizationFailed = 30,
  /// 轨迹突破关节速度/加速度硬限位。
  kJointLimitViolation = 31,

  /// 夹爪控制器的 FollowJointTrajectory action 服务端不存在或等待超时。
  kGripperActionUnavailable = 40,
  /// 夹爪目标被控制器拒绝（关节名不匹配、控制器未 active 等）。
  kGripperGoalRejected = 41,
  /// 等夹爪结果超时（控制器接了目标但没在时限内返回）。
  kGripperTimeout = 42,
  /// 控制器报完成，但实测关节值与目标偏差超阈值。
  ///
  /// 为什么必须单独判这一项：JTC 的 SUCCEEDED 只代表它自己的容差满足，
  /// 实测过"控制器报完成时手臂还在收敛"。夹爪没合到位却继续往下走，
  /// 会得到"attach 了但其实没夹住"的假成功。
  kGripperNotConverged = 43,
  /// 目标抓取宽度超出夹爪量程（张口最大值以上，或闭合极限以下）。
  /// 这是确定性结论：重试无意义，必须改物体尺寸或换夹爪。
  kGraspWidthUnreachable = 44,

  /// 捕获到第三方库抛出的异常（已记录日志，不再上抛）。
  kExceptionCaught = 90,
};

/// 错误码转可读字符串，用于日志与对外状态输出。
/// 返回静态字符串，调用方不需要管生命周期。
const char * toString(PlanErrorCode code) noexcept;

/// 该错误码代表的失败是否值得重试。
/// 采样式规划器有随机性，规划/IK 类失败重试有意义；
/// 输入与配置类失败重试纯属浪费，直接返回给调用方。
bool isRetryable(PlanErrorCode code) noexcept;

}  // namespace astribot_s1_manipulation

#endif  // ASTRIBOT_S1_MANIPULATION__ERROR_CODES_HPP_
