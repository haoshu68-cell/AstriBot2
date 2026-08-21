// Copyright 2026 Astribot
//
// 统一错误码。
//
// 为什么不用异常、也不用 bool：
//   · 任务要求"规划失败返回错误码，上层可以重试"，bool 表达不了"为什么失败"，
//     而调用方的重试策略恰恰取决于失败原因（IK 失败可以换随机种子重试，
//     关节限位违规重试多少次都是白费）。
//   · 规划链路跨 OMPL / FCL / KDL 三个第三方库，异常类型不可控，
//     所以在本包边界上一律 catch 住转成错误码，绝不让异常穿透出去。

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

  // ---- 输入与初始化类：调用方重试无意义，必须先改配置 ----
  /// 入参非法（空组名、空指针、目标维度与关节数不符等）。
  kInvalidInput = 1,
  /// RobotModel / PlanningScene 不可用（URDF/SRDF 加载失败）。
  kRobotModelUnavailable = 2,
  /// SRDF 里找不到指定的规划组。
  kPlanningGroupNotFound = 3,
  /// 配置未完成就调用了求解接口（configure 没跑或返回失败）。
  kNotConfigured = 4,

  // ---- 规划求解类：重试可能有效（采样式规划器有随机性）----
  /// OMPL 规划器无解或超时。
  kPlannerFailed = 10,
  /// follower 臂 IK 无解（闭链目标位姿超出可达空间）。
  kIkFailed = 11,
  /// 重试次数用尽仍未拿到合法轨迹。
  kRetriesExhausted = 12,

  // ---- 校验类：当前解不可用，重规划换一条 ----
  /// 闭链相对位姿残差超阈值。
  kClosedChainResidualTooLarge = 20,
  /// 轨迹中存在奇异构型（雅可比最小奇异值过小或条件数过大）。
  kSingularConfiguration = 21,
  /// 自碰撞或臂-底盘碰撞。
  kSelfCollision = 22,
  /// 与环境障碍物碰撞。
  kEnvironmentCollision = 23,

  // ---- 后处理类 ----
  /// 时间参数化失败（IPTP/TOTG 都没能给出时间戳）。
  kTimeParameterizationFailed = 30,
  /// 轨迹突破关节速度/加速度硬限位。
  kJointLimitViolation = 31,

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
