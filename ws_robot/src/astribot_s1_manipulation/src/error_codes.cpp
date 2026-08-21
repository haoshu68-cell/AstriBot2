// Copyright 2026 Astribot

#include "astribot_s1_manipulation/error_codes.hpp"

namespace astribot_s1_manipulation
{

const char * toString(PlanErrorCode code) noexcept
{
  switch (code) {
    case PlanErrorCode::kSuccess:
      return "SUCCESS";
    case PlanErrorCode::kInvalidInput:
      return "INVALID_INPUT";
    case PlanErrorCode::kRobotModelUnavailable:
      return "ROBOT_MODEL_UNAVAILABLE";
    case PlanErrorCode::kPlanningGroupNotFound:
      return "PLANNING_GROUP_NOT_FOUND";
    case PlanErrorCode::kNotConfigured:
      return "NOT_CONFIGURED";
    case PlanErrorCode::kPlannerFailed:
      return "PLANNER_FAILED";
    case PlanErrorCode::kIkFailed:
      return "IK_FAILED";
    case PlanErrorCode::kRetriesExhausted:
      return "RETRIES_EXHAUSTED";
    case PlanErrorCode::kClosedChainResidualTooLarge:
      return "CLOSED_CHAIN_RESIDUAL_TOO_LARGE";
    case PlanErrorCode::kSingularConfiguration:
      return "SINGULAR_CONFIGURATION";
    case PlanErrorCode::kSelfCollision:
      return "SELF_COLLISION";
    case PlanErrorCode::kEnvironmentCollision:
      return "ENVIRONMENT_COLLISION";
    case PlanErrorCode::kTimeParameterizationFailed:
      return "TIME_PARAMETERIZATION_FAILED";
    case PlanErrorCode::kJointLimitViolation:
      return "JOINT_LIMIT_VIOLATION";
    case PlanErrorCode::kExceptionCaught:
      return "EXCEPTION_CAUGHT";
  }
  // switch 已覆盖所有枚举值；这一行只为防止将来新增枚举后编译器走到函数尾部。
  return "UNKNOWN";
}

bool isRetryable(PlanErrorCode code) noexcept
{
  switch (code) {
    // 采样式规划器每次的随机种子不同，重试确实可能拿到解。
    case PlanErrorCode::kPlannerFailed:
    case PlanErrorCode::kIkFailed:
    // 这三类是"当前这条解不行"，换一条解可能就过了。
    case PlanErrorCode::kClosedChainResidualTooLarge:
    case PlanErrorCode::kSingularConfiguration:
    case PlanErrorCode::kSelfCollision:
    case PlanErrorCode::kEnvironmentCollision:
      return true;

    // 输入/配置/后处理类：重试不会改变结果，重试只是浪费时间。
    // 特别是 kJointLimitViolation —— 轨迹本身超限，再规划多少次
    // 也得靠改 joint_limits.yaml 或 scaling 才能解决。
    case PlanErrorCode::kSuccess:
    case PlanErrorCode::kInvalidInput:
    case PlanErrorCode::kRobotModelUnavailable:
    case PlanErrorCode::kPlanningGroupNotFound:
    case PlanErrorCode::kNotConfigured:
    case PlanErrorCode::kRetriesExhausted:
    case PlanErrorCode::kTimeParameterizationFailed:
    case PlanErrorCode::kJointLimitViolation:
    case PlanErrorCode::kExceptionCaught:
      return false;
  }
  return false;
}

}  // namespace astribot_s1_manipulation
