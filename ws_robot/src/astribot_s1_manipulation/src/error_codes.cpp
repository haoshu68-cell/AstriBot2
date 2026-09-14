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
    case PlanErrorCode::kAlreadyAtGoal:
      return "ALREADY_AT_GOAL";
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
    case PlanErrorCode::kGripperActionUnavailable:
      return "GRIPPER_ACTION_UNAVAILABLE";
    case PlanErrorCode::kGripperGoalRejected:
      return "GRIPPER_GOAL_REJECTED";
    case PlanErrorCode::kGripperTimeout:
      return "GRIPPER_TIMEOUT";
    case PlanErrorCode::kGripperNotConverged:
      return "GRIPPER_NOT_CONVERGED";
    case PlanErrorCode::kGraspWidthUnreachable:
      return "GRASP_WIDTH_UNREACHABLE";
    case PlanErrorCode::kExceptionCaught:
      return "EXCEPTION_CAUGHT";
  }
  return "UNKNOWN";
}

bool isRetryable(PlanErrorCode code) noexcept
{
  switch (code) {
    case PlanErrorCode::kPlannerFailed:
    case PlanErrorCode::kIkFailed:
    case PlanErrorCode::kClosedChainResidualTooLarge:
    case PlanErrorCode::kSingularConfiguration:
    case PlanErrorCode::kSelfCollision:
    case PlanErrorCode::kEnvironmentCollision:
      return true;

    case PlanErrorCode::kGripperNotConverged:
      return true;

    case PlanErrorCode::kSuccess:
    case PlanErrorCode::kInvalidInput:
    case PlanErrorCode::kRobotModelUnavailable:
    case PlanErrorCode::kPlanningGroupNotFound:
    case PlanErrorCode::kNotConfigured:
    case PlanErrorCode::kAlreadyAtGoal:
    case PlanErrorCode::kRetriesExhausted:
    case PlanErrorCode::kTimeParameterizationFailed:
    case PlanErrorCode::kJointLimitViolation:
    case PlanErrorCode::kGripperActionUnavailable:
    case PlanErrorCode::kGripperGoalRejected:
    case PlanErrorCode::kGripperTimeout:
    case PlanErrorCode::kGraspWidthUnreachable:
    case PlanErrorCode::kExceptionCaught:
      return false;
  }
  return false;
}

}  // namespace astribot_s1_manipulation
