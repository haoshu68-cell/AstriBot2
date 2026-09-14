// Copyright 2026 Astribot

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "astribot_s1_manipulation/optimizing_planner_wrapper.hpp"

#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace astribot_s1_manipulation
{

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{
constexpr const char * kLoggerName = "astribot_s1_manipulation.ompl_extension";

/// ompl_planning.yaml 里属于 MoveIt 自己的配置键。
/// 这些键不是 OMPL 规划器的参数，透传给 planner->params().setParam() 只会
/// 触发"未知参数"告警，所以要过滤掉。
/// 名单来源：MoveIt 的 ModelBasedPlanningContext 会消费这些键。
const std::set<std::string> & moveitReservedKeys()
{
  static const std::set<std::string> keys = {
    "type",
    "projection_evaluator",
    "longest_valid_segment_fraction",
    "enforce_constrained_state_space",
    "enforce_joint_model_state_space",
    "multi_query_planning_enabled",
    "hybridize",
    "interpolate",
    "max_velocity_scaling_factor",
    "max_acceleration_scaling_factor",
    "start_state_max_bounds_error",
    "jiggle_fraction",
    "max_sampling_attempts",
    "termination_condition",
  };
  return keys;
}

/// 把 yaml 里的规划器参数应用到 OMPL planner 实例上。
///
/// OMPL 每个规划器都用 ParamSet 声明自己支持的参数（declareParam），
/// 所以这里能做到"通用"：不需要为每个规划器手写一遍参数列表，
/// 只要 yaml 里的键名与 OMPL 的参数名一致就会生效。
/// 键名写错时 hasParam 返回 false，打 WARN —— 这一点很重要：
/// 否则参数写错了会静默不生效，让人误以为调过参了。
template<typename PlannerT>
void applyYamlParams(
  const std::shared_ptr<PlannerT> & planner,
  const std::map<std::string, std::string> & config)
{
  if (!planner) {
    return;
  }
  ob::ParamSet & param_set = planner->params();
  for (const auto & item : config) {
    const std::string & key = item.first;
    const std::string & value = item.second;
    if (moveitReservedKeys().count(key) > 0U) {
      continue;
    }
    if (!param_set.hasParam(key)) {
      RCLCPP_WARN(
        rclcpp::get_logger(kLoggerName),
        "planner '%s' has no parameter named '%s' (value '%s' ignored); "
        "check the key spelling against OMPL's declareParam names",
        planner->getName().c_str(), key.c_str(), value.c_str());
      continue;
    }
    if (!param_set.setParam(key, value)) {
      RCLCPP_WARN(
        rclcpp::get_logger(kLoggerName),
        "planner '%s': failed to set parameter '%s' = '%s' (value rejected by OMPL)",
        planner->getName().c_str(), key.c_str(), value.c_str());
    } else {
      RCLCPP_DEBUG(
        rclcpp::get_logger(kLoggerName),
        "planner '%s': %s = %s", planner->getName().c_str(), key.c_str(), value.c_str());
    }
  }
}

/// 通用 allocator 工厂：为任意 OMPL geometric 规划器生成 MoveIt 需要的 allocator。
///
/// 一律套上 OptimizingPlannerWrapper（见该头文件的长注释）。策略两项都不配时
/// 包装类内部直接短路到 PlannerT::solve()，行为与原生规划器逐字节一致 ——
/// 所以这里不做"要不要包装"的分支，只保留一条代码路径。
template<typename PlannerT>
ompl_interface::ConfiguredPlannerAllocator makeAllocator()
{
  return [](const ob::SpaceInformationPtr & si, const std::string & name,
      const ompl_interface::ModelBasedPlanningContextSpecification & spec) -> ob::PlannerPtr {
           const rclcpp::Logger logger = rclcpp::get_logger(kLoggerName);
           if (!si) {
             RCLCPP_ERROR(
               logger, "space information is null, cannot allocate planner '%s'", name.c_str());
             return ob::PlannerPtr();
           }
           std::map<std::string, std::string> config = spec.config_;
           const OptimizingPlannerPolicy policy = extractOptimizingPolicy(config, name, logger);

           auto planner = std::make_shared<OptimizingPlannerWrapper<PlannerT>>(si, policy, logger);
           if (!name.empty()) {
             planner->setName(name);
           }
           applyYamlParams(planner, config);
           return planner;
         };
}
}  // namespace

/// 自定义 OMPL 规划器管理插件。
///
/// 行为与官方 ompl_interface/OMPLPlanner 一致，唯一区别是在初始化时
/// 额外注册了 BITstar / InformedRRTstar / ABITstar / AITstar / SORRTstar。
class OmplPlannerExtension : public planning_interface::PlannerManager
{
public:
  OmplPlannerExtension() = default;
  ~OmplPlannerExtension() override = default;

  bool initialize(
    const moveit::core::RobotModelConstPtr & model,
    const rclcpp::Node::SharedPtr & node,
    const std::string & parameter_namespace) override
  {
    if (!model) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "robot model is null, cannot initialize");
      return false;
    }
    if (!node) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "node is null, cannot initialize");
      return false;
    }

    node_ = node;
    try {
      ompl_interface_ = std::make_unique<ompl_interface::OMPLInterface>(
        model, node, parameter_namespace);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "failed to construct OMPLInterface: %s", e.what());
      ompl_interface_.reset();
      return false;
    }

    registerExtraPlanners();
    return true;
  }

  bool canServiceRequest(const planning_interface::MotionPlanRequest & req) const override
  {
    return req.trajectory_constraints.constraints.empty();
  }

  std::string getDescription() const override
  {
    return "OMPL (with BITstar / InformedRRTstar registered by astribot_s1_manipulation)";
  }

  void getPlanningAlgorithms(std::vector<std::string> & algs) const override
  {
    algs.clear();
    if (!ompl_interface_) {
      RCLCPP_WARN(
        rclcpp::get_logger(kLoggerName),
        "getPlanningAlgorithms called before successful initialize()");
      return;
    }

    const auto & allocators =
      ompl_interface_->getPlanningContextManager().getRegisteredPlannerAllocators();
    const auto & configurations =
      ompl_interface_->getPlanningContextManager().getPlannerConfigurations();

    algs.reserve(allocators.size() + configurations.size());
    for (const auto & item : allocators) {
      algs.push_back(item.first);
    }
    for (const auto & item : configurations) {
      algs.push_back(item.first);
    }
    std::sort(algs.begin(), algs.end());
    algs.erase(std::unique(algs.begin(), algs.end()), algs.end());
  }

  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap & pcs) override
  {
    if (!ompl_interface_) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "setPlannerConfigurations called before successful initialize()");
      return;
    }
    ompl_interface_->setPlannerConfigurations(pcs);
  }

  planning_interface::PlanningContextPtr getPlanningContext(
    const planning_scene::PlanningSceneConstPtr & planning_scene,
    const planning_interface::MotionPlanRequest & req,
    moveit_msgs::msg::MoveItErrorCodes & error_code) const override
  {
    if (!ompl_interface_) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "getPlanningContext called before successful initialize()");
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return planning_interface::PlanningContextPtr();
    }
    if (!planning_scene) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "planning scene is null");
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return planning_interface::PlanningContextPtr();
    }

    try {
      RCLCPP_INFO(
        rclcpp::get_logger(kLoggerName),
        "planning request: group='%s' planner_id='%s' allowed_time=%.2fs",
        req.group_name.c_str(),
        req.planner_id.empty() ? "(group default)" : req.planner_id.c_str(),
        req.allowed_planning_time);

      return ompl_interface_->getPlanningContext(planning_scene, req, error_code);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "exception while creating planning context: %s", e.what());
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return planning_interface::PlanningContextPtr();
    }
  }

private:
  /// 注册官方没有注册的规划器。
  ///
  /// 注册顺序无关；同名 id 重复注册会覆盖（known_planners_ 是 map），
  /// 所以这里用的 id 与 OMPL 的类名保持一致："geometric::BITstar" 等，
  /// 与 ompl_planning.yaml 里 type 字段的写法必须完全一致。
  void registerExtraPlanners()
  {
    if (!ompl_interface_) {
      return;
    }
    ompl_interface::PlanningContextManager & manager =
      ompl_interface_->getPlanningContextManager();

    const auto before = manager.getRegisteredPlannerAllocators().size();

    manager.registerPlannerAllocator("geometric::BITstar", makeAllocator<og::BITstar>());
    manager.registerPlannerAllocator(
      "geometric::InformedRRTstar", makeAllocator<og::InformedRRTstar>());

    manager.registerPlannerAllocator("geometric::ABITstar", makeAllocator<og::ABITstar>());
    manager.registerPlannerAllocator("geometric::AITstar", makeAllocator<og::AITstar>());
    manager.registerPlannerAllocator("geometric::SORRTstar", makeAllocator<og::SORRTstar>());

    manager.registerPlannerAllocator("geometric::RRTstar", makeAllocator<og::RRTstar>());

    const auto & allocators = manager.getRegisteredPlannerAllocators();
    RCLCPP_INFO(
      rclcpp::get_logger(kLoggerName),
      "registered extra OMPL planners: %zu -> %zu total",
      before, allocators.size());

    for (const char * required :
      {"geometric::RRTstar", "geometric::BITstar", "geometric::InformedRRTstar"})
    {
      if (allocators.count(required) > 0U) {
        RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "  [OK]      %s", required);
      } else {
        RCLCPP_ERROR(
          rclcpp::get_logger(kLoggerName),
          "  [MISSING] %s  <- planning requests naming it will silently fall back "
          "to the group default planner", required);
      }
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<ompl_interface::OMPLInterface> ompl_interface_;
};

}  // namespace astribot_s1_manipulation

PLUGINLIB_EXPORT_CLASS(
  astribot_s1_manipulation::OmplPlannerExtension,
  planning_interface::PlannerManager)
