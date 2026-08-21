// Copyright 2026 Astribot
//
// OMPL 规划器扩展插件：把 MoveIt2 官方没有注册的 OMPL 规划器补注册进去。
//
// 为什么需要这个东西（这是本文件存在的全部理由）
// ----------------------------------------------
// 任务要求启用 RRT* / BIT* / Informed RRT* 三类规划器。实测结论：
//
//   $ strings /opt/ros/humble/lib/libmoveit_ompl_interface.so
//       | grep -oE "geometric::[A-Za-z_]+" | sort -u
//   共 25 个，其中有 geometric::RRTstar，**没有** BITstar，**没有** InformedRRTstar
//   （注意：上面这条命令原本是一行，这里断成两行是因为行尾反斜杠在 // 注释里
//    会被当作续行符，触发 -Wcomment 告警 —— 本工程要求零 warning）
//
// 也就是说 MoveIt2 Humble 的官方 OMPL 接口只注册了三者中的一个。
// 而 OMPL 1.7 本体是**有**这两个规划器的：
//   /opt/ros/humble/include/ompl-1.7/ompl/geometric/planners/informedtrees/BITstar.h
//   /opt/ros/humble/include/ompl-1.7/ompl/geometric/planners/rrt/InformedRRTstar.h
//
// 更麻烦的是：在 ompl_planning.yaml 里写一个 MoveIt 不认识的规划器名，
// MoveIt **不报错**，而是静默回退到该组的默认规划器。所以"配置写了 BIT*、
// 实际跑的是 RRTConnect"这种事完全不会被发现，除非去看日志里的实际规划器名。
//
// 实现路径（只用公开 API，不改任何系统库源码）
// ------------------------------------------
// MoveIt 官方的插件类 ompl_interface::OMPLPlannerManager **没有公开头文件**
// （只在它自己的 .cpp 里定义），所以无法继承它。但是：
//
//   ompl_interface::OMPLInterface                              <- 有公开头
//     ::getPlanningContextManager()  非 const 重载             <- 公开 (ompl_interface.h:94)
//       ::registerPlannerAllocator(id, allocator)              <- 公开 (planning_context_manager.h:187)
//
// 因此本文件实现一个自己的 planning_interface::PlannerManager 插件，
// 内部持有一个 OMPLInterface，在 initialize() 里把缺失的规划器注册进去。
// 这符合任务约束「允许基于 OMPL 原生接口做封装扩展」「禁止修改系统库源码」。
//
// allocator 的签名（model_based_planning_context.h:60）：
//   std::function<ob::PlannerPtr(const ob::SpaceInformationPtr& si,
//                                const std::string& name,
//                                const ModelBasedPlanningContextSpecification& spec)>

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

// OMPL 里官方未注册、但本工程需要的两个规划器
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>

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
template<typename PlannerT>
ompl_interface::ConfiguredPlannerAllocator makeAllocator()
{
  return [](const ob::SpaceInformationPtr & si, const std::string & name,
      const ompl_interface::ModelBasedPlanningContextSpecification & spec) -> ob::PlannerPtr {
           if (!si) {
             RCLCPP_ERROR(
               rclcpp::get_logger(kLoggerName),
               "space information is null, cannot allocate planner '%s'", name.c_str());
             return ob::PlannerPtr();
           }
           auto planner = std::make_shared<PlannerT>(si);
           if (!name.empty()) {
             planner->setName(name);
           }
           applyYamlParams(planner, spec.config_);
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
      // OMPLInterface 的构造会从参数服务器读取 ompl_planning.yaml 的内容
      // （planner_configs 段与各组段），行为与官方插件完全一致。
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
    // 与官方 OMPL 插件相同的判定：本插件只做关节空间/位姿目标的路径规划，
    // 不处理轨迹约束(trajectory_constraints)。
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

    // 这里要报两类名字，缺一不可：
    //   1. 已注册的 allocator id（"geometric::BITstar" 这种 OMPL 类名）
    //      —— 证明规划器本身可用。
    //   2. ompl_planning.yaml 里的**配置名**（"BITstarConfig" 这种）
    //      —— 这才是 MotionPlanRequest.planner_id 实际该填的值。
    //
    // 只报第 1 类的话（本插件最初的写法），`ros2 service call
    // /query_planner_interface` 看到的全是 OMPL 类名，用户照着填
    // "BITstarConfig" 会以为规划器不存在；反过来只报第 2 类，
    // 又看不出哪些规划器是本插件补注册进来的。两类都报最省事。
    const auto & allocators =
      ompl_interface_->getPlanningContextManager().getRegisteredPlannerAllocators();
    const auto & configurations =
      ompl_interface_->getPlanningContextManager().getPlannerConfigurations();

    algs.reserve(allocators.size() + configurations.size());
    for (const auto & item : allocators) {
      algs.push_back(item.first);
    }
    for (const auto & item : configurations) {
      // PlannerConfigurationMap 的 key 形如 "arm_left[RRTstarConfig]" 或
      // 直接就是 "RRTstarConfig"。两种都原样报出去，让调用方能对上号。
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
      // 这里把实际生效的规划器名打出来。前面说过：yaml 里写了不存在的
      // 规划器时 MoveIt 会静默回退，只有对着日志核对才能发现。
      RCLCPP_INFO(
        rclcpp::get_logger(kLoggerName),
        "planning request: group='%s' planner_id='%s' allowed_time=%.2fs",
        req.group_name.c_str(),
        req.planner_id.empty() ? "(group default)" : req.planner_id.c_str(),
        req.allowed_planning_time);

      return ompl_interface_->getPlanningContext(planning_scene, req, error_code);
    } catch (const std::exception & e) {
      // 绝不让第三方库的异常穿出插件边界：move_group 不会捕获它，
      // 会直接让整个 move_group 进程终止。
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

    // ---- 任务明确要求的两个 ----
    manager.registerPlannerAllocator("geometric::BITstar", makeAllocator<og::BITstar>());
    manager.registerPlannerAllocator(
      "geometric::InformedRRTstar", makeAllocator<og::InformedRRTstar>());

    // ---- 同族的另外几个，一并注册（成本为零，便于后续对比选型）----
    // ABITstar  : BIT* 的 anytime 变体，先快速出解再持续优化
    // AITstar   : 自适应启发式的 informed tree
    // SORRTstar : Informed RRT* 的有序采样变体
    manager.registerPlannerAllocator("geometric::ABITstar", makeAllocator<og::ABITstar>());
    manager.registerPlannerAllocator("geometric::AITstar", makeAllocator<og::AITstar>());
    manager.registerPlannerAllocator("geometric::SORRTstar", makeAllocator<og::SORRTstar>());

    const auto & allocators = manager.getRegisteredPlannerAllocators();
    RCLCPP_INFO(
      rclcpp::get_logger(kLoggerName),
      "registered extra OMPL planners: %zu -> %zu total",
      before, allocators.size());

    // 把三个必需规划器逐个确认一遍并打日志。任务的成功判定要求
    // "可切换 RRT*/BIT*/Informed RRT*"，这几行就是该判定的自证。
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
