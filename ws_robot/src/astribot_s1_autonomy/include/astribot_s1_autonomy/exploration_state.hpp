// Copyright 2026 Astribot.
#ifndef ASTRIBOT_S1_AUTONOMY__EXPLORATION_STATE_HPP_
#define ASTRIBOT_S1_AUTONOMY__EXPLORATION_STATE_HPP_

namespace astribot_s1_autonomy
{

/// 探索状态。
enum class ExplorationState
{
  kIdle,            ///< 待机：等地图/TF 就绪，或被 pause 服务冻结后的初始态
  kBootstrap,       ///< 冷启动自举：原地小幅旋转让 SLAM 插入首批扫描
  kEscape,          ///< 膨胀带脱困：低速平移挪出 253 带，让全局规划器能接受起点
  kGenNextPoint,    ///< 生成下一目标：跑前沿搜索、按代价排序候选点
  kValidating,      ///< 校验中：等 ComputePathToPose 结果并做逐点未知区校验
  kNavigating,      ///< 导航中：单个目标已下发给 Nav2，正在执行
  kArrived,         ///< 抵达收敛：位置/朝向达标，正在等稳定驻留计时满足
  kPaused,          ///< 异常暂停：导航失败/定位丢失/候选连续不合法
  kCompleted        ///< 探索完成：地图内已无任何前沿格
};

/// 状态 → 字符串。用于日志和 /exploration/state 话题。
inline const char * toString(ExplorationState s)
{
  switch (s) {
    case ExplorationState::kIdle:
      return "IDLE";
    case ExplorationState::kBootstrap:
      return "BOOTSTRAP";
    case ExplorationState::kEscape:
      return "ESCAPE";
    case ExplorationState::kGenNextPoint:
      return "GEN_NEXT_POINT";
    case ExplorationState::kValidating:
      return "VALIDATING";
    case ExplorationState::kNavigating:
      return "NAVIGATING";
    case ExplorationState::kArrived:
      return "ARRIVED";
    case ExplorationState::kPaused:
      return "PAUSED";
    case ExplorationState::kCompleted:
      return "COMPLETED";
  }
  return "UNKNOWN";
}

/// 该状态下是否允许生成新的探索目标点。
///
/// 这是「严格单点推进」的核心判据之一：**只有** kGenNextPoint 允许生成，
/// 其余状态一律拦截。需求明确禁止「未抵达当前目标就提前发布下一个探索点」，
/// 把这个判断收敛成一个函数，避免散落在各处出现漏判。
inline bool allowsGoalGeneration(ExplorationState s)
{
  return s == ExplorationState::kGenNextPoint;
}

/// 该状态下是否有目标正在 Nav2 侧执行（用于对外报告和一致性自检）。
inline bool hasActiveNavGoal(ExplorationState s)
{
  return s == ExplorationState::kNavigating || s == ExplorationState::kArrived;
}

/// 该状态下本节点是否在自己驱动底盘（而不是由 Nav2 控制器驱动）。
///
/// BOOTSTRAP 与 ESCAPE 都会这样。单独抽一个判据是为了让「谁在发 cmd_vel」这件事
/// 在代码里有唯一出处：两个源同时发速度就是打架，必须一眼能查。
///
/// ⚠️ 新增直驱底盘的状态时**必须**同步加进这里，否则这个唯一出处就破了。
inline bool drivesChassisDirectly(ExplorationState s)
{
  return s == ExplorationState::kBootstrap || s == ExplorationState::kEscape;
}

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__EXPLORATION_STATE_HPP_
