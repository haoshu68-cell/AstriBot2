// Copyright 2026 Astribot.
//
// 探索状态机的状态枚举。
//
// 需求要求「状态机严格枚举化管理，禁止裸数字状态」，因此这里用 enum class，
// 并配一个 toString() 供日志和 /exploration/state 话题输出。
#ifndef ASTRIBOT_S1_AUTONOMY__EXPLORATION_STATE_HPP_
#define ASTRIBOT_S1_AUTONOMY__EXPLORATION_STATE_HPP_

namespace astribot_s1_autonomy
{

/// 探索状态。
///
/// ============================ 状态机说明 ============================
/// 需求指定的四状态主闭环：
///   IDLE → GEN_NEXT_POINT → NAVIGATING → ARRIVED → GEN_NEXT_POINT → ...
///
/// 另外三个状态不是画蛇添足，是需求自身的规则必然要求的：
///
///   VALIDATING —— 「生成候选点」和「下发目标」之间必须夹一个等待态。
///       原因：路径合法性校验要调 Nav2 的 ComputePathToPose，那是**异步 action**，
///       不能在回调线程里阻塞等结果（会把执行器卡死、TF 和地图都收不到）。
///       它在语义上是 GEN_NEXT_POINT 的子步骤，不是第五个业务阶段。
///
///   PAUSED —— 对应需求「异常场景处理规则」第1、2条：导航失败/局部被困/定位丢失时
///       必须「停止生成新探索点，保持当前状态，输出告警，等待人工重置或自动重试」。
///       没有这个状态就没法表达「冻结但未结束」。
///
///   COMPLETED —— 对应需求第5条：「探索边界无合法已知区域可前进 → 自动停止探索任务，
///       输出探索完成状态」。
///
/// 关键区分（很容易写错）：
///   「地图里一个前沿格都没有」 ⇒ COMPLETED（真的探索完了）
///   「有前沿格但候选点全部不合法」 ⇒ PAUSED（还没探完，只是暂时找不到合法点）
///   把后者误判成 COMPLETED 会让上层提前停止探索。
/// ===================================================================
enum class ExplorationState
{
  kIdle,            ///< 待机：等地图/TF 就绪，或被 pause 服务冻结后的初始态
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
  // 枚举被扩展但忘了加分支时走到这里。返回显式的未知标记而不是空串，
  // 便于在话题里一眼看出「状态机被改坏了」。
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

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__EXPLORATION_STATE_HPP_
