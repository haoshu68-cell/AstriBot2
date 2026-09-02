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
///   BOOTSTRAP —— 「冷启动自举」。SLAM 只在机器人移动超过 minimum_travel_heading
///       (本项目 0.2rad) 之后才插入新扫描，所以刚上电时地图可能一个已知格都没有；
///       而本节点在地图不可用时不下发目标 —— 机器人不动 → 地图不长 → 永远不动。
///       实测过这个死锁：必须人工推一把（外部脚本发 cmd_vel 走 0.79m）地图才从
///       0 m² 变成 26 m²。BOOTSTRAP 就是把这一把「推」收进节点自己，做成有限次、
///       有安全门、只原地旋转的动作。
///
///   ESCAPE —— 「膨胀带脱困」。与 BOOTSTRAP 是两个不同的死锁，别混：
///       BOOTSTRAP 治「地图还没长出来」，ESCAPE 治「机器人站在膨胀带里出不来」。
///       实测（2026-08-31 一次 3h41m 运行）：
///         planner_server 报 "Starting point in lethal space!" **25652 次**，
///         14 次到位全部集中在开头 2.5 分钟，之后 222 分钟只再到位 1 次。
///       成因：SmacPlanner2D 判起点用**单个中心格** cost >= INSCRIBED(253)，
///       而 253 是**膨胀层**写的；本机 inflation 从墙面向内吃掉内切半径 0.388m，
///       机器人中心一进这条带，全局规划器就拒绝从这里起步 —— 去任何目标都拒绝。
///       而 nav_dispatch_mode=follow_path 绕过了 BT，clear_costmap/backup/spin
///       全部拿不到；协调器的「自动恢复」只是状态复位，物理上什么都没做。
///
///       ⚠️ 关键区别：**BOOTSTRAP 的原地旋转救不了 ESCAPE** ——
///       判据是单个中心格，原地转不改变中心格代价。脱困必须含平移。
///
/// 关键区分（很容易写错）：
///   「地图里一个前沿格都没有」 ⇒ COMPLETED（真的探索完了）
///   「有前沿格但候选点全部不合法」 ⇒ PAUSED（还没探完，只是暂时找不到合法点）
///   把后者误判成 COMPLETED 会让上层提前停止探索。
///
///   还有第三种，正是上面 BOOTSTRAP 要解决的：
///   「一个前沿格都没有**且地图几乎全是未知**」 ⇒ 还没开始，不是探索完了。
///   只看前沿格数会把「冷启动的空地图」判成 COMPLETED。
///
///   第四种是 ESCAPE 要解决的：
///   「候选点全部因『全局规划失败』被否，而失败原因是**起点**致命」
///   ⇒ 不是找不到合法点，是机器人自己站错了地方。这两者的正确响应完全相反：
///     前者该换候选点，后者该把机器人挪出膨胀带。
/// ===================================================================
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
