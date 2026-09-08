// Copyright 2026 Astribot. Apache-2.0.
//
// CurvatureSpeedLimitCritic —— MPPI 的曲率-速度耦合限速 critic。
//
// 思路来源：nav2 社区 PR 的 AngularVelocitySpeedLimitCritic（按角速度限制线速度）。
// 本实现在其基础上加了**路径几何前馈项**，并把两项统一到同一个侧向加速度预算
// a_lat_max 上，所以改名为 CurvatureSpeedLimitCritic。
//
// ================== 为什么不能直接照抄社区那份 ==================
// 1. 那份是 Jazzy/rolling 分支的代码，MPPI 内部已从 xtensor 迁到 Eigen；
//    本机是 humble（nav2_mppi_controller 1.1.20），CriticData 里全是
//    xt::xtensor<float,2>。整层张量运算必须重写，不是改改包名的事。
// 2. 那份只有"按 |wz| 限速"的反馈项。反馈项**只在 rollout 自己已经打算转弯时**
//    才有读数；一条"保持高速、先不转"的 rollout 的 |wz| 很小、免罚，
//    而它恰恰是过弯冲出去的那条。所以必须补一项从**路径曲率**读出来的前馈项。
//
// ================== 两项各自管什么 ==================
//   项1 反馈（lateral-accel）: 罚 rollout 自身的 v*|wz| 超出预算的部分。
//        它保证"真要这么转，就必须慢下来"，是运动学自洽的一项。
//   项2 前馈（path-curvature）: 从**路径**未来一段的最大 Menger 曲率算出速度上限，
//        罚任何超过该上限的 rollout —— 与它转不转无关。
//        它才是"入弯前就减速"的来源。
//
// ================== 与既有 critic 的关系（重要，别调错旋钮）==================
// · TwirlingCritic：罚 mean|wz|，**无条件**压制转向。本仓库刻意把它从 10.0 降到 1.0，
//   因为 motion_model 改 DiffDrive 后"对齐路径必须靠转向"，压转向就是压对齐。
//   本 critic 罚的是 v*|wz|，低速转弯代价为 0 —— 因此它**不会**把那个问题带回来，
//   也**不是**把 Twirling 调回 10.0 的替代品。两者不要一起加。
// · ConstraintCritic：罚超出 vx_max/wz_max 的**箱型**约束。本 critic 罚的是箱内
//   那条耦合曲线。两者正交，各管一件事。
// · PathAngleCritic：±57.3°(1.0rad) 死区型，死区内无航向梯度（见 memory
//   mppi-heading-is-free-within-57deg-deadband）。本 critic 是**速度** critic，
//   一点也不改善那个死区，别指望它顺手解决航向问题。
//
// ================== 标定前必须知道的一条算术 ==================
// 本仓库当前跑的是 max_linear_speed:=0.2（按轴），即 v <= 0.2；wz_max = 2.0。
// 所以侧向加速度的**物理上界**是 0.2 * 2.0 = 0.4 m/s^2。
// ⇒ a_lat_max >= 0.4 时本 critic 是**恒等于零的空操作**，一行日志都不会有。
//   默认值取 0.35 就是为了在 0.2m/s 档位下仍能咬住；解除限速跑 1.0m/s 时
//   要重新标定（1.0*2.0 = 2.0 m/s^2 的上界下，0.35 会变得非常激进）。
// 这个数是**待实测标定项**，验收看实测的"大曲率处 v 与 wz 的联合分布"，
// 不是看这个数字本身 —— 同 cost_travel_multiplier 的口径。

#ifndef ASTRIBOT_S1_PATH_TRACKING__CURVATURE_SPEED_LIMIT_CRITIC_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__CURVATURE_SPEED_LIMIT_CRITIC_HPP_

#include "astribot_s1_path_tracking/curvature_speed_math.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

// ================== 为什么这个类不在 astribot_s1_path_tracking 命名空间里 ==================
// nav2 humble 的 mppi::CriticManager::getFullName() 是**无条件**拼前缀的：
//   getFullName(name) == "mppi::critics::" + name
// 反汇编确认（libmppi_controller.so，getFullName 是一条直线、无分支，字面量就是
// "mppi::critics::"）。而 pluginlib 的查找名就是 critics.xml 里的 type 全限定名。
// ⇒ 类若留在自己包的命名空间，yaml 里无论怎么写 critics 列表都查不到，
//   运行时报 "Failed to create critic ... not found"，编译期毫无提示。
// 所以自定义 MPPI critic 在 humble 上**必须**声明进 mppi::critics。
// 纯函数层仍在 astribot_s1_path_tracking::curvature_speed 下，不受影响。
namespace mppi
{
namespace critics
{

/// 纯函数层的短别名 —— 那一层仍在本仓库自己的命名空间下，只有 critic 类被迫外迁。
namespace curvature_speed = astribot_s1_path_tracking::curvature_speed;

/// MPPI critic：按轨迹角速度与路径曲率做显式限速（vx = f(kappa) 耦合减速）。
class CurvatureSpeedLimitCritic : public mppi::critics::CriticFunction
{
public:
  void initialize() override;
  void score(mppi::CriticData & data) override;

protected:
  /// 项1 权重（罚 v*|wz| 超预算量，已归一化到 a_lat_max，量纲无关）。
  float weight_{0.0f};
  /// 项2 权重（罚速度超出路径曲率允许值的量，单位 m/s）。置 0 = 关掉前馈项。
  float path_weight_{0.0f};
  /// 代价幂次。本 critic 建议 2：越限越狠，而软带内仍保持近线性的温和梯度。
  unsigned int power_{1};

  /// 前馈项沿路径向前看的弧长 [m]。
  float path_lookahead_dist_{1.5f};
  /// 距目标小于此值时整个 critic 让位给 GoalCritic/接近段限速，不再干预。
  float threshold_to_consider_{0.5f};

  curvature_speed::Limits limits_;
  /// validate() 失败时置位：只打一次错误日志，然后本 critic 全程不打分。
  bool config_invalid_{false};
};

}  // namespace critics
}  // namespace mppi

#endif  // ASTRIBOT_S1_PATH_TRACKING__CURVATURE_SPEED_LIMIT_CRITIC_HPP_
