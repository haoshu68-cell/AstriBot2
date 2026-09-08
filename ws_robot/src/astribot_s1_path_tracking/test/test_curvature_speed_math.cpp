// Copyright 2026 Astribot. Apache-2.0.
// curvature_speed_math 的离线单测。只链纯函数层，不需要 ROS 运行时。
//
// 这里刻意**不**测"权重取某个值时代价是多少"—— 那种断言等于把当前配置抄一遍，
// 配置一改就红，且抓不到任何真 bug（见 memory tests-can-lock-in-the-bug-they-should-catch）。
// 测的是不变量：恒等式、单调性、边界、退化输入。

#include <cmath>
#include <string>
#include <vector>

#include "astribot_s1_path_tracking/curvature_speed_math.hpp"
#include "gtest/gtest.h"

using astribot_s1_path_tracking::curvature_speed::Limits;
using astribot_s1_path_tracking::curvature_speed::allowedSpeed;
using astribot_s1_path_tracking::curvature_speed::lateralAccelExcess;
using astribot_s1_path_tracking::curvature_speed::maxCurvatureInWindow;
using astribot_s1_path_tracking::curvature_speed::mengerCurvature;
using astribot_s1_path_tracking::curvature_speed::softSpeedCap;
using astribot_s1_path_tracking::curvature_speed::speedExcessOverPathCap;
using astribot_s1_path_tracking::curvature_speed::trajectoryCurvature;
using astribot_s1_path_tracking::curvature_speed::validate;

namespace
{
Limits nominal()
{
  Limits l;
  l.a_lat_max = 0.35;
  l.soft_ratio = 0.6;
  l.v_min_turn = 0.08;
  l.v_max = 1.0;
  return l;
}
}  // namespace

// ============ 核心恒等式：a_lat = v*|wz| 与 v <= sqrt(a/kappa) 必须是同一判据 ============
// 这是整个 critic 的设计依据。打分走左式、标定说右式，两者若不等价，
// 就会出现"日志说允许 0.4m/s，实际罚在 0.3m/s"这种无法反推的现象。
TEST(CurvatureSpeedMath, LateralAccelHingeEquivalentToSpeedCap)
{
  const auto lim = nominal();
  // 取硬边界做等价性检验：a_lat_max 处两式必须同时成立。
  for (double kappa : {0.2, 0.5, 1.0, 2.0, 5.0}) {
    const double v_allow = allowedSpeed(kappa, lim);
    if (v_allow <= lim.v_min_turn || v_allow >= lim.v_max) {
      continue;   // 被夹住的区段不适用等价性（夹取本身改变了判据）
    }
    const double wz = kappa * v_allow;          // 该速度下的角速度
    EXPECT_NEAR(v_allow * wz, lim.a_lat_max, 1e-9) << "kappa=" << kappa;
    // 恰在硬边界：归一化超出量应为 (1 - soft_ratio)
    EXPECT_NEAR(
      lateralAccelExcess(v_allow, wz, lim), 1.0 - lim.soft_ratio, 1e-9) << "kappa=" << kappa;
  }
}

// 软带存在：还没到硬边界时代价必须已经 > 0，否则未越限区间梯度恒为 0。
// 本仓库已两次栽在"判据区间内无梯度"上，这条是专门守它的。
TEST(CurvatureSpeedMath, SoftBandGivesGradientBeforeHardLimit)
{
  const auto lim = nominal();
  const double a_mid = 0.5 * (lim.soft_ratio * lim.a_lat_max + lim.a_lat_max);
  const double v = 0.5;
  const double wz = a_mid / v;
  const double e = lateralAccelExcess(v, wz, lim);
  EXPECT_GT(e, 0.0);
  EXPECT_LT(e, 1.0 - lim.soft_ratio);

  // 软带以下必须严格为 0（低速转弯免罚 —— 这正是它与 TwirlingCritic 的区别）。
  EXPECT_DOUBLE_EQ(lateralAccelExcess(0.05, 2.0, lim), 0.0);   // a_lat=0.10 < 0.21
}

// 原地旋转（v=0，wz 拉满）必须零代价：三段式的 ALIGN_START 段全程如此。
TEST(CurvatureSpeedMath, InPlaceRotationIsFree)
{
  const auto lim = nominal();
  EXPECT_DOUBLE_EQ(lateralAccelExcess(0.0, 2.0, lim), 0.0);
  EXPECT_DOUBLE_EQ(trajectoryCurvature(0.0, 2.0), 0.0);
  EXPECT_DOUBLE_EQ(trajectoryCurvature(1e-9, 2.0), 0.0);
}

// 单调性：曲率越大允许速度越小，且永不越出 [v_min_turn, v_max]。
TEST(CurvatureSpeedMath, AllowedSpeedMonotoneAndClamped)
{
  const auto lim = nominal();
  double prev = allowedSpeed(0.0, lim);
  EXPECT_DOUBLE_EQ(prev, lim.v_max);           // 直路 = 满速
  for (double kappa = 0.1; kappa <= 20.0; kappa += 0.1) {
    const double v = allowedSpeed(kappa, lim);
    EXPECT_LE(v, prev + 1e-12) << "kappa=" << kappa;
    EXPECT_GE(v, lim.v_min_turn);
    EXPECT_LE(v, lim.v_max);
    prev = v;
  }
  EXPECT_DOUBLE_EQ(allowedSpeed(1e6, lim), lim.v_min_turn);   // 极弯被地板夹住
}

// 软上限必须严格严于硬上限（否则前馈项与反馈项起点不一致）。
TEST(CurvatureSpeedMath, SoftCapIsStricterThanHardCap)
{
  const auto lim = nominal();
  for (double kappa : {0.5, 1.0, 2.0}) {
    const double hard = allowedSpeed(kappa, lim);
    const double soft = softSpeedCap(kappa, lim);
    EXPECT_LE(soft, hard);
    if (soft > lim.v_min_turn && hard < lim.v_max) {
      EXPECT_LT(soft, hard) << "kappa=" << kappa;
      // sqrt 关系：soft/hard 应为 sqrt(soft_ratio)
      EXPECT_NEAR(soft / hard, std::sqrt(lim.soft_ratio), 1e-9);
    }
  }
}

TEST(CurvatureSpeedMath, SpeedExcessOverPathCap)
{
  const auto lim = nominal();
  const double kappa = 1.0;                       // R = 1m
  const double cap = softSpeedCap(kappa, lim);
  EXPECT_DOUBLE_EQ(speedExcessOverPathCap(cap * 0.5, kappa, lim), 0.0);
  EXPECT_NEAR(speedExcessOverPathCap(cap + 0.2, kappa, lim), 0.2, 1e-9);
  // 直路（kappa=0）时上限就是 v_max，低于它不罚。
  EXPECT_DOUBLE_EQ(speedExcessOverPathCap(lim.v_max, 0.0, lim), 0.0);
}

// ============ Menger 曲率 ============
TEST(CurvatureSpeedMath, MengerCurvatureOnKnownCircle)
{
  // 半径 R 的圆上取三点，曲率应为 1/R。取小角度间隔以贴近离散路径。
  for (double R : {0.5, 1.0, 2.5}) {
    const double dth = 0.05;
    double px[3], py[3];
    for (int i = 0; i < 3; ++i) {
      const double th = i * dth;
      px[i] = R * std::cos(th);
      py[i] = R * std::sin(th);
    }
    EXPECT_NEAR(mengerCurvature(px[0], py[0], px[1], py[1], px[2], py[2]), 1.0 / R, 1e-6)
      << "R=" << R;
  }
}

TEST(CurvatureSpeedMath, MengerCurvatureStraightAndDegenerate)
{
  EXPECT_DOUBLE_EQ(mengerCurvature(0, 0, 1, 0, 2, 0), 0.0);        // 直线
  // 重复点：nav2 的 Path 里确实存在，不能返回 inf/nan。
  const double k = mengerCurvature(1.0, 1.0, 1.0, 1.0, 2.0, 2.0);
  EXPECT_DOUBLE_EQ(k, 0.0);
  EXPECT_FALSE(std::isnan(k));
}

// 窗口取 max 而不是 mean：直路后接急弯时，需要减速的是那个急弯。
TEST(CurvatureSpeedMath, WindowTakesMaxNotMean)
{
  std::vector<double> xs, ys;
  for (int i = 0; i < 20; ++i) {          // 先 0.95m 直路（0.05m 一点）
    xs.push_back(0.05 * i);
    ys.push_back(0.0);
  }
  const double R = 0.5, x0 = xs.back(), y0 = ys.back();
  for (int i = 1; i <= 20; ++i) {         // 再接 R=0.5m 的弯
    const double th = i * 0.1;
    xs.push_back(x0 + R * std::sin(th));
    ys.push_back(y0 + R * (1.0 - std::cos(th)));
  }
  // 看得足够远 -> 抓到 1/R = 2.0
  EXPECT_NEAR(maxCurvatureInWindow(xs, ys, 0, 5.0), 1.0 / R, 0.05);
  // 只看 0.3m -> 还在直路里，应为 0
  EXPECT_NEAR(maxCurvatureInWindow(xs, ys, 0, 0.3), 0.0, 1e-9);
}

TEST(CurvatureSpeedMath, WindowDegenerateInputs)
{
  std::vector<double> xs{0.0, 1.0}, ys{0.0, 0.0};
  EXPECT_DOUBLE_EQ(maxCurvatureInWindow(xs, ys, 0, 1.0), 0.0);     // 不足 3 点
  std::vector<double> e;
  EXPECT_DOUBLE_EQ(maxCurvatureInWindow(e, e, 0, 1.0), 0.0);       // 空路径
  std::vector<double> xs3{0.0, 1.0, 2.0}, ys3{0.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(maxCurvatureInWindow(xs3, ys3, 5, 1.0), 0.0);   // 锚点越界
}

// ============ 参数自检 ============
TEST(CurvatureSpeedMath, ValidateRejectsIllegalConfig)
{
  std::string why;
  auto lim = nominal();
  EXPECT_TRUE(validate(lim, why)) << why;

  lim = nominal(); lim.a_lat_max = 0.0;
  EXPECT_FALSE(validate(lim, why));
  EXPECT_NE(why.find("a_lat_max"), std::string::npos);

  lim = nominal(); lim.soft_ratio = 0.0;
  EXPECT_FALSE(validate(lim, why));
  lim = nominal(); lim.soft_ratio = 1.5;
  EXPECT_FALSE(validate(lim, why));
  lim = nominal(); lim.soft_ratio = 1.0;
  EXPECT_TRUE(validate(lim, why)) << "1.0 合法（退化成硬 hinge），只是不推荐";

  lim = nominal(); lim.v_min_turn = 2.0;   // > v_max
  EXPECT_FALSE(validate(lim, why));
  lim = nominal(); lim.v_max = 0.0;
  EXPECT_FALSE(validate(lim, why));
}
