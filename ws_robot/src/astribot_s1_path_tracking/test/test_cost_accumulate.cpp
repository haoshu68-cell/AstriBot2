// Copyright 2026 Astribot. Apache-2.0.
//
// 这个测试守的是**一次真实的 SIGSEGV**：写 data.costs += expr 会让 nav2 持有的
// costs 缓冲区被换成我们这边分配的那块（xtensor computed-assign 先分配临时缓冲
// 再 move 进去），而 nav2 的 libmppi_controller.so 按 32 字节对齐读它，
// 我们这边只有 malloc 的 16 字节保证 ⇒ 约一半概率崩在 vmovaps。
// 完整证据链见 cost_accumulate.hpp 文件头。
//
// 判据只有一条、且是这个 bug 的**唯一**可观测量：data() 指针不变。
// 数值对不对是次要的 —— 数值错了看得见，缓冲区被换掉只在别人的进程里崩。

#include <gtest/gtest.h>

#include <xtensor/xtensor.hpp>
#include <xtensor/xbuilder.hpp>
#include <xtensor/xmath.hpp>

#include "astribot_s1_path_tracking/cost_accumulate.hpp"

using astribot_s1_path_tracking::cost_accumulate::accumulateInPlace;

TEST(CostAccumulate, DoesNotReplaceTheBuffer)
{
  // 形状照抄线上：batch=2000、time=56。
  const std::size_t batch = 2000, steps = 56;
  xt::xtensor<float, 1> costs = xt::zeros<float>({batch});
  const void * before = costs.data();

  xt::xtensor<float, 2> rollout = xt::ones<float>({batch, steps});
  const xt::xtensor<float, 1> term = xt::pow(
    xt::sum(rollout, {1}, xt::evaluation_strategy::immediate) * 0.05f * 6.0f, 2u);

  ASSERT_TRUE(accumulateInPlace(costs, term));
  EXPECT_EQ(before, costs.data())
    << "costs 的数据指针变了 —— 说明累加又走回了 computed-assign（costs += ...），"
       "线上会在 nav2 侧 SIGSEGV";
}

TEST(CostAccumulate, ReferenceBehaviourOfPlainPlusEqualIsToReplaceIt)
{
  // 反面对照：证明上面那条断言不是空的 —— 同一份表达式用 += 就会换缓冲区。
  // 这条测试是**记录 xtensor 的行为**，不是要求它这样；哪天 xtensor 改了，
  // 这条失败即提示"上面那道防护可能已无必要"，而不是有 bug。
  const std::size_t batch = 2000, steps = 56;
  xt::xtensor<float, 1> costs = xt::zeros<float>({batch});
  const void * before = costs.data();

  xt::xtensor<float, 2> rollout = xt::ones<float>({batch, steps});
  costs += xt::pow(
    xt::sum(rollout, {1}, xt::evaluation_strategy::immediate) * 0.05f * 6.0f, 2u);

  EXPECT_NE(before, costs.data());
}

TEST(CostAccumulate, RefusesShapeMismatchInsteadOfWriting)
{
  xt::xtensor<float, 1> costs = xt::zeros<float>({std::size_t(2000)});
  const xt::xtensor<float, 1> wrong = xt::ones<float>({std::size_t(1999)});
  EXPECT_FALSE(accumulateInPlace(costs, wrong));
  EXPECT_FLOAT_EQ(0.0f, costs(0));   // 一个字节都没写进去
}

TEST(CostAccumulate, AccumulatesRatherThanOverwrites)
{
  xt::xtensor<float, 1> costs = {1.0f, 2.0f, 3.0f};
  const xt::xtensor<float, 1> term = {0.5f, 0.5f, 0.5f};
  ASSERT_TRUE(accumulateInPlace(costs, term));
  EXPECT_FLOAT_EQ(1.5f, costs(0));
  EXPECT_FLOAT_EQ(2.5f, costs(1));
  EXPECT_FLOAT_EQ(3.5f, costs(2));
}
