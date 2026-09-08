// Copyright 2026 Astribot. Apache-2.0.
//
// 往 nav2 自己持有的 costs 张量里累加代价的**唯一**允许写法。
//
// ================== 为什么不能写 data.costs += expr ==================
// 那句话（stock nav2 critic 都是这么写的）在**我们自己的**编译单元里会打死
// controller_server，SIGSEGV。实测证据链：
//   · 崩溃栈顶在 mppi::Optimizer::updateControlSequence() 里，不在我们的 score() 里；
//   · 出错指令 vmovaps (%rax),%ymm1 —— 要求 32 字节对齐的 AVX 载入；
//     rax = 0x…db0 即 16 mod 32（未对齐），si_addr = 0x0 且地址本身合法，
//     是典型的"对齐型 #GP"，不是野指针；
//   · rdx = rsi = 2000 = batch_size ⇒ 出事的缓冲区就是 costs_。
// 机制：xtensor 的 operator+= 走 computed-assign —— 先在**调用方**编译单元里
// 分配一块 temporary_type，再 move 进容器。于是 nav2 的 costs_ 缓冲区被换成
// 我们这边分配的那块。而：
//   · nav2 的 libmppi_controller.so 编译时带 -DXTENSOR_USE_XSIMD（+ AVX2），
//     XTENSOR_DEFAULT_ALIGNMENT = 32，它自己的代码按 32 字节对齐去读 costs_；
//   · 我们这边没有那个宏，容器用 std::allocator，只有 malloc 的 16 字节保证。
// ⇒ 换进去的缓冲区约一半概率是 16 mod 32，nav2 下一拍 vmovaps 就崩。
// 双编译单元探针已复现："我方 TU 里一句 += 之后 data() 指针改变"。
//
// ================== 为什么不是去 CMake 里对齐编译参数 ==================
// 靠加 -DXTENSOR_USE_XSIMD -march=native 去"凑"成一样是不可靠的：对齐值取自
// xsimd::default_arch，随架构而变（本开发机 AVX2 → 32，实机 aarch64 NEON → 16）。
// 猜错不报错，只是换成另一半概率崩。原地累加与 nav2 用什么参数编译**无关**，
// 所以选它。
//
// ================== 代价 ==================
// noalias 版不分配临时缓冲，直接逐元素写进对方的缓冲区（写指令由我们这边生成，
// 用的是非对齐 store，对方缓冲区对齐更严也无妨）。规模是 batch(2000) 个 float 加法，
// 与 rollout 张量（2000×56）比可以忽略。

#ifndef ASTRIBOT_S1_PATH_TRACKING__COST_ACCUMULATE_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__COST_ACCUMULATE_HPP_

#include <xtensor/xtensor.hpp>
#include <xtensor/xnoalias.hpp>

namespace astribot_s1_path_tracking
{
namespace cost_accumulate
{

/// 原地把 term 累加进 costs。**不得**改成 costs += term（理由见文件头）。
/// 形状不一致时直接返回 false 不写：宁可这一项不生效，也不能越界写别人的缓冲区。
template<class E>
inline bool accumulateInPlace(xt::xtensor<float, 1> & costs, const E & term)
{
  if (costs.shape(0) != term.shape(0)) {
    return false;
  }
  xt::noalias(costs) += term;
  return true;
}

}  // namespace cost_accumulate
}  // namespace astribot_s1_path_tracking

#endif  // ASTRIBOT_S1_PATH_TRACKING__COST_ACCUMULATE_HPP_
