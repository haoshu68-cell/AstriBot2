// Copyright 2026 Astribot.
//
// livox_custom_convert 的纯逻辑测试。不需要 ROS / 雷达 / 仿真。
//
// 重点覆盖两件在实机上量到的事：
//   1. 无效点堆在**外参平移处**而不是原点（back 那一团在 0.496m 外），
//      所以「距原点很近」这条规则滤不掉它 —— 这正是现有 range_min=0.35 的漏洞。
//   2. timebase 与 header.stamp 谁大都有可能，无符号相减会绕回成天文数字。
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "astribot_s1_autonomy/livox_custom_convert.hpp"

using astribot_s1_autonomy::ConvertedPoint;
using astribot_s1_autonomy::LivoxConvertConfig;
using astribot_s1_autonomy::LivoxConvertConfigError;
using astribot_s1_autonomy::LivoxConvertStats;
using astribot_s1_autonomy::LivoxPoint;
using astribot_s1_autonomy::convert_frame;
using astribot_s1_autonomy::should_drop;

namespace
{
LivoxPoint mk(float x, float y, float z, std::uint8_t tag = 0,
              std::uint32_t offset = 0, std::uint8_t line = 0,
              std::uint8_t refl = 7)
{
  LivoxPoint p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.tag = tag;
  p.offset_time = offset;
  p.line = line;
  p.reflectivity = refl;
  return p;
}

/// front 那一路：外参恒等，无效点在原点。
LivoxConvertConfig front_cfg()
{
  LivoxConvertConfig c;
  c.null_x = 0.0F;
  c.null_y = 0.0F;
  c.null_z = 0.0F;
  return c;
}

/// back 那一路：外参已在设备内应用，无效点堆在平移量处。
/// 数值取自厂商 MID360_config.json 的 lidar[1]：x=1mm y=-496mm z=84mm。
LivoxConvertConfig back_cfg()
{
  LivoxConvertConfig c;
  c.null_x = 0.001F;
  c.null_y = -0.496F;
  c.null_z = 0.084F;
  return c;
}
}  // namespace

// ── 无效点剔除 ──────────────────────────────────────────────────────
TEST(NullPoint, FrontNullAtOriginIsDropped)
{
  LivoxConvertStats s;
  EXPECT_TRUE(should_drop(mk(0.0F, 0.0F, 0.0F), front_cfg(), &s));
  EXPECT_EQ(s.dropped_null, 1U);
}

TEST(NullPoint, BackNullAtExtrinsicTranslationIsDropped)
{
  LivoxConvertStats s;
  EXPECT_TRUE(should_drop(mk(0.001F, -0.496F, 0.084F), back_cfg(), &s));
  EXPECT_EQ(s.dropped_null, 1U);
}

TEST(NullPoint, BackNullIsFarFromOriginSoRangeMinCannotCatchIt)
{
  // 这条测试是整个文件的理由：back 的无效点距原点 0.4968m，
  // 大于 livox_preprocess_node 的 range_min=0.35，所以球面距离门限滤不掉它。
  // 若哪天有人把无效点剔除删掉、改回「靠 range_min 就够了」，这条会挂。
  const float r = std::sqrt(0.001F * 0.001F + 0.496F * 0.496F + 0.084F * 0.084F);
  EXPECT_GT(r, 0.35F) << "前提变了：back 无效点不再在 range_min 之外";

  LivoxConvertConfig c = back_cfg();
  c.null_radius = 0.0F;      // 关掉无效点剔除
  c.min_range = 0.35F;       // 只靠球面距离
  LivoxConvertStats s;
  EXPECT_FALSE(should_drop(mk(0.001F, -0.496F, 0.084F), c, &s))
    << "range_min 竟然滤掉了 back 的无效点 —— 那本文件的前提不成立";
}

TEST(NullPoint, OriginIsNotDroppedByBackConfig)
{
  // back 那一路配了非零无效点位置，(0,0,0) 就不该被当成无效点。
  // 反过来说：两路配串了会各自漏掉自己的假点团。
  LivoxConvertStats s;
  EXPECT_FALSE(should_drop(mk(0.0F, 0.0F, 0.0F), back_cfg(), &s));
  EXPECT_EQ(s.dropped_null, 0U);
}

TEST(NullPoint, JustOutsideRadiusIsKept)
{
  LivoxConvertConfig c = front_cfg();
  c.null_radius = 0.005F;
  LivoxConvertStats s;
  EXPECT_TRUE(should_drop(mk(0.004F, 0.0F, 0.0F), c, &s));
  EXPECT_FALSE(should_drop(mk(0.006F, 0.0F, 0.0F), c, &s));
}

TEST(NullPoint, ZeroRadiusDisablesTheCheck)
{
  LivoxConvertConfig c = front_cfg();
  c.null_radius = 0.0F;
  LivoxConvertStats s;
  EXPECT_FALSE(should_drop(mk(0.0F, 0.0F, 0.0F), c, &s));
}

// ── NaN / Inf ───────────────────────────────────────────────────────
TEST(NonFinite, NanAndInfAreDroppedAndCountedSeparately)
{
  LivoxConvertStats s;
  const float nan = std::nan("");
  const float inf = std::numeric_limits<float>::infinity();
  EXPECT_TRUE(should_drop(mk(nan, 0.0F, 0.0F), front_cfg(), &s));
  EXPECT_TRUE(should_drop(mk(0.0F, inf, 0.0F), front_cfg(), &s));
  EXPECT_TRUE(should_drop(mk(0.0F, 0.0F, -inf), front_cfg(), &s));
  EXPECT_EQ(s.dropped_nonfinite, 3U);
  // NaN 不该被算成无效点或距离越界 —— 统计量要指向根本原因
  EXPECT_EQ(s.dropped_null, 0U);
  EXPECT_EQ(s.dropped_range, 0U);
}

// ── 距离门限 ────────────────────────────────────────────────────────
TEST(Range, MinAndMaxAreExclusiveOfTheirOwnBoundary)
{
  LivoxConvertConfig c = front_cfg();
  c.null_radius = 0.0F;
  c.min_range = 1.0F;
  c.max_range = 5.0F;
  LivoxConvertStats s;
  EXPECT_TRUE(should_drop(mk(0.99F, 0.0F, 0.0F), c, &s));
  EXPECT_FALSE(should_drop(mk(1.01F, 0.0F, 0.0F), c, &s));
  EXPECT_FALSE(should_drop(mk(4.99F, 0.0F, 0.0F), c, &s));
  EXPECT_TRUE(should_drop(mk(5.01F, 0.0F, 0.0F), c, &s));
  EXPECT_EQ(s.dropped_range, 2U);
}

TEST(Range, DisabledByDefaultSoWeDoNotStackTwoThresholds)
{
  // 默认不做距离过滤：那是 livox_preprocess_node 的职责。
  // 两处各有一个门限、只改一处，是本项目吃过的亏（两层限速串联叠乘）。
  //
  // 取样点要落在无效点球（默认半径 5mm）**之外**：我第一版用了 (0.001,0,0)，
  // 那个点距原点 1mm，会被默认的无效点规则正确地丢掉，于是这条测试挂了 ——
  // 挂的是测试而不是代码。顺带说明物理上也不冲突：MID360 盲区 0.5m，
  // 1mm 处根本不可能有真实回波。
  const LivoxConvertConfig c;
  EXPECT_EQ(c.min_range, 0.0F);
  EXPECT_EQ(c.max_range, 0.0F);
  LivoxConvertStats s;
  EXPECT_FALSE(should_drop(mk(0.02F, 0.0F, 0.0F), c, &s));
  EXPECT_FALSE(should_drop(mk(500.0F, 0.0F, 0.0F), c, &s));
  EXPECT_EQ(s.dropped_range, 0U);
}

// ── tag 噪点位 ──────────────────────────────────────────────────────
TEST(Tag, SpatialNoiseBitsFilteredWhenThresholdSet)
{
  LivoxConvertConfig c = front_cfg();
  c.null_radius = 0.0F;
  c.max_spatial_noise = 2U;   // 丢弃 bit0-1 >= 2
  LivoxConvertStats s;
  EXPECT_FALSE(should_drop(mk(1.0F, 0.0F, 0.0F, 0x00U), c, &s));
  EXPECT_FALSE(should_drop(mk(1.0F, 0.0F, 0.0F, 0x01U), c, &s));
  EXPECT_TRUE(should_drop(mk(1.0F, 0.0F, 0.0F, 0x02U), c, &s));
  EXPECT_TRUE(should_drop(mk(1.0F, 0.0F, 0.0F, 0x03U), c, &s));
  EXPECT_EQ(s.dropped_noise, 2U);
}

TEST(Tag, DefaultThresholdKeepsEverything)
{
  const LivoxConvertConfig c;
  EXPECT_EQ(c.max_spatial_noise, 4U);
  LivoxConvertStats s;
  for (std::uint8_t tag = 0; tag < 4U; ++tag) {
    EXPECT_FALSE(should_drop(mk(1.0F, 0.0F, 0.0F, tag), c, &s))
      << "tag=" << static_cast<int>(tag);
  }
}

TEST(Tag, HigherBitsOfTagAreIgnored)
{
  // tag 的高位是强度置信度与回波类型，不该影响空间噪点判定。
  LivoxConvertConfig c = front_cfg();
  c.null_radius = 0.0F;
  c.max_spatial_noise = 1U;
  LivoxConvertStats s;
  EXPECT_FALSE(should_drop(mk(1.0F, 0.0F, 0.0F, 0xFCU), c, &s));  // bit0-1 = 0
}

// ── 时间换算 ────────────────────────────────────────────────────────
TEST(Time, OffsetIsRelativeToHeaderStampNotTimebase)
{
  std::vector<LivoxPoint> in{mk(1.0F, 0.0F, 0.0F, 0U, 1000000U)};  // +1ms
  std::vector<ConvertedPoint> out;
  // timebase 比 header 早 10ms
  const std::uint64_t timebase = 1'000'000'000ULL;
  const std::uint64_t header = 1'010'000'000ULL;
  convert_frame(in, timebase, header, front_cfg(), &out);
  ASSERT_EQ(out.size(), 1U);
  // (1.000s + 1ms) - 1.010s = -9ms
  EXPECT_NEAR(out[0].time, -0.009F, 1e-6F);
}

TEST(Time, HeaderLaterThanTimebaseGivesNegativeNotHugePositive)
{
  std::vector<LivoxPoint> in{mk(1.0F, 0.0F, 0.0F)};
  std::vector<ConvertedPoint> out;
  convert_frame(in, 5ULL, 1'000'000'000ULL, front_cfg(), &out);
  ASSERT_EQ(out.size(), 1U);
  EXPECT_LT(out[0].time, 0.0F) << "header 晚于 timebase 时必须是负偏移";
  EXPECT_NEAR(out[0].time, -1.0F, 1e-3F);
}

TEST(Time, TheActualHazardIsUnsignedStraightToFloat)
{
  // 这条测的是**算术本身**，用来把「为什么全程用 int64」钉在文件里。
  //
  // 我原先以为危险在于「无符号相减会绕回」。实测不是：补码下 uint64 相减
  // 再转回 int64 恰好还原正确的负值，两种写法给出同一个 -999999995 ——
  // 所以那个变异体（M2）能通过全部测试，我的原注释是错的。
  //
  // 真正会出事的是把无符号差值**直接转成浮点**：得到约 1.8e10 秒（约 585 年），
  // 不报错，下游只会看到「这一点来自遥远未来」。
  const std::uint64_t timebase = 5ULL;
  const std::uint64_t header = 1'000'000'000ULL;

  const std::int64_t via_int64 =
    static_cast<std::int64_t>(timebase) - static_cast<std::int64_t>(header);
  const std::int64_t via_unsigned_then_int64 =
    static_cast<std::int64_t>(timebase - header);
  EXPECT_EQ(via_int64, via_unsigned_then_int64) << "补码下这两者应当相等";

  const double hazard = static_cast<double>(timebase - header) * 1e-9;
  EXPECT_GT(hazard, 1.0e10) << "无符号直转浮点应当给出天文数字";
  EXPECT_NEAR(static_cast<double>(via_int64) * 1e-9, -1.0, 1e-6);
}

TEST(Time, ZeroOffsetWithMatchingStampIsZero)
{
  std::vector<LivoxPoint> in{mk(1.0F, 0.0F, 0.0F)};
  std::vector<ConvertedPoint> out;
  convert_frame(in, 12345ULL, 12345ULL, front_cfg(), &out);
  ASSERT_EQ(out.size(), 1U);
  EXPECT_FLOAT_EQ(out[0].time, 0.0F);
}

// ── 整帧转换 ────────────────────────────────────────────────────────
TEST(Frame, FieldsAreCarriedThrough)
{
  std::vector<LivoxPoint> in{mk(1.5F, -2.5F, 0.25F, 0x00U, 0U, 3U, 200U)};
  std::vector<ConvertedPoint> out;
  convert_frame(in, 0ULL, 0ULL, front_cfg(), &out);
  ASSERT_EQ(out.size(), 1U);
  EXPECT_FLOAT_EQ(out[0].x, 1.5F);
  EXPECT_FLOAT_EQ(out[0].y, -2.5F);
  EXPECT_FLOAT_EQ(out[0].z, 0.25F);
  EXPECT_FLOAT_EQ(out[0].intensity, 200.0F);
  EXPECT_EQ(out[0].ring, 3U);
}

TEST(Frame, StatsAddUp)
{
  std::vector<LivoxPoint> in{
    mk(0.0F, 0.0F, 0.0F),                    // 无效点
    mk(1.0F, 0.0F, 0.0F),                    // 保留
    mk(std::nan(""), 0.0F, 0.0F),            // NaN
    mk(2.0F, 0.0F, 0.0F),                    // 保留
  };
  std::vector<ConvertedPoint> out;
  const LivoxConvertStats s = convert_frame(in, 0ULL, 0ULL, front_cfg(), &out);
  EXPECT_EQ(s.total, 4U);
  EXPECT_EQ(s.kept, 2U);
  EXPECT_EQ(s.dropped_null, 1U);
  EXPECT_EQ(s.dropped_nonfinite, 1U);
  EXPECT_EQ(s.kept + s.dropped_null + s.dropped_range + s.dropped_noise +
            s.dropped_nonfinite, s.total);
  EXPECT_NEAR(s.kept_ratio(), 0.5, 1e-9);
}

TEST(Frame, RealisticBackFrameDropsAboutOneThird)
{
  // 复现实测比例：back 约 34% 的点是无效点。
  std::vector<LivoxPoint> in;
  for (int i = 0; i < 100; ++i) {
    if (i < 34) {
      in.push_back(mk(0.001F, -0.496F, 0.084F));
    } else {
      in.push_back(mk(1.0F + static_cast<float>(i) * 0.01F, 0.5F, 0.0F));
    }
  }
  std::vector<ConvertedPoint> out;
  const LivoxConvertStats s = convert_frame(in, 0ULL, 0ULL, back_cfg(), &out);
  EXPECT_EQ(s.dropped_null, 34U);
  EXPECT_EQ(s.kept, 66U);
}

TEST(Frame, EmptyInputIsNotAnError)
{
  std::vector<ConvertedPoint> out;
  const LivoxConvertStats s = convert_frame({}, 0ULL, 0ULL, front_cfg(), &out);
  EXPECT_EQ(s.total, 0U);
  EXPECT_EQ(s.kept, 0U);
  EXPECT_EQ(s.kept_ratio(), 0.0);
  EXPECT_TRUE(out.empty());
}

TEST(Frame, OutputIsClearedBetweenCalls)
{
  std::vector<ConvertedPoint> out;
  convert_frame({mk(1.0F, 0.0F, 0.0F)}, 0ULL, 0ULL, front_cfg(), &out);
  convert_frame({mk(2.0F, 0.0F, 0.0F)}, 0ULL, 0ULL, front_cfg(), &out);
  ASSERT_EQ(out.size(), 1U) << "上一帧的点残留了";
  EXPECT_FLOAT_EQ(out[0].x, 2.0F);
}

// ── 配置校验 ────────────────────────────────────────────────────────
TEST(Config, DefaultIsValid)
{
  EXPECT_NO_THROW(LivoxConvertConfig().validate());
}

TEST(Config, MaxRangeBelowMinRangeIsRejected)
{
  LivoxConvertConfig c;
  c.min_range = 5.0F;
  c.max_range = 1.0F;
  EXPECT_THROW(c.validate(), LivoxConvertConfigError);
}

TEST(Config, NonFiniteNullPositionIsRejected)
{
  LivoxConvertConfig c;
  c.null_x = std::nan("");
  EXPECT_THROW(c.validate(), LivoxConvertConfigError);
}

TEST(Config, NegativeRadiusIsRejected)
{
  LivoxConvertConfig c;
  c.null_radius = -0.01F;
  EXPECT_THROW(c.validate(), LivoxConvertConfigError);
}

TEST(Config, OutOfRangeNoiseThresholdIsRejected)
{
  LivoxConvertConfig c;
  c.max_spatial_noise = 5U;
  EXPECT_THROW(c.validate(), LivoxConvertConfigError);
}

TEST(Config, ConvertFrameValidatesBeforeDoingWork)
{
  LivoxConvertConfig c;
  c.min_range = 5.0F;
  c.max_range = 1.0F;
  std::vector<ConvertedPoint> out;
  EXPECT_THROW(convert_frame({mk(1.0F, 0.0F, 0.0F)}, 0ULL, 0ULL, c, &out),
               LivoxConvertConfigError);
}
