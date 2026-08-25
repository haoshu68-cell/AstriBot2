// Copyright 2026 Astribot
//
// GripperCommander 的离线单测。
//
// 为什么用合成夹爪模型而不是真实 astribot_s1 URDF（与 test_robot_fixture.hpp 同理）：
//   真实 URDF 要 xacro 展开、要 mesh 文件，测试就不再是纯算法测试。更关键的是
//   合成模型的张口宽度**能手算**，断言才写得死；拿真实模型只能"跑出来多少就断言多少"，
//   那种测试改坏了也照样绿。
//
// 合成夹爪（最小平行夹爪，几何取整数便于手算）：
//
//     palm --(fixed z+0.1)--> tcp          ← 张口沿 tcp 的 x 轴
//       |--jaw_master(prismatic, axis -x, origin x=+0.05)--> pad_left
//       `--jaw_mimic (prismatic, axis +x, origin x=-0.05, mimic master ×1)--> pad_right
//
// 两个指垫都是 0.02 立方体。主动关节走 q 时：
//     pad_left  中心 x =  0.05 - q      占据 [0.04-q, 0.06-q]
//     pad_right 中心 x = -0.05 + q      占据 [-0.06+q, -0.04+q]
//     张口(相向面间距) = (0.04-q) - (-0.04+q) = 0.08 - 2q
// 于是：q=0 张口 0.08 m；q=0.035 张口 0.01 m。全是可以口算的数。
//
// 刻意用 prismatic 而不是 revolute：真机是四连杆转动，但被测代码不关心关节类型，
// 而 prismatic 让张口成为 q 的线性函数，断言可以精确到 1e-9，不用留几何裕度。
//
// 覆盖的是最容易写错、且错了不会当场报错的几件事：
//   1. 开合角有没有真的从 SRDF 的 group_state 读（写死 0/0.93 也能"跑通"）
//   2. 按物体宽度反解抓取角对不对（错了只会在物体上方闭合到空气里）
//   3. 闭合位设过头、两指对穿时能不能被拦住（张口非单调，插值会静默给错角度）

#include <cmath>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "astribot_s1_manipulation/gripper_commander.hpp"

#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace
{

/// 主动关节为 0 时的张口（m）。见文件头的手算。
constexpr double kWidthAtZero = 0.08;
/// 张口对主动关节的变化率（m/rad，本合成模型里是 m/m）。
constexpr double kWidthSlope = -2.0;
/// 正常配置下的闭合角（m）。取 0.035 → 张口 0.01 m，仍在两指对穿之前。
constexpr double kShutNormal = 0.035;
/// 故障注入用的闭合角：0.08 已经越过两指对穿点（q=0.05），张口先减后增。
constexpr double kShutCrossover = 0.08;

double expectedWidth(double q)
{
  return kWidthAtZero + kWidthSlope * q;
}

std::string makeGripperUrdf(double mimic_multiplier)
{
  const std::string inertial =
    "    <inertial><mass value=\"0.05\"/>"
    "<inertia ixx=\"1e-4\" ixy=\"0\" ixz=\"0\" iyy=\"1e-4\" iyz=\"0\" izz=\"1e-4\"/>"
    "</inertial>\n";
  const std::string pad_collision =
    "    <collision><geometry><box size=\"0.02 0.02 0.02\"/></geometry></collision>\n";

  std::string xml;
  xml += "<robot name=\"synthetic_gripper\">\n";
  xml += "  <link name=\"palm\">\n" + inertial + "  </link>\n";
  // tcp 是纯坐标系、无碰撞几何（与真机 tcp_link 一致）。
  xml += "  <link name=\"tcp\"/>\n";
  xml +=
    "  <joint name=\"tcp_joint\" type=\"fixed\">\n"
    "    <origin xyz=\"0 0 0.1\"/>\n"
    "    <parent link=\"palm\"/><child link=\"tcp\"/>\n"
    "  </joint>\n";
  xml += "  <link name=\"pad_left\">\n" + inertial + pad_collision + "  </link>\n";
  xml += "  <link name=\"pad_right\">\n" + inertial + pad_collision + "  </link>\n";
  xml +=
    "  <joint name=\"jaw_master\" type=\"prismatic\">\n"
    "    <origin xyz=\"0.05 0 0.1\"/>\n"
    "    <parent link=\"palm\"/><child link=\"pad_left\"/>\n"
    "    <axis xyz=\"-1 0 0\"/>\n"
    "    <limit effort=\"1\" velocity=\"1\" lower=\"0\" upper=\"0.2\"/>\n"
    "  </joint>\n";
  xml +=
    "  <joint name=\"jaw_mimic\" type=\"prismatic\">\n"
    "    <origin xyz=\"-0.05 0 0.1\"/>\n"
    "    <parent link=\"palm\"/><child link=\"pad_right\"/>\n"
    "    <axis xyz=\"1 0 0\"/>\n"
    "    <limit effort=\"1\" velocity=\"1\" lower=\"-0.2\" upper=\"0.2\"/>\n"
    "    <mimic joint=\"jaw_master\" multiplier=\"" + std::to_string(mimic_multiplier) +
    "\" offset=\"0\"/>\n"
    "  </joint>\n";
  xml += "</robot>\n";
  return xml;
}

/// 合成夹爪 SRDF。
///
/// group_state 的名字刻意不叫 open/closed、值刻意不是真机的 0.0/0.93 ——
/// 实现里只要有一处写死了真机的名字或数值，这些测试就会挂。
std::string makeGripperSrdf(double shut_value)
{
  return
    "<robot name=\"synthetic_gripper\">\n"
    "  <group name=\"synthetic_jaw\">\n"
    "    <joint name=\"jaw_master\"/>\n"
    "  </group>\n"
    "  <group_state name=\"wide\" group=\"synthetic_jaw\">\n"
    "    <joint name=\"jaw_master\" value=\"0.0\"/>\n"
    "  </group_state>\n"
    "  <group_state name=\"shut\" group=\"synthetic_jaw\">\n"
    "    <joint name=\"jaw_master\" value=\"" + std::to_string(shut_value) + "\"/>\n"
    "  </group_state>\n"
    "</robot>\n";
}

moveit::core::RobotModelPtr makeModel(
  double mimic_multiplier = 1.0, double shut_value = kShutNormal)
{
  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(makeGripperUrdf(mimic_multiplier));
  if (urdf_model == nullptr) {
    return nullptr;
  }
  auto srdf_model = std::make_shared<srdf::Model>();
  if (!srdf_model->initString(*urdf_model, makeGripperSrdf(shut_value))) {
    return nullptr;
  }
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

astribot_s1_manipulation::GripperConfig makeConfig()
{
  astribot_s1_manipulation::GripperConfig cfg;
  cfg.group_name = "synthetic_jaw";
  cfg.action_name = "/synthetic_jaw_controller/follow_joint_trajectory";
  cfg.open_state_name = "wide";
  cfg.closed_state_name = "shut";
  cfg.tcp_link = "tcp";
  cfg.left_pad_link = "pad_left";
  cfg.right_pad_link = "pad_right";
  cfg.jaw_axis_in_tcp = {1.0, 0.0, 0.0};
  cfg.grasp_preload_m = 0.0;   // 反解断言先不加预紧，数字才干净
  return cfg;
}

/// GripperCommander 需要一个节点来建 action client 与订阅，
/// 但离线测试不需要有人 spin 它，也不需要控制器在线。
class GripperCommanderTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("gripper_commander_test");
  }

  rclcpp::Node::SharedPtr node_;
};

}  // namespace

using astribot_s1_manipulation::GripperCommander;
using astribot_s1_manipulation::GripperConfig;
using astribot_s1_manipulation::PlanErrorCode;

// ---------------------------------------------------------------------------
// 一、开合角必须来自 SRDF，不能写死
// ---------------------------------------------------------------------------
TEST_F(GripperCommanderTest, ReadsOpenClosedAnglesFromSrdf)
{
  moveit::core::RobotModelPtr model = makeModel();
  ASSERT_NE(model, nullptr);

  GripperCommander commander(node_);
  std::string detail;
  ASSERT_EQ(commander.configure(model, makeConfig(), detail), PlanErrorCode::kSuccess) << detail;

  EXPECT_NEAR(commander.openAngle(), 0.0, 1e-9);
  EXPECT_NEAR(commander.closedAngle(), kShutNormal, 1e-9);
  EXPECT_EQ(commander.jointName(), "jaw_master");
}

TEST_F(GripperCommanderTest, RejectsMissingGroupState)
{
  moveit::core::RobotModelPtr model = makeModel();
  ASSERT_NE(model, nullptr);

  GripperConfig cfg = makeConfig();
  cfg.closed_state_name = "no_such_state";

  GripperCommander commander(node_);
  std::string detail;
  EXPECT_EQ(commander.configure(model, cfg, detail), PlanErrorCode::kInvalidInput);
  EXPECT_NE(detail.find("no_such_state"), std::string::npos) << detail;
}

TEST_F(GripperCommanderTest, RejectsUnknownGroup)
{
  moveit::core::RobotModelPtr model = makeModel();
  ASSERT_NE(model, nullptr);

  GripperConfig cfg = makeConfig();
  cfg.group_name = "not_a_group";

  GripperCommander commander(node_);
  std::string detail;
  EXPECT_EQ(commander.configure(model, cfg, detail), PlanErrorCode::kPlanningGroupNotFound);
}

TEST_F(GripperCommanderTest, RejectsPadLinkWithoutCollisionGeometry)
{
  moveit::core::RobotModelPtr model = makeModel();
  ASSERT_NE(model, nullptr);

  // tcp 是纯坐标系。拿它当指垫必须被明确拒绝，
  // 而不是把包络当成一个点、静默算出一个偏大的张口。
  GripperConfig cfg = makeConfig();
  cfg.left_pad_link = "tcp";

  GripperCommander commander(node_);
  std::string detail;
  EXPECT_EQ(commander.configure(model, cfg, detail), PlanErrorCode::kInvalidInput);
  EXPECT_NE(detail.find("没有碰撞几何"), std::string::npos) << detail;
}

TEST_F(GripperCommanderTest, RejectsUnconfiguredUse)
{
  GripperCommander commander(node_);
  EXPECT_FALSE(commander.isConfigured());

  double angle = 0.0;
  std::string detail;
  EXPECT_EQ(
    commander.graspAngleForWidth(0.06, angle, detail), PlanErrorCode::kNotConfigured);
  EXPECT_EQ(commander.open().code, PlanErrorCode::kNotConfigured);
  EXPECT_EQ(commander.closeToWidth(0.06).code, PlanErrorCode::kNotConfigured);
}

// ---------------------------------------------------------------------------
// 二、张口宽度：正解量出来的必须与手算一致
// ---------------------------------------------------------------------------
TEST_F(GripperCommanderTest, JawWidthMatchesHandComputedGeometry)
{
  moveit::core::RobotModelPtr model = makeModel();
  ASSERT_NE(model, nullptr);

  GripperCommander commander(node_);
  std::string detail;
  ASSERT_EQ(commander.configure(model, makeConfig(), detail), PlanErrorCode::kSuccess) << detail;

  // 张口 = 0.08 - 2q，量的是相向面间距（已扣掉两个指垫各自 0.01 的半厚）。
  // 若实现里错用了两个 link 原点的距离，这里会大 0.02，断言立刻挂。
  for (const double q : {0.0, 0.005, 0.01, 0.02, kShutNormal}) {
    EXPECT_NEAR(commander.jawWidthAtAngle(q), expectedWidth(q), 1e-9) << "q=" << q;
  }
}

TEST_F(GripperCommanderTest, JawWidthClampsOutsideRange)
{
  moveit::core::RobotModelPtr model = makeModel();
  ASSERT_NE(model, nullptr);

  GripperCommander commander(node_);
  std::string detail;
  ASSERT_EQ(commander.configure(model, makeConfig(), detail), PlanErrorCode::kSuccess) << detail;

  // 超出 [open, closed] 一律夹到端点，不做外插 —— 外插出来的张口在物理上不存在。
  EXPECT_NEAR(commander.jawWidthAtAngle(-1.0), expectedWidth(0.0), 1e-9);
  EXPECT_NEAR(commander.jawWidthAtAngle(10.0), expectedWidth(kShutNormal), 1e-9);
}

// ---------------------------------------------------------------------------
// 三、按物体宽度反解抓取角
// ---------------------------------------------------------------------------
TEST_F(GripperCommanderTest, GraspAngleForWidthIsExact)
{
  moveit::core::RobotModelPtr model = makeModel();
  ASSERT_NE(model, nullptr);

  GripperCommander commander(node_);
  std::string detail;
  ASSERT_EQ(commander.configure(model, makeConfig(), detail), PlanErrorCode::kSuccess) << detail;

  // 张口 0.08-2q = width  =>  q = (0.08 - width)/2
  for (const double width : {0.08, 0.06, 0.04, 0.02, 0.01}) {
    double angle = 0.0;
    ASSERT_EQ(commander.graspAngleForWidth(width, angle, detail), PlanErrorCode::kSuccess)
      << "width=" << width << " detail=" << detail;
    EXPECT_NEAR(angle, (kWidthAtZero - width) / 2.0, 1e-6) << "width=" << width;
    // 自洽性：反解出的角再正解回去必须还原原宽度。
    EXPECT_NEAR(commander.jawWidthAtAngle(angle), width, 1e-6) << "width=" << width;
  }
}

TEST_F(GripperCommanderTest, GraspAngleAppliesPreload)
{
  moveit::core::RobotModelPtr model = makeModel();
  ASSERT_NE(model, nullptr);

  GripperConfig cfg = makeConfig();
  cfg.grasp_preload_m = 0.004;

  GripperCommander commander(node_);
  std::string detail;
  ASSERT_EQ(commander.configure(model, cfg, detail), PlanErrorCode::kSuccess) << detail;

  // 预紧的语义是"比物体再多合 preload"：目标张口 = width - preload。
  double angle = 0.0;
  ASSERT_EQ(commander.graspAngleForWidth(0.06, angle, detail), PlanErrorCode::kSuccess) << detail;
  EXPECT_NEAR(commander.jawWidthAtAngle(angle), 0.06 - 0.004, 1e-6);
  // 有预紧时的角度必须比无预紧时更大（合得更紧），而不是更小。
  EXPECT_GT(angle, (kWidthAtZero - 0.06) / 2.0);
}

TEST_F(GripperCommanderTest, RejectsObjectWiderThanMaxOpening)
{
  moveit::core::RobotModelPtr model = makeModel();
  ASSERT_NE(model, nullptr);

  GripperCommander commander(node_);
  std::string detail;
  ASSERT_EQ(commander.configure(model, makeConfig(), detail), PlanErrorCode::kSuccess) << detail;

  double angle = 0.0;
  EXPECT_EQ(
    commander.graspAngleForWidth(kWidthAtZero + 0.01, angle, detail),
    PlanErrorCode::kGraspWidthUnreachable) << detail;
  EXPECT_FALSE(astribot_s1_manipulation::isRetryable(PlanErrorCode::kGraspWidthUnreachable));
}

TEST_F(GripperCommanderTest, RejectsObjectNarrowerThanFullyClosedGap)
{
  moveit::core::RobotModelPtr model = makeModel();
  ASSERT_NE(model, nullptr);

  GripperCommander commander(node_);
  std::string detail;
  ASSERT_EQ(commander.configure(model, makeConfig(), detail), PlanErrorCode::kSuccess) << detail;

  // 完全闭合时张口仍有 0.01 m。比它更窄的物体合到底也碰不到 ——
  // 而这种情况**碰撞检测查不出来**（没接触就没碰撞），只能在这里拦。
  double angle = 0.0;
  EXPECT_EQ(
    commander.graspAngleForWidth(0.005, angle, detail),
    PlanErrorCode::kGraspWidthUnreachable) << detail;
}

TEST_F(GripperCommanderTest, RejectsNonPositiveWidth)
{
  moveit::core::RobotModelPtr model = makeModel();
  ASSERT_NE(model, nullptr);

  GripperCommander commander(node_);
  std::string detail;
  ASSERT_EQ(commander.configure(model, makeConfig(), detail), PlanErrorCode::kSuccess) << detail;

  double angle = 0.0;
  EXPECT_EQ(commander.graspAngleForWidth(0.0, angle, detail), PlanErrorCode::kInvalidInput);
  EXPECT_EQ(commander.graspAngleForWidth(-0.01, angle, detail), PlanErrorCode::kInvalidInput);
}

// ---------------------------------------------------------------------------
// 四、故障注入：闭合位越过两指对穿点
//
// q=0.05 时两个指垫中心重合，再往下合就互相穿过去，张口先减到负再增回来。
// 这时线性插值反解出的角度是错的，而且错得很安静 —— 必须在 configure 阶段就拦住。
// ---------------------------------------------------------------------------
TEST_F(GripperCommanderTest, RejectsNonMonotonicJawTable)
{
  moveit::core::RobotModelPtr model = makeModel(1.0, kShutCrossover);
  ASSERT_NE(model, nullptr);

  GripperCommander commander(node_);
  std::string detail;
  EXPECT_EQ(commander.configure(model, makeConfig(), detail), PlanErrorCode::kInvalidInput);
  EXPECT_NE(detail.find("不单调"), std::string::npos) << detail;
  EXPECT_FALSE(commander.isConfigured());
}

// ---------------------------------------------------------------------------
// 五、故障注入：张口方向选错
//
// 张口沿 tcp 的 x 轴。把 jaw_axis_in_tcp 改成 z 之后，两个指垫在该方向上
// 的投影几乎不随角度变化 —— 这能通过单调性检查（处处 delta≈0 也算单调），
// 所以必须由行程检查来拦。否则错误会推迟到反解时才暴露，
// 报的却是"物体比闭合间隙还窄"，指向完全错的方向。
// ---------------------------------------------------------------------------
TEST_F(GripperCommanderTest, RejectsWrongJawAxis)
{
  moveit::core::RobotModelPtr model = makeModel();
  ASSERT_NE(model, nullptr);

  GripperConfig cfg = makeConfig();
  cfg.jaw_axis_in_tcp = {0.0, 0.0, 1.0};

  GripperCommander commander(node_);
  std::string detail;
  EXPECT_EQ(commander.configure(model, cfg, detail), PlanErrorCode::kInvalidInput);
  EXPECT_NE(detail.find("几乎没动"), std::string::npos) << detail;
}

TEST_F(GripperCommanderTest, RejectsZeroJawAxis)
{
  moveit::core::RobotModelPtr model = makeModel();
  ASSERT_NE(model, nullptr);

  GripperConfig cfg = makeConfig();
  cfg.jaw_axis_in_tcp = {0.0, 0.0, 0.0};

  GripperCommander commander(node_);
  std::string detail;
  EXPECT_EQ(commander.configure(model, cfg, detail), PlanErrorCode::kInvalidInput);
}

// ---------------------------------------------------------------------------
// 六、mimic 从动关节确实参与了张口计算
//
// 这条曾经真的挂过，抓到的是实现里的一个静默 bug：用
// setJointGroupPositions() 设角度时，MoveIt 只更新**组内**的 mimic 关节，
// 而 SRDF 里夹爪组刻意只含主动关节 —— 从动关节一个都没动，
// 张口变化率只有真实值的一半，且全程零报错。
// 改用 setJointPositions(master, ...) 才会遍历 master->getMimicRequests()。
//
// 这个测试把 multiplier 改成 0（从动指垫完全不动），断言变化率随之减半 ——
// 从而证明正解真的把 mimic 算进去了，而不是碰巧数值对得上。
// ---------------------------------------------------------------------------
TEST_F(GripperCommanderTest, MimicJointParticipatesInJawWidth)
{
  moveit::core::RobotModelPtr moving = makeModel(1.0);
  moveit::core::RobotModelPtr frozen = makeModel(0.0);
  ASSERT_NE(moving, nullptr);
  ASSERT_NE(frozen, nullptr);

  GripperCommander with_mimic(node_);
  GripperCommander without_mimic(node_);
  std::string detail;
  ASSERT_EQ(with_mimic.configure(moving, makeConfig(), detail), PlanErrorCode::kSuccess) << detail;
  ASSERT_EQ(
    without_mimic.configure(frozen, makeConfig(), detail), PlanErrorCode::kSuccess) << detail;

  const double q = kShutNormal;
  const double closing_with = kWidthAtZero - with_mimic.jawWidthAtAngle(q);
  const double closing_without = kWidthAtZero - without_mimic.jawWidthAtAngle(q);
  EXPECT_NEAR(closing_with, 2.0 * q, 1e-9);
  EXPECT_NEAR(closing_without, 1.0 * q, 1e-9);
}
