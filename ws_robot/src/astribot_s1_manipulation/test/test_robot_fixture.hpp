// Copyright 2026 Astribot
//
// 单测用的合成机器人模型。
//
// 为什么不加载真实的 astribot_s1 URDF：
//   1. 真实 URDF 要跑 xacro 展开、依赖 mesh 文件路径，单测就不再是"纯算法测试"，
//      在 CI 里很容易因为找不到文件而失败。
//   2. 奇异点/闭链残差这两件事的**数学正确性**与具体机型无关。用一个几何关系
//      一眼能算清的合成模型，断言才写得死（能预判确切数值），
//      而不是"跑出来是多少就断言多少"那种自我循环的测试。
//
// 合成模型结构（两条平面 3R 臂，正好能构造出确定的奇异构型）：
//
//     world
//       |-- fixed --> left_base  --j1(z)--> l1 --j2(z)--> l2 --j3(z)--> l3 --fixed--> left_tcp
//       `-- fixed --> right_base --j1(z)--> r1 --j2(z)--> r2 --j3(z)--> r3 --fixed--> right_tcp
//
// 每段连杆沿 x 轴长 0.3m，所有关节绕 z 转 —— 这是标准平面 3R：
// 全部关节为 0 时手臂完全伸直，是教科书上的奇异构型（雅可比丢秩）。

#ifndef TEST_ROBOT_FIXTURE_HPP_
#define TEST_ROBOT_FIXTURE_HPP_

#include <memory>
#include <string>

#include <moveit/robot_model/robot_model.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace astribot_test
{

/// 单段连杆的 xml。带 collision 几何，这样碰撞检测也能用这个模型。
inline std::string makeLinkXml(const std::string & name)
{
  return
    "  <link name=\"" + name + "\">\n"
    "    <inertial>\n"
    "      <mass value=\"1.0\"/>\n"
    "      <origin xyz=\"0.15 0 0\"/>\n"
    "      <inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\"/>\n"
    "    </inertial>\n"
    "    <collision>\n"
    "      <origin xyz=\"0.15 0 0\"/>\n"
    "      <geometry><box size=\"0.3 0.05 0.05\"/></geometry>\n"
    "    </collision>\n"
    "  </link>\n";
}

/// 一条 3R 臂的 xml。prefix 用来区分左右。
inline std::string makeArmXml(const std::string & prefix, double base_y)
{
  std::string xml;
  // 臂基座：从 world 固连过来，左右在 y 方向分开，保证两臂不会天生重叠。
  xml +=
    "  <link name=\"" + prefix + "_base\">\n"
    "    <inertial><mass value=\"1.0\"/>\n"
    "      <inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\"/>\n"
    "    </inertial>\n"
    "  </link>\n"
    "  <joint name=\"" + prefix + "_mount\" type=\"fixed\">\n"
    "    <parent link=\"world\"/>\n"
    "    <child link=\"" + prefix + "_base\"/>\n"
    "    <origin xyz=\"0 " + std::to_string(base_y) + " 0\"/>\n"
    "  </joint>\n";

  for (int i = 1; i <= 3; ++i) {
    const std::string link_name = prefix + "_l" + std::to_string(i);
    const std::string joint_name = prefix + "_j" + std::to_string(i);
    const std::string parent = (i == 1) ? (prefix + "_base") :
      (prefix + "_l" + std::to_string(i - 1));
    // 第一个关节在基座原点，后续关节在上一段连杆末端（x 方向 0.3m）。
    const std::string origin = (i == 1) ? "0 0 0" : "0.3 0 0";
    xml += makeLinkXml(link_name);
    xml +=
      "  <joint name=\"" + joint_name + "\" type=\"revolute\">\n"
      "    <parent link=\"" + parent + "\"/>\n"
      "    <child link=\"" + link_name + "\"/>\n"
      "    <origin xyz=\"" + origin + "\"/>\n"
      "    <axis xyz=\"0 0 1\"/>\n"
      "    <limit lower=\"-3.14\" upper=\"3.14\" velocity=\"2.0\" effort=\"100\"/>\n"
      "  </joint>\n";
  }

  // TCP：纯坐标系，无碰撞几何（与真实机器人的 tool_link 一致）。
  xml +=
    "  <link name=\"" + prefix + "_tcp\"/>\n"
    "  <joint name=\"" + prefix + "_tcp_fixed\" type=\"fixed\">\n"
    "    <parent link=\"" + prefix + "_l3\"/>\n"
    "    <child link=\"" + prefix + "_tcp\"/>\n"
    "    <origin xyz=\"0.3 0 0\"/>\n"
    "  </joint>\n";
  return xml;
}

inline std::string makeUrdf()
{
  std::string xml = "<?xml version=\"1.0\"?>\n<robot name=\"test_dual_arm\">\n";
  xml +=
    "  <link name=\"world\">\n"
    "    <inertial><mass value=\"1.0\"/>\n"
    "      <inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\"/>\n"
    "    </inertial>\n"
    "  </link>\n";
  xml += makeArmXml("left", 0.5);
  xml += makeArmXml("right", -0.5);
  xml += "</robot>\n";
  return xml;
}

inline std::string makeSrdf()
{
  std::string xml = "<?xml version=\"1.0\"?>\n<robot name=\"test_dual_arm\">\n";
  xml +=
    "  <group name=\"arm_left\">\n"
    "    <chain base_link=\"left_base\" tip_link=\"left_tcp\"/>\n"
    "  </group>\n"
    "  <group name=\"arm_right\">\n"
    "    <chain base_link=\"right_base\" tip_link=\"right_tcp\"/>\n"
    "  </group>\n"
    // 双臂组：故意做成非链组，用来验证代码正确拦住"对非链组求雅可比"。
    "  <group name=\"dual_arm\">\n"
    "    <group name=\"arm_left\"/>\n"
    "    <group name=\"arm_right\"/>\n"
    "  </group>\n";
  // 相邻对必须关掉，否则任何构型都判碰撞。
  for (const std::string prefix : {"left", "right"}) {
    xml += "  <disable_collisions link1=\"" + prefix + "_l1\" link2=\"" + prefix +
      "_l2\" reason=\"Adjacent\"/>\n";
    xml += "  <disable_collisions link1=\"" + prefix + "_l2\" link2=\"" + prefix +
      "_l3\" reason=\"Adjacent\"/>\n";
  }
  xml += "</robot>\n";
  return xml;
}

/// 构造可用的 RobotModel。失败返回 nullptr（测试里直接 ASSERT_TRUE 即可）。
inline moveit::core::RobotModelPtr makeRobotModel()
{
  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(makeUrdf());
  if (!urdf_model) {
    return nullptr;
  }
  auto srdf_model = std::make_shared<srdf::Model>();
  if (!srdf_model->initString(*urdf_model, makeSrdf())) {
    return nullptr;
  }
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

}  // namespace astribot_test

#endif  // TEST_ROBOT_FIXTURE_HPP_
