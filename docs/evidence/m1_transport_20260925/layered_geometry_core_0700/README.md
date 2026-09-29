# 分层包络核心：替换旧切片算法

按本任务用户最新明确要求“不要并存，旧的删掉”，删除 RobotModel 内固定 0.25 m 切片循环，只有一个显式高度配置驱动的 `slices` 输出。未保留 `layered_slices` 双输出或旧算法回退。目录名仅为证据标识，实际开始/完成时间见 manifest.json。

本次独占修改 robot_model.hpp、robot_model_probe.cpp、layered_robot_model_test.cpp、test_layered_height_profile.py；不改 geometry_state_node、绑定、CMake、ROS 安装或共享 Git/index。这些调用方由主线负责人接入，未完成接入前不代表整包可运行。

## 实现契约

- `geometry(q, errors, attachments, padding, floor, layer_edges)` 的末尾参数为 base 坐标高度边界。省略或空列表明确抛错；至少两个有限且严格递增的边界，不默默恢复旧算法。
- 复用同一次 FK、形体支持高度、padding/关节误差膨胀和实际 attachments，不从旧粗切片重采样。
- 每个闭区间包含与其接触/相交形体的完整保守二维投影；边界形体同时属于两侧层。空层保留有高度上下界的空多边形，输出数量始终等于边界数量减一。
- 总 physical/reserved、真实 height/z_min 的计算保留。超出配置顶端/底端仍返回真实高度，供调用方判定观测覆盖；本核心不会自行把缺失覆盖标为完整。
- probe 必须输入 `layer_edges`，只输出 `slices`。新独立结果字段已删除。
- 共享 profile 来源：`ws_robot/src/astribot_s1_mapping/config/height_slices.yaml`，地面边界 `[0.05,0.25,0.68,1.18,1.63,2.30]`。独立 profile 测试用当前平地仿真离线 `ground_in_base=-0.095`，对应 `[-0.045,0.155,0.585,1.085,1.535,2.205]`；不把此偏移视为坡地或真机标定。

## 验证与证据范围

替换版本独立编译及 11 个 C++ GTest 全通过，覆盖：改变分层时整体几何保持一致、箱体触界、边界 ±2^-40 m、球解析圆、倾斜圆柱解析轮廓、关节旋转、偏置载荷、padding/保持误差、覆盖不足仍暴露真实高度、窄层不沿用粗切片过覆盖、非法/缺省配置及恢复。

独立 profile pytest 在 O2/O3 各通过一次（2 passed），检查真实 YAML、地面到 base 换算、空层与单输出形态。历史 Python 差分测试按主线要求暂停，未把此前并存候选的通过结果计入当前替换版本。此前并存候选仅在 `superseded_parallel_candidate/` 留存证据，标记 NOT_ADOPTED。

没有运行 ROS、共享 colcon、整栈导航/搬运或真机，也没有性能基准；测试执行总耗时不是性能结论。调用方接入、原有高度覆盖/时效/版本门槛、ROS 协议与规划避障耦合由主线继续验收。空层必须继续被消费者正确识别，不能直接套用非空多边形约束。

## 复现

在仓库根目录执行：

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Werror -I/usr/include/eigen3 \
  -Iws_robot/src/astribot_s1_robot_geometry/include \
  ws_robot/src/astribot_s1_robot_geometry/test/layered_robot_model_test.cpp \
  -ltinyxml2 -lcrypto -lgtest_main -lgtest -pthread -o /tmp/layered_robot_model_test
/tmp/layered_robot_model_test
python3 -m pytest -q ws_robot/src/astribot_s1_robot_geometry/test/test_layered_height_profile.py
```

可复用 probe：`/home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/m1_transport_20260925/layered_geometry_core_0700/bin/robot_model_probe`。

before/after、changes.patch、gtest.xml、pytest.xml 与源码/可执行文件哈希见本目录及 manifest.json。Git 提交由主线负责人统一管理，本子任务未自行提交或改共享 index。
