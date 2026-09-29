# 相机识别链路增量实施记录

日期：2026-09-21。用户已授权继续上一阶段相机、识别和位姿链路，运行时优先 C++。

本次范围是既有链路的同步/输入校验修复及 YOLO 适配，不接管任务执行、不做 VLA 验证。
仍使用仓库导航环境；构建和 ROS 消息验证放在本轮专用目录与域，不启动或清理其他会话。

## 步骤和放行标准

1. 用真实 C++ 节点构造回归，复现旧帧配对、过期健康、frame/标定不一致、坏掩码及虚假姿态语义。
2. 实现采集时间配对、有界缓存、输入布局校验、明确三维表面中心/朝向未知，修复候选闸门默认跳过版本校验问题。
3. 基于已有 OpenCV DNN 实现 C++ YOLO ONNX 适配；模型输出格式显式指定；不兼容模型、推理超时、过期输入拒收。
4. 用固定输入测试预处理、框还原和类别内 NMS；有可用权重时执行真实离线推理。合成张量不作为模型精度证据。
5. 检查独立/仓库 launch，并记录代码、测试和待验收边界。输入/检测质量不等于机械臂可执行许可。

## 决策记录

- 保持当前工作分支中的已有未提交实现，避免新 worktree 丢失上一阶段接口和其他会话工作；专用 build/install 不覆盖共享安装。
- 三维中心观测只描述可见表面的局部位置，不提供物体朝向。GraspNet 给出夹爪抓取 6D 候选；物体 6D 姿态需独立算法，不能由 GraspNet 等同替代。
- 先采用本机已有 OpenCV DNN C++ 后端验证 ONNX 合同，保留后续 TensorRT/ONNX Runtime 替换边界；不为此安装全局 Python/PyTorch 环境。
- 安全检查保留；不能通过放宽 freshness、碰撞或几何门槛提高成功率。

## 证据

执行日志及测试结果保存在 `docs/evidence/vision_cpp_20260921/`。完整结果在执行完毕后填写。

## 本轮结果（2026-09-21）

代码和隔离构建已完成。未启动、停止或接管共享 Gazebo；未连接真机；未做 VLA 验证。
根目录 `install/` 未覆盖，候选消息和节点安装在 `/tmp/vision_cpp_build/install/`。

| 链路 | 本轮落地 | 已验证范围 |
|---|---|---|
| 相机健康 | C++ 检查图像布局、内参、三路时效/频率、标定版本；启动生成新 epoch；时钟暂停也按接收时效失效 | 6 个 C++ 用例，包含停帧、坏深度、无标定、内参改变 |
| YOLO | C++ OpenCV DNN；保持比例缩放补边、框还原、类别内 NMS；最新一帧队列、推理前后检查上下文 | 6 个张量/解码用例；真实 YOLOv5n v6.0 ONNX 推理 |
| 检测门控 | 来源、epoch、标定、帧、置信度、时间与掩码布局检查；接收时效独立检查 | 1 个门控暂停时钟用例及下游故障矩阵 |
| RGB-D 观测 | 采集时间配对、有界深度/内参历史、16UC1/32FC1、坏输入拒绝、发布前再验时效 | 23 个 C++ 用例；明确 position 有效 / orientation 未知 |
| 完整消息链 | YOLO → 检测门控 → RGB-D 位置观测 | 4 个真实模型 ROS 用例，深度和相机健康由测试合成；不是 Gazebo/真机验收 |
| 抓取候选 | 三版本上下文必填且只读、单位四元数、碰撞声明、时效、数组 frame 检查 | 10 个 C++ 用例；筛选结果仍要求 MTC 重新验证 |
| 启动集成 | 头部/腹部话题分离、原始/后处理模式一致、按相机选择 YOLO，默认关闭推理 | 5 组离线 launch 配置验证 |

最终 **50 个 GTest 用例全部通过，无跳过**，另外 2 个既有 CTest 校验通过，5 组启动配置通过。
测试入口和日志：

- `tools/vision/verify_cpp_vision.sh`：依次构建消息、感知、操作包，在隔离域执行测试，不启动机器人栈。
- `tools/vision/verify_vision_launch.py`：仅求值启动参数和条件，不执行节点。
- `docs/evidence/vision_cpp_20260921/verification_final.log`：完整最终构建与测试输出。
- `docs/evidence/vision_cpp_20260921/summary.json` 及 `*.gtest.xml`：准确用例数和结果。
- `*_red.log`：保留修复前可复现的失败；这些失败日志不是当前验收状态。

审查另发现并修复：退化内参引起 NaN、协方差溢出、时钟冻结时旧缓存复用、同版本内参漂移、
参数显示新版本但仍使用旧版本、无效首帧污染标定基线、计算结束后的时效复检缺失。
独立代码复查确认这些修复；源码复查与运行测试分别记录，不等同真机放行。

## 模型兼容性与复现

本机 OpenCV 4.5.4 无法加载原始 YOLOv5n v6.0 动态图的 `Range` 算子。
离线脚本 `tools/vision/prepare_yolov5_onnx.py` 固定 `[1,3,640,640]` 输入，提取单一已解码输出
`[1,25200,85]`。原始模型和转换模型在两个固定随机输入上的 ONNX Runtime 输出最大绝对差
均为 **0.0**；转换只用于兼容图结构，未修改检测阈值。ONNX Runtime/onnxsim 仅安装在
`/tmp/astribot_onnx_tools/packages` 供转换及验证，运行节点依赖 C++ OpenCV DNN。

- 原始权重：[YOLOv5 v6.0 官方发布](https://github.com/ultralytics/yolov5/releases/tag/v6.0)，文件 `yolov5n.onnx`。
- 官方样例：[bus.jpg](https://raw.githubusercontent.com/ultralytics/yolov5/v6.0/data/images/bus.jpg)。
- 标签来源：[coco128.yaml](https://raw.githubusercontent.com/ultralytics/yolov5/v6.0/data/coco128.yaml) 的 `names`，按顺序转换为每行一个标签。
- 模型输入/输出哈希、工具版本和数值校验：`docs/evidence/vision_cpp_20260921/model_conversion.json`。
- 当前转换模型 SHA256：`83008218c4d28c0c43834f81d6aef96d061d26adbcb44d8b5b280cdd1173f951`。

真实样例输出 5 个框（4 人、1 公交车），末次单张推理 **107.575 ms**（此前一次 88.1334 ms）。
这两次数字包含预/后处理，是本机样例记录，不是持续吞吐、最坏延迟或机器人检测精度评估。
检测 CSV 和耗时保存在 `yolo_real_inference_final.*`。
YOLOv8 仅验证了张量解析，未验证真实 v8 ONNX 图兼容性；分割、端到端 NMS 模型尚不支持。
参考 [OpenCV YOLO DNN 说明](https://docs.opencv.org/4.x/da/d9d/tutorial_dnn_yolo.html)，不同导出图和预处理须分别验证。

本机已备好的验证资产在 `/tmp/astribot_vision_models/`，不会提交大模型权重；临时目录清理后需重新准备。
从仓库根目录复跑：

```bash
VISION_TEST_MODELS=/tmp/astribot_vision_models bash tools/vision/verify_cpp_vision.sh
```

上述脚本默认测试域 181，可用 `VISION_TEST_DOMAIN` 和 `VISION_BUILD_ROOT` 指定专用域/目录。
需要已有仓库基础依赖安装；不是空白机器的完整部署脚本。
**接口 0.2.0 不与旧 Detection2D/ObjectPoseObservation 二进制兼容**：消息与消费者必须一起重建，
禁止新消息搭配根目录旧视觉节点运行。所有本轮测试均使用候选 overlay。

## 使用入口与尚未验收部分

导航仓库源码入口保留原环境，只新增可选配置：
`enable_yolo_detector`、`yolo_camera_id`、`yolo_model_path`、`yolo_labels_path`、
`yolo_model_revision`、`yolo_model_layout`，配合已有 `enable_rgbd_pose_estimator`。
默认 `enable_yolo_detector=false`，不自动下载或启动模型；默认为头部一路，显式选择 `torso_rgbd` 可切腹部。
原始模式下健康、检测、深度投影使用同一组 raw 话题和 Gazebo 原生 frame。
本轮验证了源码 launch 的求值与参数，不宣称现有运行会话已经加载新 launch。

独立入口 `rgbd_vision_pipeline.launch.py` 默认处理头部，也可选择腹部；
`enable_health=false` 用于复用导航环境已启动的健康源，避免重复发布。
启动真实推理需提供本地模型、标签及明确模型版本；缺失或不兼容模型直接启动失败，不生成假检测。

仍待后续实施或验收：

1. 真机开启后的驱动、外参/内参版本、对齐深度、曝光/运动与遮挡质量验收。
2. Gazebo 多相机持续负载、导航并发下时效/丢帧/算力预算回归。本轮不把合成深度链路视为此项通过。
3. 实例分割、物体身份跟踪/世界对象账本；当前 `object_id` 是每帧假设 ID，不是持久物体 ID。
4. 独立物体 6D 姿态算法；当前只有可见表面三维位置。框内背景和遮挡会偏置位置，未知朝向不可执行。
5. GraspNet 模型服务、独立 MoveIt 碰撞/IK/可达性检查、动态场景版本接入 MTC。`ComputeGrasps` 只有契约，
   `grasp_candidate_gate_node` 只筛选声明，不计算碰撞，也不授予运动权限。

VLA 与完整动态抓取、搬运、放置验证继续按用户要求暂缓。
