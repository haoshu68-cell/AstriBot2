# 操作感知 C++ Action 服务

M3 新增的 C++ 感知到 MTC 提案客户端、双服务实例部署和受限箱体注册适配见 [PICK_PLANNING.md](PICK_PLANNING.md)。它返回规划提案，不执行机器人；当前离线/隔离协议证据不等于模型驱动抓放闭环通过。

这两个接口只返回观测与建议，不控制机械臂、夹爪或底盘：

- `/perception/compute_grasps`：真实 GraspNet TorchScript，经独立 LibTorch C++ worker 产生 6D **抓取候选**。
- `/perception/estimate_object_pose`：注册 CAD 的 C++ 全局配准，返回相机光学坐标中的 **物体坐标系姿态**。它与抓取姿态不是同一概念。

ROS、输入校验、生命周期、坐标转换、模型输出检查均为 C++。启动、权重准备/导出和证据采集使用 Python/shell。LibTorch 在独立进程中，避免其 C++ ABI 与 ROS/OpenCV 相互影响。每次请求启动一个 worker；取消和超时只终止本服务创建的进程组，并回收子进程。

## 数据契约

调用方显式提供已分割目标的 `PointCloud2`、采集 Header、相机 ID、源 epoch、标定/场景/包络版本、有效截止时间和单调时钟推理超时。

- XYZ 为米，处于指定相机光学坐标系；仅接受有限 float32、合法步长和布局、正深度。上限 200000 点。GraspNet 至少 2048 点，CAD 配准至少 64 点。
- 相机健康必须来自配置的独立健康话题，且持续有效。输入最多 0.5 秒；健康 ROS 时间与接收时钟均最多 0.5 秒。源 epoch、标定、frame 变化或相机掉线使在途结果无效。
- 结果保留原始采集时间，默认最多保留 5 秒的**感知推理快照**；这不是执行许可，不修改下游执行新鲜度门槛。超时、取消、模型文件替换、过期结果不返回可用候选。
- 单节点只容许一项推理同时执行；忙时拒绝新目标。部署上下文参数只读，默认版本为 0 时拒绝请求；切换世界上下文应重建服务，不以伪造固定版本替代真实任务上下文。
- GraspNet 官方原始分数可大于 1。`raw_model_score` 原样保留；`score=s/(1+s)` 仅为单调、有界排序值，`score_semantics` 明确标注它不是概率。不得当作经过标定的成功率。
- 上游回归头也可能产生非正分或截断为零的夹爪宽度；逐个剔除这些不可用 proposal，并报告原始/筛除数，不能让一个低分 proposal 拖垮整批。非有限值、非法旋转和协议错误仍导致失败。
- 抓取 `collision_checked=false`、`collision_free=false`；返回的建议仍须通过当前 MTC/MoveIt 全机器人碰撞、IK、可达性与任务准入。当前服务不做场景点云碰撞检查，也不发布到执行话题。
- `grasp_pose` 是 GraspNet 原生夹爪坐标系在相机光学坐标中的变换，不是 Astribot 末端 TCP 指令。接入 MTC 时必须使用注册的模型夹爪到实际末端变换，并检查实际夹爪尺寸。
- 6D 只支持注册的已知模型；对称性由注册表声明，歧义必须拒绝。配准残差/覆盖率不是位姿协方差，返回 `1e6` 对角线表示未标定的不确定性，不可据此融合为高精度定位。

## 构建与配置

仓库已有 ROS2 Humble/MoveIt/OpenCV 的 underlay 后执行：

```bash
bash tools/vision/build_manipulation_perception.sh
source runs/grasp_pose_build/install/local_setup.bash
```

`astribot_perception_msgs` 0.3 增加了字段与 Action，是一次 ABI 变化。必须同时重建 `perception_components`、`manipulation` 和本包；不要混用旧 Python 生成消息或旧抓取候选二进制。脚本已覆盖这些消费者。

先按 `astribot_graspnet_runtime/README.md` 准备模型和 worker。将 `config/inference.yaml` 复制到本次运行目录，填写 worker、模型、相机、健康话题和权威版本。`model_registry` 为 JSON，客户端只能指定其中的 ID：

```json
{
  "asymmetric_union": {
    "path": "/absolute/path/asymmetric_union.xyz",
    "visibility_model": "/absolute/path/asymmetric_union.visibility.json"
  }
}
```

可选 `symmetry: "box"` 只适用于确有该对称性的模型，不能给非对称物体标注来掩盖错误朝向。可见性几何必须与 CAD 一致；缺少该几何时保留全 CAD 覆盖率的保守门槛。

```bash
ros2 launch astribot_s1_manipulation_perception inference.launch.py config:=/absolute/path/inference.yaml
```

模型文件在启动时计算 SHA256，请求前后再核验；注册表/worker 路径只由部署者配置，客户端不可传任意可执行文件或模型路径。

## 验证边界

`test/fixture_worker.cpp` 只用于消息、取消、时效和错误处理测试，不安装、不作为算法实现或精度证据。

`tools/vision/validate_inference_actions.py` 从当前导航仓库的真实 RGB-D 采集新快照，使用明确标注的颜色分割夹具，发起 Action 并保留时间、frame、health、结果与延迟。它不读取仿真真值，也不能证明 YOLO 实例分割已完成。独立仿真评分读取单独的 truth 文件。

这一阶段提供推理服务和明确输入接口；世界物体账本、MTC 候选选择、物理抓放验收与 VLA 实模型验证是另外的边界。不要以服务成功代替这些验收。

## 相机传输

导航仓库在 `localhost_only:=true` 时自动为 `ros_gz_bridge` 使用 `camera_bridge_fastdds.xml`：16 MiB 共享内存、4 MiB 消息上限，UDP 发现仅 loopback。不调整全局内核参数、不清理其他任务共享内存。Humble 中桥接进程的 `ROS_LOCALHOST_ONLY=0` 用于避免额外注入默认 512 KiB SHM，网络隔离由 XML 的 127.0.0.1 白名单保证；其余节点保持原设置。`localhost_only:=false` 时继承调用方网络和 RMW 环境，不强制 loopback；也可用 `camera_bridge_dds_profile` 显式指定 profile，此时网络范围由该文件负责。本次没有跨机吞吐验收。

这是有记录的传输容量修复，不能替代相机完整帧配对、0.25 秒时效和 8 Hz 最低速率检查。头相机传输正常也不代表深度几何有效：当前头部网格自遮挡需要另外解决。
