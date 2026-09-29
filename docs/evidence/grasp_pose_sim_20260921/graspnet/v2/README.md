# GraspNet FPS v2 修复与复验

2026-09-21。保留原v1模型及原证据；本目录对应修正采样并列语义后的v2模型。源checkpoint未改变，严格加载官方baseline固定提交对应的已核验权重，无随机权重、几何候选或truth输入。

## 修复内容

上游固定commit `280c215129f759ed8649cb4e89fc5dfee55f4f80` 的 `pointnet2/_ext_src/src/sampling_gpu.cu` 在线程内保持首个最大距离点；线程间按block_size/2、...、1偏移归约，距离并列时保留左子树。其lane优先级是bit reversal，不是递增lane编号。

最小复现：513点均为(0,0,1)，index1=(1,0,1)、index256=(-1,0,1)，采2点。v1给[0,1]，上游控制流及v2给[0,256]。测试先失败后修复；4至512的所有blocksize均覆盖；另以逐lane、逐归约树的独立参考实现验证13种点数、两个batch、12步采样，包含大量精确距离并列和近原点排除。CPU及CUDA环境各10/10测试通过，日志位于上一级 `fps_tie_regression_*.log`。

v2按bit-reversed lane rank，再按lane内访问顺序排序，保留可脚本化Torch算子；未修改网络层、权重或候选解码。仍未编译/运行原始CUDA extension，因此不宣称与原CUDA extension整体逐位等价。

## 模型与C++一致性

- CUDA：`runs/grasp_pose_sim_20260921/models/graspnet_cuda_v2.pt`，SHA256 `5acc1ff77600a6e1b8a5a814b363956c9e12bf74abf89d85b94ba6f66cfd1b7c`。
- CPU：`runs/grasp_pose_sim_20260921/models/graspnet_cpu_v2.pt`，SHA256 `82a2d7208b38333b10fdb025ed6f4efab0f661b914f32da563c142c5048535c7`。
- checkpoint仍为 `60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868`。
- 导出报告包含源码、checkpoint、算子和模型hash。新的模型文件名与revision避免覆盖活跃v1服务。
- 官方RGB-D demo及将原始输入x平移11mm后的独立输入：CPU/CUDA的现有C++ worker均与对应设备eager输出一致，4次比较最大绝对误差均0，各331候选。详见 `cpp_cpu_equivalence/report.json`、`cpp_cuda_equivalence/report.json`。
- 准备时TorchScript变输入校验的CUDA最大差异为2.68e-7，处于预定rtol2e-5/atol2e-6以内；C++独立进程比较恰为0，两个结论分别记录。

## 真实RGB-D离线回放

| 场景 | 实际点数 | CPU/GPU原始候选数 | GPU完整进程耗时 |
|---|---:|---:|---:|
| clear | 2556 | 915 / 915 | 1.275 s |
| tilted | 2389 | 749 / 749 | 1.236 s |
| yawed | 2186 | 1022 / 1022 | 1.217 s |

两种设备均验证输出有限、旋转矩阵合法；空输入、稀疏输入、非有限输入和负深度分别被拒绝。原始分数未经概率校准，可以有非正值；报告另记录正/非正候选数，交由上层明确筛选。没有把原始候选等同于有效抓取、碰撞通过、IK可达或执行批准。

这里是保存的真实Gazebo RGB-D数据回放，不是新鲜ROS Action验收。现场接口验收由主线程另行记录。未执行机械臂抓取、真机或VLA测试。

## 评测器旧结果隔离

`sim_pose_evaluate.py`调用前删除旧summary及本次目标结果，只允许exit0+success=true、exit2+success=false。其余状态停止评分；不会把旧JSON成功结果计入新运行。`sim_pose_test_evaluate.py`使用独立失败CLI和预存成功JSON，3个故障回归先失败、修复后5/5通过，日志在 `../../simulation/evaluator_regression_*.log`。该修复不修改之前已取得的真实结果。
