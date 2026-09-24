# M3 文件归属与 baseline 边界

整个 manipulation_perception 包本轮开始前已存在但未被Git跟踪，不能把整个包当成本轮新实现。当前33个源/配置文件分为19个新文件、4个修改的既有文件、10个原样保留的既有文件；生成的__pycache__不计。hash见delivery_manifest.json；本任务未整包add或commit。

## 新增客户端与几何映射（11）

- `PICK_PLANNING.md`
- `include/astribot_s1_manipulation_perception/pick_planning_client.hpp`
- `include/astribot_s1_manipulation_perception/known_box_grasp_mapping.hpp`
- `src/pick_planning_contract.cpp`
- `src/pick_planning_client.cpp`
- `src/pending_action.hpp`
- `src/known_box_grasp_mapping.cpp`
- `test/pick_planning_test.cpp`
- `test/pick_planning_protocol_test.cpp`
- `test/known_box_grasp_mapping_test.cpp`
- `launch/paired_inference.launch.py`

## 新增RGB-D源（8）

- `include/astribot_s1_manipulation_perception/single_box_request_source.hpp`
- `src/single_box_request.cpp`
- `src/single_box_request_source.cpp`
- `src/single_box_source_detail.hpp`
- `test/single_box_fixture.hpp`
- `test/single_box_request_test.cpp`
- `test/single_box_source_protocol_test.cpp`
- `config/single_box_projection.yaml`

## 修改既有文件（4）

- `CMakeLists.txt`：client/mapping/source依赖、导出与测试
- `package.xml`：新增库所需依赖
- `README.md`：客户端入口说明
- `src/manipulation_perception_server.cpp`：仅接受square_prism_z注册声明

## 保留既有文件（10）

- `config/inference.yaml`
- `include/astribot_s1_manipulation_perception/inference_contract.hpp`
- `include/astribot_s1_manipulation_perception/worker_process.hpp`
- `launch/inference.launch.py`
- `src/inference_contract.cpp`
- `src/worker_process.cpp`
- `test/action_contract_test.cpp`
- `test/fixture_worker.cpp`
- `test/inference_contract_test.cpp`
- `test/worker_process_test.cpp`

## 授权包外改动

object_pose_core 的CMakeLists.txt、README.md、src/register_cli.cpp是既有文件局部修改，tests/square_prism_cli_test.cpp为新增。范围仅square_prism_z声明与验证，normal_box目录记录模型资产及4+4语义/CLI边界证据。

## 原始副本与证据

本任务没有保存开始前完整源码副本，任务目录也未找到对应时点副本，不能提供由完整原文件支持的旧包补丁。不可把hash或推测重建当作原副本。上级source_manifest.json是10:25阶段14文件hash，已包含早期改动，不是修改前baseline。

最近找到的历史服务离线验证为docs/evidence/task_chain_20260922_gpu_recovery/test_results.json的inference分组：Action契约24、纯契约9、worker4，共37例，原XML时间为2026-09-22 23:14:53/54；比09-21的28例更新。历史结果不自动证明当前源码。本轮原有三套测试已使用独立夹具重跑，结果单列rgbd_source/status.json，未增加新矩阵或运行真实模型。

本轮normal_box资产与rgbd_source证据是交付材料；server/contract/worker仍是既有实现。总调度若保存完整可重建包快照，应保留上述归属区别。
