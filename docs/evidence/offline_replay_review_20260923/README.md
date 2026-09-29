# 离线复盘评审证据与复现入口

日期：2026-09-23。生产实现只读，验证代码在 `probes/`。报告见 [评审](../../OFFLINE_REPLAY_UI_REVIEW_20260923.md)，操作见 [手册](../../manuals/OFFLINE_REPLAY_OPERATION_VALIDATION_20260923.md)。

## 原始结果

- `baseline.json` / `source_unchanged.json`：关键文件哈希和前后复核。
- `build.log` / `rebuild.log`：隔离构建；没有覆盖生产安装目录。
- `test_evidence.xml`：5 项参数证据测试通过。
- `test_replay_stream.xml`：1 项显示白名单与速度发布隔离测试通过。
- `probe_results.json`：实际播放器计时观测和 Reader 排序实验。该探针成功退出表示完成采样，**不表示产品满足时序要求**。
- `normal_player.log` / `seek_player.log`：对应实际播放器 stdout/stderr。
- `rollback_metadata.yaml`：时钟回退合成包的元数据。Reader 的时间排序不保留时钟段身份。
- `ui_probe_results.json` / `ui_event_selection.png` / `ui_probe.log`：真实 Qt 窗口源码，替换子播放器和 RViz 后的 UI 逻辑观测；不是实际 RViz/OGRE 渲染证据。

Qt 替身发送 start=100 s、duration=10 s 的合成协议，第一次 POSITION=106 s、第二次 POSITION=100 s。事件包含 101 s 参数 0.3 和 105 s 参数 0.1。协议刻意独立于地图 bag 时长，仅检查窗口收到进度后的事件/详情处理。`viewer_probe.cpp` 包含生产 `replay_viewer.cpp`，未复制或修改被测逻辑。

实际计时探针用 1200 条间隔 2 ms 的历史地图消息，跳到 2.4 s，观察之后相隔 200 ms 的两条消息；地图值 120/121 仅作识别标记，不用于真实地图展示。普通播放 199.138 ms，跳转后 1.132 ms，是一对单次观测。初步 1 ms 密集样本使普通播放也追赶，未用于最终对照。

## 重新运行（仅开发验证）

依赖当前 ROS2 Humble、Qt5 Widgets、nlohmann_json、ament gtest。先核对选定 ROS domain 无本机相关会话，确认不与共享仿真/真机混用；不要结束他人的进程。本次使用 93，复测可换成已经核对的空闲域。以下命令不启动 Gazebo 或真实 RViz。

```bash
cd /home/yjh/WorkSpace/astribot_sdk_ros2
source /opt/ros/humble/setup.bash
review_tmp=$(mktemp -d /tmp/astribot-replay-review.XXXXXX)
cmake -S docs/evidence/offline_replay_review_20260923/probes -B "$review_tmp/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$review_tmp/build" -j2
mkdir -p "$review_tmp/real_prefix/share/ament_index/resource_index/packages" "$review_tmp/real_prefix/lib/astribot_operator_station"
touch "$review_tmp/real_prefix/share/ament_index/resource_index/packages/astribot_operator_station"
ln -s "$review_tmp/build/replay_stream" "$review_tmp/real_prefix/lib/astribot_operator_station/replay_stream"
export ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1
export AMENT_PREFIX_PATH="$review_tmp/real_prefix:$AMENT_PREFIX_PATH"
"$review_tmp/build/test_evidence" --gtest_output="xml:$review_tmp/test_evidence.xml"
"$review_tmp/build/test_replay_stream" --gtest_output="xml:$review_tmp/test_replay_stream.xml"
"$review_tmp/build/review_probe" "$review_tmp/build/replay_stream" "$review_tmp/runtime"
```

检查 `runtime/probe_results.json`：两组都应收到目标和下一样本。若都出现大幅偏差，先查测试负载/订阅条件，不把受影响的普通播放当成有效控制组。合格产品应在跳转后仍保持约 200 ms；现有版本复现的是偏差。

UI 探针需要前一步生成的 incident 夹具：

```bash
python3 docs/evidence/offline_replay_review_20260923/probes/prepare_ui_fixture.py "$review_tmp"
export AMENT_PREFIX_PATH="$review_tmp/ui_prefix:$AMENT_PREFIX_PATH"
export PATH="$review_tmp/ui_prefix/bin:$PATH"
export QT_QPA_PLATFORM=offscreen
export REVIEW_UI_COUNTER="$review_tmp/ui_count"
export REVIEW_UI_REPORT="$review_tmp/ui_probe_results.json"
export REVIEW_UI_IMAGE="$review_tmp/ui_event_selection.png"
"$review_tmp/build/viewer_probe" "$review_tmp/runtime/seek"
```

每次复测使用新临时目录/新 counter，避免旧调用次数影响第一帧。UI 替身不加入 ROS，也不显示实际 3D。探针正常退出时由实际窗口清理其创建的子进程。

预期暴露现有问题：`label_after_event_click` 为 6 s，选中详情为 101 s；`label_after_seek` 为 0 s，详情仍是 101 s 参数。修复后应更新断言，不能继续把“复现缺陷”的退出码当作产品通过。

本次在 `/tmp/astribot_replay_review_20260923` 执行。该路径会被系统清理，持久证据在本目录；合成 bag 可由探针重建。
