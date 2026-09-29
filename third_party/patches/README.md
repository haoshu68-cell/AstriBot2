# 本地子模块修改快照

2026-09-29 推送项目时，以下子模块有尚未提交到第三方上游的本地差异。主仓库的 gitlink 仍固定原提交；补丁保存这些差异，没有向第三方仓库推送，也没有改变现有工作区的子模块内容。

| 子模块 | 基准提交 | 补丁内容 |
|---|---|---|
| `ws_robot/src/livox_ros_driver2` | `4a1def929e5b59c7a8122d19fce6efba581ce9f7` | ROS2 构建接入项目 Eigen vendor，按可用命令选择 typesupport 逻辑 |
| `ws_robot/src/aws-robomaker-small-warehouse-world` | `ee0af733315e78432408c3cd98d378ecee5f767c` | 保留 world 文件的一行空白差异，无功能变更 |

在**新检出的仓库**根目录恢复本次快照：

```bash
git submodule update --init --recursive
git -C ws_robot/src/livox_ros_driver2 apply --check ../../../third_party/patches/livox_ros_driver2.patch
git -C ws_robot/src/livox_ros_driver2 apply ../../../third_party/patches/livox_ros_driver2.patch
git -C ws_robot/src/aws-robomaker-small-warehouse-world apply --check ../../../third_party/patches/aws-robomaker-small-warehouse-world.patch
git -C ws_robot/src/aws-robomaker-small-warehouse-world apply ../../../third_party/patches/aws-robomaker-small-warehouse-world.patch
```

当前工作区已经包含这些修改，不重复应用。Livox 的未跟踪 `launch/` 是 `build.sh` 从已跟踪的 `launch_ROS2/` 复制出的重复启动文件，不纳入补丁。应用检查及内容比对只证明补丁可恢复源文件，不代表驱动构建或机器人运行验收。
