# 旧运行版本清理记录

按用户“较早的 fixed_envelope 程序删除，其他只保留最新版本”执行。清理限于本轮导航恢复调用链的7类程序/配置产物，保留源码、当前构建工作区、当前选定安装、原始日志和验收证据；未执行整仓库清空。

- 已先归档并校验，再删除旧 fixed_envelope 二进制及废弃 deployment_candidate/v2 两套运行候选。
- 后续清理69个旧文件/入口：fixed_envelope 16、geometry_state 13、导航helper 19、恢复BT库3、恢复规划/控制库3、constraint5、旧BT配置10。
- 合计删除17个旧 fixed_envelope 文件；当前运行 ae91a376… 及其实际安装入口保留。
- 删除前检查进程可执行文件与共享库映射，无待删文件正在使用；删除后7项选定产物的SHA256全部一致。相同最新版本的构建/安装副本属于当前工作区，不按旧版本误删。
- 仿真未重启；本轮录制已关闭，前次失败的资源释放和停车证据已保存。

归档及删除清单（仓库根目录下）：

- `runs/mainline_20260926/recovery_loop_1852/archive/obsolete_runtime_20260926.tar.gz`
- `runs/mainline_20260926/recovery_loop_1852/archive/superseded_artifacts_20260926.tar.gz`
- `runs/mainline_20260926/recovery_loop_1852/cleanup_removed.json`
- `runs/mainline_20260926/recovery_loop_1852/cleanup_completed.json`
- `runs/mainline_20260926/recovery_loop_1852/actual_runtime_front61.json`

旧运行目录不再作为可启动基线。当前仿真运行v3冻结资产；用户随后要求取消短段执行中重复准入，该后续源码修改需独立构建验证后才能替换当前运行安装，不能把v3运行证据当作新版本验收。
