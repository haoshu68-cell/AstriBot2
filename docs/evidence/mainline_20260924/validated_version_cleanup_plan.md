# 已验证版本清理候选（只读计划）

检查时间：2026-09-24T05:30:21.044700+00:00。未删除、未压缩、未构建/ROS。仅检查所列四处目录；`clock_cleanup` runs 目录不存在，实际对比树是 scene_binding 下 timing_*。

|目录（相对仓库）|处置|文件占用 MiB|依据|
|---|---|---:|---|
|runs/mainline_20260924/scene_binding_first_stage/build|ARCHIVE_THEN_DELETE|32.24|旧首段构建树；当前 install 无指向此树的符号链接；保留整个构建证据于压缩包。|
|runs/mainline_20260924/scene_binding_first_stage/timing_build|ARCHIVE_THEN_DELETE|29.36|clock cleanup 对比构建，已由 capture 候选接替。|
|runs/mainline_20260924/scene_binding_first_stage/timing_install|ARCHIVE_THEN_DELETE|7.52|clock cleanup 对比安装，当前 M2/M3 已审 overlay 未引用。|
|runs/mainline_20260924/scene_binding_first_stage/install|KEEP|7.51|M3 runtime_overlay 直接引用；M2 capture_overlay 也先 source 旧 overlay。|
|runs/mainline_20260924/scene_binding_first_stage/capture_build|KEEP|32.38|当前 M2 capture 候选构建。|
|runs/mainline_20260924/scene_binding_first_stage/capture_install|KEEP|7.55|M2 当前 execution_response overlay 显式使用。|
|runs/mainline_20260924/scene_binding_first_stage/source|KEEP|0.24|capture_build 的 CMAKE_HOME_DIRECTORY 指向本源副本，仍是候选构建依赖。|
|runs/mainline_20260924/scene_binding_first_stage/capture_evidence/source|ARCHIVE_THEN_DELETE|0.24|只读验证源副本，完整映射压缩留档；不删同级 raw/journal/CDR。|
|runs/mainline_20260924/scene_binding_first_stage/timing_evidence/source|ARCHIVE_THEN_DELETE|0.07|clock cleanup 的 RED/GREEN 源对比副本，完整压缩留档；不删同级 raw/journal/CDR。|
|runs/mainline_20260924/trajectory_time_scaling/before|ARCHIVE_THEN_DELETE|0.07|已验证补丁的旧源对比；docs/evidence/trajectory_time_scaling_20260924/before 已有逐字节副本。|
|runs/mainline_20260924/trajectory_time_scaling/mtc_build|KEEP|13.13|当前 M2 time_scaling 候选构建，result.json 的编译依赖证据仍引用。|
|runs/mainline_20260924/trajectory_time_scaling/install|KEEP|6.54|M2 overlay 显式选用 manipulation/MTC 安装。|
|runs/m1_transport_20260924/build|ARCHIVE_THEN_DELETE|28.28|旧首段构建，对比版本已被 next/full 候选替代；无符号链接依赖。|
|runs/m1_transport_20260924/install|ARCHIVE_THEN_DELETE|6.91|旧首段安装；所审当前 M2/M3 overlay 链未引用，文档旧命令作为历史保留并映射到归档。|
|runs/m1_transport_20260924/next_build|ARCHIVE_THEN_DELETE|29.84|next_install 为独立普通文件，无到构建树的符号链接；旧构建可完整归档。|
|runs/m1_transport_20260924/next_install|KEEP|6.98|READY→M1_scene_prepare overlay 显式 source，当前 M2/M3 overlay 的间接依赖。|
|runs/m1_transport_20260924/full_build|KEEP|49.52|M1 当前完整链工作候选；不是废弃对比。|
|runs/m1_transport_20260924/full_install|KEEP|15.36|M1 当前完整链安装；不是已被替代的旧版本。|
|runs/m1_transport_20260924/payload_fix|ARCHIVE_THEN_DELETE|3.46|早期 payload 两个对比测试二进制，full_build/full_install 已有后续候选；整个目录压缩留档。|

可归档后回收约 138.0 MiB（按普通文件分配块统计，不含目录；不是已删除量）。

关键 KEEP：旧 scene_binding/install 仍是 M3 直接依赖；m1/next_install 虽旧，仍被 READY→M1_scene_prepare 间接加载。capture_build/install、trajectory mtc_build/install、M1 full_build/install 保持。

原 journal、日志、raw CDR 和协议目录不列作废弃实现。选中的 build 树也须整体归档，保留其中测试证据。已有 docs/evidence 源副本和报告是留档，但不等于已完成压缩校验；仅 trajectory_time_scaling/before 已逐文件确认与 docs 副本一致。

唯一符号链接是 capture_error_cdr 下 torso_controller_goal.cdr.tmp → /dev/full：保留为故障注入证据且禁止跟随。扫描根目录内未发现压缩包。

每个候选的精确原路径、建议压缩包路径及 archive member prefix 见 JSON。root 执行时先复核实时依赖，再压缩、建立每文件 hash/链接目标映射、隔离解压校验和包 hash；完成后才删除精确目录。不得删除父目录或把旧文档路径改写成新产物身份。

本次 /proc 路径扫描未见上述候选正在运行，不能替代执行前重新核对。M1/M3 仍有开发候选，已经按 KEEP 处理。
