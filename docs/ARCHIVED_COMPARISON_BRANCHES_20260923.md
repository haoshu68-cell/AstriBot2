# 历史对照分支归档

2026-09-23按用户要求统一在`chassis-effort-drive`开发。三条09-21比较/迁移记录分支已删除其分支引用；每个原tip及其全部历史先由annotated tag和完整git bundle保存并验证。没有将已暂停的候选实现合入当前源码。

| 原分支 | 保留的Git标签 | 原tip |
|---|---|---|
| codex/cpp-migration-report-20260921 | archive/20260923/cpp-migration-report-20260921 | fb245ca445f29b64dfeda05dba25b1b033d52f7d |
| codex/pybind-migration-20260921 | archive/20260923/pybind-migration-20260921 | a309d9bfcfef13d9dac2aaf9432b4fdd6804a6ea |
| codex/stamp-candidate-paused-20260921 | archive/20260923/stamp-candidate-paused-20260921 | fcc5ccb5a7c9425b00b37c43bd23b7cd4b120cee |

旧报告中命令若引用这些分支，改用本表标签或原SHA。标签仅保留历史证据，不是第二条开发基线。时间戳扩域候选按此前决定已停用，不恢复。其历史方案/失败日志只供审计，不作为当前实施计划；正式附件来源/ArmHold/握手联合接入子任务以joint_acceptance_20260923/plan.md为准；整体仍沿统一导航I0.2→I1–I6任务链，共同基线索引为docs/evidence/unified_navigation_resume_20260921/JOINT_BASELINE_20260923.md。

bundle路径和SHA256及`git bundle verify`结果见 `docs/evidence/joint_acceptance_20260923/branch_archive.json`。没有删除仍被运行安装引用的文件或当前未完成任务源码。
