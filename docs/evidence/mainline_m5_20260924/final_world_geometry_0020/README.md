# 最终 world 保守几何判据修复

唯一产品修改是私有 `verify_full_transfer.py` 的最终 world 几何校验块。原判据要求最终 world 尺寸等于 scenario 的物理原箱尺寸，和 native DETACH 保留保守附着体形状的契约冲突。scenario、运行时、runner、其他验证器均未修改。

新判据从本父任务 lease/epoch/owner journal 绑定唯一 ATTACH 与 DETACH 的 submission、application、confirmation，核对事务顺序、command_id 和最终库存版本；用 ATTACH 应用的准确 source_revision/source_sequence 定位原始 observation，再匹配同来源版本、ledger_epoch、attachment_revision 和完整物体数组的 confirmed ledger。最终 world 严格比较该来源的保守 primitives。报告保存来源索引、版本、事务、期望及实际 primitives，并明确 independent_pre_detach_scene=false；没有声称独立捕获 DETACH 前 Scene。原最终 EMPTY、独立完整场景读回及业务释放门槛保持不变。

## 离线证据

`scene21_geometry_fixture.json` 的原始 source 是 payload_records[3331]，revision=2/sequence=1418；confirmed ledger 为 [3334]，真实保守尺寸为 0.07146969384566991、0.07146969384566991、0.1314696938456699 m。原始 journal[67/68] 的物理提交/应用记录保留。scene21 实际失败且 UNRESOLVED，没有成功 ATTACH confirmation 或 DETACH。

`check_offline.py` 只编译并执行产品文件中的最终几何块，不导入 ROS。正例使用上述真实原始几何，并显式构造完成 ATTACH/DETACH 的事务确认夹具；这不是把 scene21 记为成功。7 项通过：保守几何接受；尺寸篡改、原尺寸回缩、缺失 source、source 错 revision、ledger 错 revision、DETACH 错 revision 均拒绝。完整脚本语法解析通过。未启动 ROS/GPU、解码、新仿真或额外矩阵。

另一审阅者只读核对相同产品 SHA，未发现当前单 BOX、单父任务契约下的必改问题。真实 MoveIt 保留尺寸的既有 6/6 结果位于 `docs/evidence/m1_transport_20260924/payload_scene_apply_2358/verified_core_test/`；本次未重跑。新增验证器尚待协调者 scene23 实跑，不宣称完整搬运通过。

复现离线检查：在仓库运行 `python3 docs/evidence/mainline_m5_20260924/final_world_geometry_0020/check_offline.py`。before/after、差异、来源和检查输出的 SHA-256 见 manifest.json。产品文件冻结后由协调者复制使用和 Git 留档。
