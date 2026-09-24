# Owner 派生 summary 更正

M2 在首次归档后纠正 scene03 的派生 summary；本目录保存其当前文件的精确字节副本，来源和新旧 SHA-256 见 `correction_manifest.json`。这是报告表述更正，不是原始执行数据改变。原归档 summary 与历史 `copy_manifest.json` 保持原字节及原 hash，不把旧 hash 改成新 hash。

`measured_held_joints` 是 geometry.joints / JointState，不是 ArmHoldStatus。本次原归档已使用该键；这次两个 summary 的实际差异仅以 correction_manifest 的 changed_top_level_fields 为准。

在线 typed Hold 门控实际只核验接收龄 <0.3 s、hold_confirmed 和匹配 hold_id。lease/epoch 由 Action feedback 与 executor status 交叉绑定；没有独立核验 typed 消息自身的 source stamp、valid_until、lease/epoch。原始 typed 消息未独立序列化，因此不能据此宣称完成其源时间/有效期/资源字段的独立审计。现有六方 ACK 与取消释放结论不作扩大。

原 `verification.json` 保留为首次审计记录，其 typed_hold_scope 由本 errata 进一步限定。`copy_manifest.json` 中 all_copy_hashes_match 描述首次复制时的事实；owner 后续更正不应被解释成当前源文件仍等于旧 hash。重现归档脚本会重写历史文件，历史已冻结后不应重跑归档构建步骤。
