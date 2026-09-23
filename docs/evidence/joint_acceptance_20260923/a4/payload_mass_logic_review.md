# 有载质量准入修复：实施前逻辑复核

属于 A4，仍使用14:13:31–15:43:31总预算。导航任务继续冻结当前EMPTY N4安装，本次只修改共同源码并用独立构建验证。

发现：固定包络当前只拒绝非空附件+mass<=0；0.001kg可以代表真实2kg附件，未绑定账本总质量。

数据流：独立物理库存 → 已确认AttachmentState → 现有C++ payload::Consumer（会话/来源、摘要、版本、ROS+steady租约、重启与重复包）→ 固定包络核心质量核对 → 六消费者。复用Consumer，避免新写一套身份/时效规则；不新增ROS消息字段或第二个物体账本。

核心：通过只读provider取得Consumer当前有效State。非空geometry时必须存在同attachment_revision、完全相同object IDs的全量确认状态；累计真实weight，拒绝非有限、非正单体质量和溢出；请求质量低于总和拒绝，保守高报允许。对已获准包络持续检查，失效锁存并要求新请求；包络有效期不超过账本租约。

空载保持现有A1–A3前置（几何已确认明确EMPTY）；不存在任何“有载缺来源回退空载”。未配置消费者来源时有载拒绝，不能根据首个收到的消息学习可信身份。

节点：显式payload_environment/session_id/source_id参数与geometry/ledger共用配置；callback喂Consumer，core只消费其current；独立wall tick使ROS冻结时仍过期。ACK反馈放大修复保留，重复数据不续期。

离线覆盖：低报/等额/高报、NaN/零值、缺来源、revision或IDs不一致、旧源重复/ROS冻结、持有期间账本撤销与新请求恢复、EMPTY→ATTACHED→EMPTY。保留18项既有链路和权威契约回归。完成构建/离线不宣称有载动态通过。
