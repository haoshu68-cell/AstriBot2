# scene39：PICK后缀均完成，导航准入失败

原始目录：`/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/full_transfer_scene39_world52`。只读复核已关闭场景；未修改配置/脚本，未运行 ROS/GPU/build 或重解码。

实际ATTACH提交/场景应用/后缀采纳/事务确认依次为 journal[70/71/72/73]。采纳事件 `transport_replanned=true`，PICK的两个后缀 returned=adopted=sent 指纹、点数、有序关节名和context/transaction均一致，response保留同份发送元数据。

| 阶段 | 点数 | send → response → terminal → confirmed | 同一UUID |
|---|---:|---|---|
| LIFT，index4/gen5 | 7 | 75 → 81 → 87 → 93 | d98337b1a503959dbe4872a22fe5ac00 |
| TRANSPORT_POSTURE，index5/gen6 | 654 | 96 → 102 → 108 → 114 | 7e598e3261d51a34152e84103e1a2c27 |

两段terminal均success=true/code4，stage_confirmed.children包含对应UUID。因此本场完整PICK动作阶段及重规划后缀交接/执行已获得仿真证据。完整摘要与原始关联事件见 `verification.json`、`stage_excerpt.json`；没有以控制器状态替代目标指纹或UUID。

父反馈[1408]为 NAVIGATE/MANIPULATION_HOLD_CONFIRMED、hold_confirmed=true，接收ROS104.544、steady43242.486098030；[1409]为 NAVIGATE/FIXED_ENVELOPE_REJECTED:ARM_HOLD_UNCONFIRMED，接收ROS104.561、steady43242.518688231。导航Goal UUID始终为空：进入的是导航准入阶段，没有导航执行或PLACE完成证据。保留原失败 `RENEW_REJECTED:FIXED_ENVELOPE_REJECTED:ARM_HOLD_UNCONFIRMED`，导航交接时序由root/M1另核，本复核不推断根因。

原视频owner完整解码499帧，与metadata及连续0–498 sidecar一致。逐帧状态标注：281–293执行LIFT，306–370执行TRANSPORT_POSTURE，371–374进入NAVIGATE准入失败，375–498同reason且authority=RELEASED。497条非空status与原事件一致，初始2帧为空。M5未解码或做像素判断；动作完成依据是journal，不由视频标签单独推出。

父status5、success=false、resources_released=true；cleanup_complete=true、无cleanup_errors，stack stopped/remaining_owned_pids=[]。原停车判据通过，36样本/0.7s，漂移0.000192968m、旋转0.000437762rad、最大命令0。整场仍失败，PLACE未到达，完整成功路径验收未执行；本场仍属于已授权放宽仿真条件。所有原始文件及失败证据保留。
