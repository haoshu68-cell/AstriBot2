# Fast DDS endpoint 级发现失效(实机,未解决)

**日期** 2026-08-31 · 机器人 `orin`(aarch64)
**状态** ⚠️ **未解决**。已确认无法通过"重启我们全部进程"恢复,待整机重启后复验。
**影响** nav2 无法完成 lifecycle 握手 → `controller_server` 永不 activate → 导航与路径跟踪无法验证

---

## 1 · 症状

### nav2 侧(生产链路上的实际后果)

```
controller_server:   进程活着(pid 953556，状态 Sl)，local_costmap 已创建，自身日志正常
lifecycle_manager:   Waiting for service controller_server/get_state...  × 251 次
                     累计等待 > 222 秒，从未收敛
```

第一次启动时是对称的另一半:

```
bt_navigator: failed to send response to /bt_navigator/get_state (timeout):
              client will not receive response
   at .../rmw_fastrtps_shared_cpp/src/rmw_response.cpp:154
   at .../rcl/src/rcl/service.c:314
lifecycle_manager: Failed to change state for node: bt_navigator.
                   async_send_request failed.
                   Failed to bring up all requested nodes. Aborting bringup.
```

`bt_navigator` **发出了**响应,`lifecycle_manager` **收不到**。

> 注意报错路径是 `/home/astribot/workspace/astribot_ros/...` ——
> 跑的是**厂商自己编译的 `rmw_fastrtps`**,不是 `/opt/ros/humble` 那份。

### 关键刻画:participant 级能发现,endpoint 级不能

连续 80 秒采样(原生 rclpy,绕开 `ros2` CLI 与 daemon):

```
 20s  节点=17  lidar=N  slam=N   lidar_pub=0 scan_pub=0 tf_pub=0
 40s  节点=19  lidar=N  slam=N   lidar_pub=0 scan_pub=0 tf_pub=0
 60s  节点=21  lidar=Y  slam=N   lidar_pub=0 scan_pub=0 tf_pub=0
 80s  节点=22  lidar=Y  slam=N   lidar_pub=0 scan_pub=0 tf_pub=0
```

三个特征:

1. **节点数缓慢爬升**(17→19→21→22),80 秒仍未收敛
2. **发现状态会反转** —— `livox_lidar_publisher` 在不同轮次里出现过 Y→N→Y
3. **最关键:节点被发现了,它的话题发布者数仍是 0**(`lidar=Y` 而 `lidar_pub=0`)

第 3 点是本故障的本质:**participant 级发现(SPDP)能成,endpoint 级发现(SEDP)不成。**
这正好解释 nav2 症状 —— `lifecycle_manager` 看得到 `controller_server` 这个节点,
但发现不到它的 `get_state` **service endpoint**。

### 发布侧完全正常(重要旁证)

```
chassis_odom:  帧=254500 行程=0.324m 跳变=0     ← 内环 25 万帧持续运行
map_odom_tf:   更新=102274 跳变=0 倾角超限=0     ← 它能读到 SLAM 与 odom 的 TF
```

`map_odom_tf` 能正常工作,说明**已建立连接的进程之间通信不受影响**。
坏的只是**新加入 participant 的 endpoint 发现**。

---

## 2 · 已排除的原因(每条都跑了能证伪它的命令)

| 猜测 | 判据 | 结论 |
|---|---|---|
| Fast DDS 共享内存段耗尽 | `ls /dev/shm` → 44 个段**全是** nvidia `nvsci*`,**0 个** fastdds/fastrtps | ❌ DDS 走 UDP，与共享内存无关 |
| 多播发现路径断了 | 239.255.0.1:7400 自环实测 → 收到 `b'probe'` | ❌ 多播通 |
| 网卡/地址丢了 | `eno1` operstate=up carrier=1;`192.168.0.11/24` 在 | ❌ |
| iptables 拦截 | 规则只有 `-i/-o wlP1p1s0 -d 239.255.0.1 -j DROP` | ❌ 不影响 eno1 |
| 系统资源耗尽 | 负载 3.11;内存 6.6G/61G;无 OOM;fd 15360(上限极大);`/dev/shm` 124K/31G | ❌ |
| `ros2-daemon` 缓存陈旧 | 原生 rclpy 绕开 CLI,现象一致 | ❌ |
| DDS 环境变量不一致 | 查询 shell 与发布节点的 `ROS_DOMAIN_ID=25`/`LOCALHOST_ONLY=0`/`RMW`/`FASTRTPS_DEFAULT_PROFILES_FILE` **逐项一致** | ❌ |
| nav2 节点重名(两套实例) | 确实存在(952398 与 953556 两个 `controller_server`);**杀掉重复实例后仍不恢复** | ❌ 不是根因 |
| 重启我们全部进程 | 停掉全部 12 个进程(残留复核为空)后逐层重起;**起到第 2 层就重现** | ❌ 无效 |

---

## 3 · 唯一相关的观察(未验证为根因)

发现不到的进程都是**重进程**:

| 进程 | 线程 | RSS |
|---|---|---|
| `chassis_odom`(厂商 SDK) | **66** | 352 MB |
| `state_bridge`(厂商 SDK) | **66** | 351 MB |
| `voxelslam` | 15 | 547 MB |
| `rviz2` | 15 | 824 MB |

对比能被发现的 `cloud_to_grid`:23 线程。

⚠️ **这只是相关性,不是根因。** 反例:重启后单独起 `livox_lidar_publisher`(19 线程)
能被正常发现且 `pub=1`;加了 `voxelslam` 之后它反而变成发现不到。
所以"线程多所以发现不了"讲不通,**真实机制未知**。

---

## 4 · 排查这类问题的判据教训

这轮排查里我**五次**栽在判据本身写错上,每一次都一度得出相反结论。
记下来是因为这些坑会重复出现:

| # | 错误判据 | 造成的错误结论 | 正确做法 |
|---|---|---|---|
| 1 | `ros2 topic list` 判话题存在 | "链路已通"(实际发布者全 0) | 只看 `Publisher count` |
| 2 | frame 名写 `astribot_torso_link1` | "robot_state_publisher 有问题" | 真名带下划线 `_link_1`;从 `got segment` 日志取名 |
| 3 | 把厂商 SDK 无条件打的 `acquired control rights` 当自己节点的行为 | "chassis_odom 持有控制权" | 看代码:`sdk_session.py:292` 写死 `high_control_rights=False` |
| 4 | 探测窗口只有 8~15 秒 | "全局 DDS 彻底坏了" | **本机 DDS 发现要 30~50 秒**,窗口至少 40s |
| 5 | `pgrep -x controller_server` | "controller_server 没起来"(它活着) | 长路径可执行文件用 `ps -eo args \| grep <路径片段>` |

另外 **`pkill -f <模式>` 在这轮里三次杀掉自己的远端 shell** ——
模式串出现在 `bash -c` 的命令行里,`pkill` 连自己一起杀,表现为命令从中间静默断掉
(exit 255 / 143 / 144)。**清理进程必须先 `ps`/`pgrep` 拿 PID,再 `kill <pid>`。**

---

## 5 · 下一步

**已选方案:整机重启**(由用户执行;代价是厂商本体驱动需要重新启动一次)。

理由:故障跨越了"重启我们全部进程"这道边界,状态残留在厂商栈或更底层。
厂商栈已连续运行 3.5 小时,期间起停过数十个 participant。

### 重启后的复验顺序(每层用 ≥40 秒窗口)

```bash
# 判据脚本骨架：绕开 ros2 CLI，用原生 rclpy，长窗口
python3 - <<'PY'
import rclpy
from rclpy.node import Node
rclpy.init(); n = Node("probe")
end = n.get_clock().now().nanoseconds + 45e9
while n.get_clock().now().nanoseconds < end:
    rclpy.spin_once(n, timeout_sec=0.2)
print("节点数:", len(n.get_node_names_and_namespaces()))
for t in ["/livox/lidar_front", "/map_scan_filtered", "/tf", "/odom"]:
    print(" ", t, "pub =", n.count_publishers(t))
n.destroy_node(); rclpy.shutdown()
PY
```

**判据不是"节点被发现",而是"话题 `pub > 0`"** —— 本故障恰恰是前者成立而后者不成立。

逐层顺序见 [real_robot_chain_verification.md](real_robot_chain_verification.md) §8。

### 若整机重启后仍复现

值得查的方向(本轮未做):

- 抓 7400/7410 端口的 SPDP/SEDP 报文,看 SEDP 是否发出/被丢
- 厂商 `rmw_fastrtps` 与 `/opt/ros/humble` 那份的差异(已知厂商 middle_ware 与 humble
  重叠 184 个包、其中 150 个版本不同)
- Fast DDS 的 `FASTDDS_LOG` / `--log-level debug` 输出
- 是否与"起了多少个 participant"相关 —— 做一个递增压测,记录首次失败时的数量

---

## 相关文档

- [real_robot_chain_verification.md](real_robot_chain_verification.md) —— 生产拓扑与启动顺序
- [vnc_visualization_tutorial.md](vnc_visualization_tutorial.md) —— 可视化
