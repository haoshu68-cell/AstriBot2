# 真机启动入口

统一入口为 `tools/s1_hardware_bringup.sh`，支持 `start / stop / status / verify`。
控制器选项为 `--controller mppi` 或 `--controller rpp`，默认 MPPI。
例如：`bash tools/s1_hardware_bringup.sh verify --controller rpp`。
`--help` 在 ROS 环境加载、文件写入和进程操作之前返回，可离线查看。

`tools/s1_hardware_bringup_rpp.sh` 保留为五行兼容入口，转发到同目录主脚本并选择 RPP。
部署旧 RPP 入口时必须同时部署主脚本，不再维护独立启动逻辑与 MD5 漂移检查。
两种控制器共用主脚本的厂商 SLAM 保护列表；RPP 不再沿用旧副本中将厂商 SLAM 列入清理范围的差异。
控制器验收分别读取 MPPI 的 vx_max/vy_max 和 RPP 的 desired_linear_vel；RPP 回落值无法区分参数漏传时仍明确提示。

`--keep-nav2-yaml` 的空操作兼容已移除，传入时提示迁移并以状态码 2 退出；现在始终保留安装配置。
没有在仓库脚本中发现其它调用者，仓库外的部署调用需移除此参数。

本次仅实施 A1～A4：去除未使用函数与无条件包装、删除空操作兼容、归档历史说明、合并控制器副本。
默认 DRIVE=true、自动探索、RViz、完整验收等 B 类事项没有调整；`--no-drive` 仍不等同于纯只读启动。
历史说明见 [归档](legacy/HARDWARE_BRINGUP_NOTES.md)，其中旧实测数据不代表当前验证结果。

验证：两个脚本 bash -n、11 个内嵌 Python 块编译、隔离参数解析、RPP 转发、伪 ROS 参数服务下的双控制器选择/越限/缺参检查通过。
外部验证脚本位于 `/tmp/astribot-bringup-audit/check.py`。没有执行 start/stop/status/verify，也没有启动或控制机器人。
