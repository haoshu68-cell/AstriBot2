# 业务场景与设备标记：实现与验证记录

日期：2026-09-21。

## 结果

三个功能包构建成功：`astribot_map_manager`、`astribot_operator_backend`、`astribot_operator_station`。累计 16 个测试程序、64 个 GoogleTest 用例，最新结果均为零失败、零错误。完整明细与时间戳见 `verification.json`，原始测试 XML 位于 `gtest/`。

本次没有新增运行时 Python 或采集脚本。地图管理器保存场景与设备，操作后端负责控制权及执行上下文检查，Qt/RViz 页面负责编辑和显示。

## 本次覆盖

- 设备与场景落盘、重启读取、版本冲突与旧引用拒绝。
- 五种业务类型的页面选择、位置/停靠/等待点标记、复核状态、设备停用。
- 同图共享设备与场景切换复用地图；不同地图、人工跨楼层及加载中断恢复。
- 地图范围/未知或占用栅格检查复用已有工位校验；错误坐标系、尺寸和版本拒绝。
- 观察模式禁止写入；过期场景拒绝导航；当前场景正常提交导航给假动作服务器。
- 整个 Workstation 中的设备选点不追加导航路线点。
- 地图尚未就绪时仍可显式请求加载场景，已中断事务需要先恢复。
- 现有地图管理、规划事务、回放、录制、工作站及循环路线相关回归。

测试过程中新增的 Workstation 假目录缺少现有 MapPage 必需的 `floor` 和 `transaction` 字段，造成一次失败。补齐完整测试数据后，Workstation 与 ScenePage 两组重新执行成功。最后一处界面修改将原始坐标 JSON 改为带单位的坐标/角度显示，并再次通过这两组测试。其余组沿用同轮源码验证结果。

## 证据边界

ROS 测试使用隔离域和假后端，Qt 使用 offscreen。截图是测试页面渲染，不能作为 Gazebo 或真机实测证据。未启动/关闭共享仿真，未操作真实底盘、机械臂或设备；未验证实景重定位、设备几何标定或业务动作。

构建与安装在 `/tmp/astribot_scenes_build`、`/tmp/astribot_scenes_install`。未覆盖当前运行栈使用的 `ws_robot/install`，所以现有 RViz 不会热更新。

## 下次启动使用候选版本

在下一次完整仿真启动的终端和 RViz 终端中，依次加载 ROS2、原工作空间、候选安装，然后沿用现有完整仿真启动流程：

```bash
source /opt/ros/humble/setup.bash
source /home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/install/local_setup.bash
source /tmp/astribot_scenes_install/local_setup.bash
```

需要三个包同时使用候选版本；不能只更换 RViz 插件而仍使用旧后端。RViz 载入候选包内 `config/operator.rviz`，或在个人配置中添加 `/operator/equipment_markers` 的 MarkerArray 显示。

临时构建安装会随系统清理失效，正式部署需从仓库重新构建上述三个包。业务目录的默认保存位置是用户持久数据目录，与临时构建目录无关。旧 `/tmp/astribot_map_catalog` 未迁移；配置与迁移边界详见 `docs/manuals/RVIZ_BUSINESS_SCENES.md`。
