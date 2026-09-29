# C++ 迁移 Git 对照与修复记录

本次变更留存在本地分支 **`codex/pybind-migration-20260921`**。起点为 `a7026bcd`。记录分支使用独立暂存区生成，没有切换共享工作分支 `chassis-effort-drive`，没有重置工作区，也没有将其他任务的相机、地图、操作界面改动混入提交。尚未推送远端。

## 已保存的检查点

| 提交 | 范围 | 状态 |
|---|---|---|
| `c01c0bb808818d0a643457bf5ff420a576e05183` | 三个 native 包、几何/桥接核心、直接 ROS 节点与差分验证工具 | 保留尚未迁移消费者使用的三个绑定；不是全项目去绑定完成 |
| `4dcf0bb67ca08a0c3983f15d185d0d2862423c7c` | 已迁移节点的唯一 C++ launch/console/main 入口，旧 Python 启动入口退役 | 已从该 Git 树解出源码，入口测试 16/16；不代表共享安装已更新 |
| `942a099c9f55f3966656e5b2ca3884d82c516081` | 速度转换动态参数/退出码修复、几何完成通知延迟修复 | cmd 30 通过/12 项旧接口例外跳过；导航 CTest 3/3、几何 CTest 5/5 与原始性能证据 |
| `ba85229f10a2be91ed7b350248833816887c68d1` | 臂展开限速动态参数、非法原子更新拒绝与启动失败退出码 | 22 场景通过，Release CTest 7/7；保留 TF/迟滞/周期重发规则 |
| `ee14a5675882165fea92ec5c132ef6616ef7d0b1` | 第一轮原始回归、性能、源哈希和 85 文件盘查 | 原始失败及成功证据完整留存 |
| `262f604e88e0abbae72bd7bd5c88a916dfe34629` | 17 个已退役 Python 参考模块搬出生产安装；测试显式加载 | 干净安装、83 回归、226 ROS 对照、52 参数用例及来源检查 |
| `bb94029dcb04980c897ef91809460f8926fab64c` | 臂底盘 Twist 耦合完整 C++ 运行时与旧生产包删除 | 10,027 核心输入、31 pytest、2 CTest、8,000 消息配对及尾延迟例外 |
| `fa91842ff90b478709b534f9791e876d55f1dd1d` | 导航任务仲裁 C++ action owner、唯一 launch 入口 | 安装后 77 测试、1 CTest、每实现 500 假 Nav2 任务配对 |
| `80a70bdee4ee88c344fc45112e01cf8027aa0e96` | map→odom 与跨域地图中继原生包；删除两旧入口、三生产模块 | 整包 4 CTest：34 核心、64 TF、46 中继；干净安装与各4项安装smoke；原始性能、SIGINT和环境失败留档 |
| `f9790e2939fd7f88ecfee1dd3830ffec773a8ea0` | 策略持续融合/健康/多边形扫掠/风险 C++ 内核及数值门槛修复 | 8/8 CTest、126 差分、1,783 额外风险回放、sanitizer及AB/BA原始性能；两个策略入口尚未切换 |
| `1b51e14e9f92b3dc17206b6640858bf8f7cf337b` | 直接 C++ observer 候选、包络/观测适配、完整 wire 版本及边界修复 | 独立构建安装11/11 CTest；34组配对ROS场景、安装34项、1600性能帧；JSON整数/特殊字符串差异待补，未切入口 |

新增检查点 **`b7a2240b2e4bacc08a0b3c5934dbbc6c12e4af30`** 保存策略任意整数修复、P2行为/路径证据核心及原始验证结果（318个变更文件）。记录树的198个构建输入与实测快照逐字节核对一致；独立13/13 CTest、82/82成对安装ROS和1600性能帧。特殊字符串、Stamp和controller仍未完成，未删除这两个生产入口。中断后丢失的临时运行结果已重新生成，未冒充归档证据。

新增检查点 **`f4feccdf91932bb9626c78359b86bf7a3d0fca50`** 保存JSON字符串及原ROS转换边界修复。200个构建/测试输入与Git记录逐字节一致；14/14 CTest、108/108成对安装ROS、47+28适配器/字符串sanitizer和1600输出匹配性能帧通过。中间真实失败与错误文本断言修正分别留档，未删除尚在用的策略入口/绑定。

后续修复与清单按独立提交追加，使用下方日志命令查看最新链。Python 参考类用于差分验证，仍在运行的未迁移 Python 消费者单独列在盘查清单；没有用删除导入或静默 Python 回退伪装成去 pybind。

## 如何对照问题

```bash
# 查看这条迁移链及每次改动的范围
git log --oneline --stat a7026bcd..codex/pybind-migration-20260921

# 对照迁移前后某个文件；将路径替换为实际问题文件
git diff a7026bcd codex/pybind-migration-20260921 -- \
  ws_robot/src/astribot_s1_navigation/launch/navigation.launch.py

# 在新目录复现记录版本，保持当前共享工作区不动
git worktree add --detach ../astribot-cpp-review codex/pybind-migration-20260921
```

从独立工作树构建时只扫描 `ws_robot/src`，使用独立 build/install/log，避免发现 `runs` 下的重复 ROS 包。修复应在新分支中针对具体提交进行对照，先复现原始失败输入，再补充边界/协议回归。不要把整栈资源清理或真机动作作为普通代码回归步骤。

## 证据与复现边界

- 迁移范围、源文件哈希和验证分层：[去 pybind 记录](PYBIND_REMOVAL_20260921.md)。
- 剩余 Python 文件、接口、生产调用者及迁移次序：[逐文件盘查](PYTHON_CPP_REMAINING_20260921.md)。
- 原始测试、失败样本、性能数据：[证据目录](evidence/pybind_removal_20260921/)。

共享工作树中的 `navigation.launch.py`、`nav2_full_bringup.launch.py` 和部分 package.xml 还含其他任务的改动。记录分支仅收录迁移相关修改，所以这些完整文件的哈希可能与共享工作树不同；独立提交树已另测入口选择，不能用工作区全文件哈希替代提交树的身份。构建候选及报告内的源码/ELF SHA256 用来关联实测版本。过去的中间数据保留原始哈希，不重新标记为最新候选。

本地 Git 记录可以追溯；它不表示主工作分支已合并、共享安装已替换、所有 pybind 已删除，或已通过整机闭环/真机验收。

`tools/sim/verify_multi_load_dynamic.py` 在本轮开始时属于未收录的并行验证脚本。为不混入其完整改动，只将本轮引用迁移的精确 patch 和前后哈希存入 `reference_retirement/untracked_validation_consumer.*`；需要时可从对应 Git 提交取出该补丁核对。其余四个共享 launch/package.xml 的选择性收录规则继续保持。
