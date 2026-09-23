# FoundationPose 容器环境的受控准备与安装

此脚本用于本机 Ubuntu 22.04 amd64 的首次 Docker Engine + NVIDIA Container Toolkit 安装。Docker 是本次选定的隔离与复现路径，并非 FoundationPose 算法必需条件；FoundationPose 的环境说明也提供主机依赖安装路径。

官方依据（2026-09-23 核查）：

- [Docker Ubuntu 安装](https://docs.docker.com/engine/install/ubuntu/)：官方签名 APT 源、Engine/CLI/containerd/Buildx/Compose 包及冲突包清单。
- [NVIDIA Container Toolkit 安装](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)：stable APT 源，四个 Toolkit 包及 `nvidia-ctk runtime configure --runtime=docker`。
- [Docker daemon 参数](https://docs.docker.com/reference/cli/dockerd/) 与[网络/防火墙说明](https://docs.docker.com/engine/network/packet-filtering-firewalls/)：默认网络可能新建桥接接口并修改路由或防火墙。

## 两个阶段

准备必须使用普通用户运行，证据目录必须为新的绝对路径，仅允许英文字母、数字、`/`、`_`、`.`、`-`，防止路径改变 APT 源字段或替换表达式：

```bash
bash tools/vision/foundationpose/install_container_prerequisites.sh \
  --prepare --evidence-dir /absolute/path/to/new-container-evidence
```

准备只写证据目录。Ubuntu、Docker、NVIDIA 的源、密钥、APT lists/cache/state/log 均放在目录内；私有 `APT_CONFIG` 排除主机 APT 配置和 hooks，不写 `/etc`，不安装包，不启动服务。下载连接有超时，APT metadata update 限时 300 秒。主机须已具备 `python3`、`apt`、`curl`、`gpg`、系统 CA 与 Ubuntu archive keyring；缺失时拒绝，不在导航期间自动补装。检查 `dependency-versions.txt`、`apt-policy.txt`、`requested-packages.txt`、`planned-packages.txt` 和两次模拟日志。顶层包和全部新增依赖均冻结版本。

安装须等到约定时间已到，且导航会话所有者明确释放维护窗口；时间到本身不构成释放。完成独立脚本审查后，由操作者显式运行：

```bash
sudo bash tools/vision/foundationpose/install_container_prerequisites.sh \
  --install --navigation-released \
  --evidence-dir /absolute/path/to/new-container-evidence
```

`--navigation-released` 是操作者对会话归属和释放状态的明确确认，脚本不会推断 ROS domain、时间或进程归属。脚本不读取、保存或代输密码。它也不设置 `policy-rc.d` 或自动关闭 Docker 网络。Docker 包安装可能立即启动服务并创建默认桥接网络，因此不得提前执行安装阶段。

## 保护范围

- 只首次安装。发现已有 Docker/containerd/runc/Toolkit、旧运行状态、现存 systemd 单元或相关 APT 源时拒绝，不卸载、不迁移、不覆盖。重复安装安全拒绝；失败重跑需先审查已产生的包和配置，脚本不自动回滚或清理。
- 使用官方 HTTPS、各源 `Signed-By` 密钥，不使用 `curl | sh`，不关闭签名校验，不启用 NVIDIA experimental 源。
- 不运行 `upgrade`。每次 APT 模拟均拒绝 removal/purge、任何现有包版本变更及驱动/CUDA/ROS/kernel 包新增；真实安装使用 `--no-remove --no-upgrade --no-install-recommends`。依赖无法在这些边界内解决时停止。
- 安装前重核完整包清单和冻结交易；不同则要求在新目录重新准备。现有驱动/CUDA/ROS不作为升级或修复对象。
- `/etc/docker/daemon.json` 存在时先备份，不存在时记录 `ABSENT`。保留 dpkg 既有配置，使用官方 `nvidia-ctk` 增加 NVIDIA runtime，不把它设为默认 runtime，不更改既有 daemon 的网络策略。记录配置校验与 Docker 重启结果。
- 不拉取镜像、不执行任何容器、不启动 GPU 工作负载、不修改 docker 组成员。使用 `sudo docker`；docker 组属于接近 root 的权限，不在本步骤授予。

## 验证和限制

安装完成会记录包版本、Docker/Toolkit 版本、已注册 runtime、前后网络接口/路由与 GPU UUID/驱动版本。`INSTALLED` 仅表示基础包、daemon 配置和服务检查完成；不表示 GPU 容器、FoundationPose 推理、ROS 图像链路或导航共存已经验收。

脚本拒绝已有配置冲突，不会替操作者解除 APT 锁、终止其他包管理进程或重启主机。若安装中途失败，已安装的包及官方源可能保留，Docker 也可能已启动；查看日志后处理，不要盲目重复执行。证据目录使用私有权限，daemon 备份可能含原有代理配置，不应公开上传。
