# 🤖 Astribot SDK  &nbsp;|&nbsp;  控制 • 仿真 • 研究

<p align="center">
  <img src="https://img.shields.io/badge/Ubuntu-22.04%20LTS-orange" />
  <img src="https://img.shields.io/badge/ROS2-Humble-blue" />
  <img src="https://img.shields.io/badge/Python-%E2%89%A53.8-yellowgreen" />
  <img src="https://img.shields.io/badge/License-BSD%203--Clause-brightgreen" />
</p>

---

## 1 · 项目简介

* **官方 SDK**，基于 ROS2 使用 Python 控制 <kbd>Astribot S1</kbd>。


&nbsp;

## 2 · 快速上手

### 2.1 环境配置

#### 前置条件
- **操作系统:** Ubuntu 22.04
- **中间件:** ROS2 Humble
- **网络:** 将您的 PC 设置为 192.168.0.x 网段，x 需要 > 20

#### 安装步骤

```bash
# 下载代码
git clone www.github.astribot_sdk.com

# 安装
cd path/to/astribot_sdk
./install.sh
```


&nbsp;

### 2.2 使用方法

> ⚠️ **重要提示:** 每次使用 SDK 前都需要 source env.sh 脚本，同时需要先激活机器人。
> ⚠️ **说明:** env.sh 用于设置 DOMAIN_ID，机器DOMAIN_ID为25，您也可以将 env.sh 路径添加到 ~/.bashrc 文件中。

```bash
# Source SDK 环境
source /path/to/astribot_sdk/env.sh

# 运行示例代码
python3 example/100-get_robot_properties.py
```



&nbsp;


### 2.3 仿真模式

我们提供 **基于 Mujoco 的仿真环境**。请阅读 GitHub 文档: https://github.com/Astribot-Dev/astribot_simulation



&nbsp;

## 3 · 社区与支持

* **问题反馈 / 功能请求:** <https://github.com/astrihub/AstribotSDK/issues>
* **电子邮件:** support@astribot.com
* **官方网站:** <https://www.astribot.com>

---

© 2024 Astribot Co., Ltd.  基于 **BSD-3-Clause** 许可证发布。

