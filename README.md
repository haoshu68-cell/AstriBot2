# 🤖 Astribot SDK  &nbsp;|&nbsp;  Control • Simulate • Research

<p align="center">
  <img src="https://img.shields.io/badge/Ubuntu-22.04%20LTS-orange" />
  <img src="https://img.shields.io/badge/ROS2-Humble-blue" />
  <img src="https://img.shields.io/badge/Python-%E2%89%A53.8-yellowgreen" />
  <img src="https://img.shields.io/badge/License-BSD%203--Clause-brightgreen" />
</p>

---

## 1 · What is this repo?

* **Official SDK** for controlling the <kbd>Astribot S1</kbd> base on ROS2 with python.


&nbsp;

## 2 · Quick Start

###  2.1 Environment Setup
####  Prerequisites
-  **Operating System:** Ubuntu 22.04
-  **Middleware:** ROS2 Humble
-  **Network:** Set your PC into 192.168.0.x ip address, x need to >20

####  Installation 
```bash
# Download the code
git clone www.github.astribot_sdk.com

# Install 
cd path/to/astribot_sdk
./install.sh
```


&nbsp;

###  2.2 Usage

> ⚠️ **Important:** You need to source the env.sh scripts **every time** and also you need to active the robot before using the SDK.
> ⚠️ **Note:** The env.sh is used for set the DOMAIN_ID,robot DOMAIN_ID set to 25， you can also add the env.sh path into ~/.bashrc file

```bash
# Source the SDK environment
source /path/to/astribot_sdk/env.sh

# Run the example code
python3 example/100-get_robot_properties.py
```


&nbsp;


###  2.3 Simulation Mode
We provide a **Mujoco-based simulation environment**. Follow read the github readme: https://github.com/Astribot-Dev/astribot_simulation



&nbsp;

## 3 · Community & Support

* **Issues / Feature Requests:** <https://github.com/astrihub/AstribotSDK/issues>  
* **E-mail:** support@astribot.com  
* **Website:** <https://www.astribot.com>

---
© 2024 Astribot Co., Ltd.  Released under the **BSD-3-Clause** license.
