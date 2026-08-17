import os
import sys

# 添加到 sys.path 来查找 astribot_rclcpp_py_ext_pybind11 模块
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
from .astribot_rclcpp_py_ext_pybind11 import RclcppInit, RclcppShutdown
