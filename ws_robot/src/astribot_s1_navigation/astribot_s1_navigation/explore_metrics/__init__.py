"""探索评测的纯逻辑计算。无 ROS 依赖，可离线复核、可单测。

模块划分：
  geometry.py      几何：横向偏差、弧长、平滑度、超调、漂移、足迹半径
  signals.py       时序信号：拍率、变号计数、互相关时延、段落检测
  scan_metrics.py  激光：净空、居中度、窄段、自滤残留、有效帧率
  round_metrics.py 一轮时序 -> 一行指标（唯一判据出口）
  state_parse.py   /exploration/state 单行串解析 + 协调器计数器提取

`explore_metrics_recorder_node.py` 只负责订阅与落盘，不含任何判据。
"""
