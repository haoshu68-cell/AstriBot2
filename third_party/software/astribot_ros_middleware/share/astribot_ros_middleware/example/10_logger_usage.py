#!/usr/bin/env python3
"""日志器使用示例：演示不同级别的日志输出"""

import astribot_ros_middleware as ast_ros_middleware
import time

if __name__ == "__main__":
    # 初始化ROS
    ast_ros_middleware.init()

    # 创建节点
    node = ast_ros_middleware.Node("logger_example")
    logger = node.get_logger()

    # 不同级别的日志输出
    logger.debug("这是DEBUG级别日志")
    logger.info("这是INFO级别日志")
    logger.warn("这是WARN级别日志")
    logger.error("这是ERROR级别日志")

    # 在循环中使用日志
    logger.info("开始计数...")
    for i in range(5):
        logger.info(f"计数: {i+1}/5")
        time.sleep(0.5)

    logger.info("计数完成")

    # 条件日志
    value = 42
    if value > 40:
        logger.warn(f"值 {value} 超过阈值 40")

    # 异常日志
    try:
        result = 10 / 0
    except ZeroDivisionError as e:
        logger.error(f"发生错误: {e}")

    # 销毁节点
    node.destroy_node()

    # 关闭ROS
    ast_ros_middleware.shutdown()
