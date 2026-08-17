#!/usr/bin/env python3
"""频率控制示例：使用Rate控制循环频率"""

import astribot_ros_middleware as ast_ros_middleware
import time

if __name__ == "__main__":
    # 初始化ROS
    ast_ros_middleware.init()

    # 创建节点
    node = ast_ros_middleware.Node("rate_control_example")
    logger = node.get_logger()

    # 创建不同频率的Rate对象
    rate_10hz = ast_ros_middleware.create_rate(10)  # 10Hz
    rate_1hz = ast_ros_middleware.create_rate(1)  # 1Hz

    logger.info("演示10Hz频率控制...")
    start_time = time.time()
    for i in range(20):
        logger.info(f"10Hz循环 {i+1}/20")
        rate_10hz.sleep()
    elapsed = time.time() - start_time
    logger.info(f"10Hz循环完成，耗时: {elapsed:.2f}秒（预期: 2.0秒）")

    logger.info("\n演示1Hz频率控制...")
    start_time = time.time()
    for i in range(3):
        logger.info(f"1Hz循环 {i+1}/3")
        rate_1hz.sleep()
    elapsed = time.time() - start_time
    logger.info(f"1Hz循环完成，耗时: {elapsed:.2f}秒（预期: 3.0秒）")

    # 销毁节点
    node.destroy_node()

    # 关闭ROS
    ast_ros_middleware.shutdown()
