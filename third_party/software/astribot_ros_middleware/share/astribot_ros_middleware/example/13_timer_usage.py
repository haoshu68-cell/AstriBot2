#!/usr/bin/env python3
"""定时器示例：使用Timer定期执行回调函数"""

import astribot_ros_middleware as ast_ros_middleware
import time

counter = 0


def timer_callback_1hz():
    global counter
    counter += 1
    print(f"[1Hz定时器] 触发次数: {counter}")


def timer_callback_5hz():
    print(f"[5Hz定时器] 当前时间: {time.time():.2f}")


if __name__ == "__main__":
    # 初始化ROS
    ast_ros_middleware.init()

    # 创建节点
    node = ast_ros_middleware.Node("timer_example")
    logger = node.get_logger()

    logger.info("创建定时器...")

    # 创建1Hz定时器（每秒触发一次）
    timer_1hz = ast_ros_middleware.create_timer(1.0, timer_callback_1hz)
    logger.info("1Hz定时器已创建")

    # 创建5Hz定时器（每0.2秒触发一次）
    timer_5hz = ast_ros_middleware.create_timer(0.2, timer_callback_5hz)
    logger.info("5Hz定时器已创建")

    logger.info("定时器运行中，按Ctrl+C退出...")

    # 让定时器运行一段时间
    try:
        time.sleep(5.0)
    except KeyboardInterrupt:
        logger.info("收到中断信号")

    # 关闭定时器
    logger.info("关闭定时器...")
    timer_1hz.shutdown()
    timer_5hz.shutdown()

    logger.info(f"1Hz定时器总共触发了 {counter} 次")

    # 销毁节点
    node.destroy_node()

    # 关闭ROS
    ast_ros_middleware.shutdown()
