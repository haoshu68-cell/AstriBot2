#!/usr/bin/env python3
"""发布者示例：发布字符串消息"""

import astribot_ros_middleware as ast_ros_middleware
from std_msgs.msg import String

if __name__ == "__main__":
    # 初始化ROS
    ast_ros_middleware.init()

    # 创建节点
    node = ast_ros_middleware.Node("publisher_example")
    logger = node.get_logger()

    # 创建发布者
    publisher = node.create_publisher(String, "chatter", qos_profile=10)
    logger.info("发布者已创建，话题: /chatter")

    # 创建频率控制器 (10Hz)
    rate = ast_ros_middleware.create_rate(10)

    # 发布消息
    try:
        for count in range(100):
            msg = String()
            msg.data = f"Hello World {count}"
            publisher.publish(msg)
            logger.info(f"发布: {msg.data}")
            rate.sleep()
    except KeyboardInterrupt:
        logger.info("程序被中断")
    finally:
        node.destroy_node()
        ast_ros_middleware.shutdown()
