#!/usr/bin/env python3
"""多发布者示例：一个节点发布多个话题"""

import threading
import astribot_ros_middleware as ast_ros_middleware
from std_msgs.msg import String, Int32, Float32


def timeout_exit(node, timeout=3.0):
    """超时后强制退出"""
    import time
    import os

    time.sleep(timeout)
    print(f"\n超时 {timeout} 秒，程序退出")

    # 销毁节点
    try:
        node.destroy_node()
    except Exception:
        pass

    # 关闭ROS
    ast_ros_middleware.shutdown()

    # 强制退出
    os._exit(0)


if __name__ == "__main__":
    # 初始化ROS
    ast_ros_middleware.init()

    # 创建节点
    node = ast_ros_middleware.Node("multiple_publishers_example")
    logger = node.get_logger()

    # 创建多个发布者
    pub_string = node.create_publisher(String, "topic_string", qos_profile=10)
    pub_int = node.create_publisher(Int32, "topic_int", qos_profile=10)
    pub_float = node.create_publisher(Float32, "topic_float", qos_profile=10)

    logger.info("已创建3个发布者")
    logger.info("  - /topic_string (std_msgs/String)")
    logger.info("  - /topic_int (std_msgs/Int32)")
    logger.info("  - /topic_float (std_msgs/Float32)")
    logger.info("(程序将在3秒后自动退出)")

    # 启动超时退出线程
    timeout_thread = threading.Thread(
        target=timeout_exit, args=(node, 3.0), daemon=True
    )
    timeout_thread.start()

    # 创建频率控制器
    rate = ast_ros_middleware.create_rate(1)  # 1Hz

    # 发布消息
    try:
        for count in range(10):
            # String消息
            msg_str = String()
            msg_str.data = f"Message {count}"
            pub_string.publish(msg_str)

            # Int32消息
            msg_int = Int32()
            msg_int.data = count
            pub_int.publish(msg_int)

            # Float32消息
            msg_float = Float32()
            msg_float.data = count * 0.1
            pub_float.publish(msg_float)

            logger.info(f"已发布第{count+1}组消息")
            rate.sleep()

    except (KeyboardInterrupt, SystemExit):
        logger.info("程序被中断")
    except Exception as e:
        logger.error(f"程序异常: {e}")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass  # 可能已经被超时线程销毁
        ast_ros_middleware.shutdown()
