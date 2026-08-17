#!/usr/bin/env python3
"""订阅者和发布者示例：在同一节点内订阅和发布字符串消息
注意：程序会持续运行等待消息，使用 Ctrl+C 退出，或3秒后自动退出
"""

import threading
import astribot_ros_middleware as ast_ros_middleware
from std_msgs.msg import String


# 全局计数器
publish_count = 0


def callback(msg):
    """回调函数处理接收到的消息"""
    print(f"接收到消息: {msg.data}")


def timer_callback(publisher, logger):
    """定时器回调函数：定期发布消息"""
    global publish_count
    publish_count += 1
    msg = String()
    msg.data = f"Hello from subscriber node! Count: {publish_count}"
    publisher.publish(msg)
    logger.info(f"发布消息: {msg.data}")


def timeout_exit(node, timeout=3.0):
    """超时后强制退出"""
    import time
    import os

    time.sleep(timeout)
    print(f"\n超时 {timeout} 秒，程序退出")

    # 销毁节点以中断 spin()
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
    node = ast_ros_middleware.Node("subscriber_example")
    logger = node.get_logger()

    # 创建订阅者
    subscriber = node.create_subscriber(String, "chatter", callback, qos_profile=10)
    logger.info("订阅者已创建，话题: /chatter")

    # 创建发布者
    publisher = node.create_publisher(String, "response", qos_profile=10)
    logger.info("发布者已创建，话题: /response")

    # 创建定时器，每0.5秒发布一次消息
    timer = node.create_timer(0.5, lambda: timer_callback(publisher, logger))

    logger.info("等待消息...")
    logger.info("(程序将在3秒后自动退出)")

    # 启动超时退出线程
    timeout_thread = threading.Thread(
        target=timeout_exit, args=(node, 3.0), daemon=True
    )
    timeout_thread.start()

    try:
        # 开始循环接收消息
        node.spin()
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
