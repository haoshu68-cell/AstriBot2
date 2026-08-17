#!/usr/bin/env python3
"""多订阅者示例：一个节点订阅多个话题
注意：程序会持续运行等待消息，使用 Ctrl+C 退出，或3秒后自动退出
"""

import threading
import astribot_ros_middleware as ast_ros_middleware
from std_msgs.msg import String, Int32, Float32


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


class MultiSubscriberExample:
    def __init__(self):
        self.node = ast_ros_middleware.Node("multiple_subscribers_example")
        self.logger = self.node.get_logger()

        # 创建多个订阅者
        self.sub_string = self.node.create_subscriber(
            String, "topic_string", self.string_callback, qos_profile=10
        )
        self.sub_int = self.node.create_subscriber(
            Int32, "topic_int", self.int_callback, qos_profile=10
        )
        self.sub_float = self.node.create_subscriber(
            Float32, "topic_float", self.float_callback, qos_profile=10
        )

        self.logger.info("已创建3个订阅者")
        self.logger.info("  - /topic_string (std_msgs/String)")
        self.logger.info("  - /topic_int (std_msgs/Int32)")
        self.logger.info("  - /topic_float (std_msgs/Float32)")
        self.logger.info("等待消息...")
        self.logger.info("(程序将在3秒后自动退出)")

        # 启动超时退出线程
        self.timeout_thread = threading.Thread(
            target=timeout_exit, args=(self.node, 3.0), daemon=True
        )
        self.timeout_thread.start()

    def string_callback(self, msg):
        self.logger.info(f"[String] {msg.data}")

    def int_callback(self, msg):
        self.logger.info(f"[Int32] {msg.data}")

    def float_callback(self, msg):
        self.logger.info(f"[Float32] {msg.data:.2f}")

    def spin(self):
        try:
            self.node.spin()
        except (KeyboardInterrupt, SystemExit):
            self.logger.info("程序被中断")
        except Exception as e:
            self.logger.error(f"程序异常: {e}")
        finally:
            try:
                self.node.destroy_node()
            except Exception:
                pass  # 可能已经被超时线程销毁


if __name__ == "__main__":
    # 初始化ROS
    ast_ros_middleware.init()

    example = MultiSubscriberExample()
    example.spin()

    # 关闭ROS
    ast_ros_middleware.shutdown()
