#!/usr/bin/env python3
"""使用全局API的订阅者示例"""

import threading
import time
import astribot_ros_middleware as ast_ros_middleware
from std_msgs.msg import String


def callback(msg):
    print(f"收到消息: {msg.data}")


def shutdown_timer():
    time.sleep(3)
    print("3秒已到，程序即将退出...")
    ast_ros_middleware.shutdown()


def main():
    ast_ros_middleware.init(node_name="global_subscriber")

    timer_thread = threading.Thread(target=shutdown_timer, daemon=True)
    timer_thread.start()

    ast_ros_middleware.create_subscriber(
        String, "global_chatter", callback, qos_profile=10
    )
    print("全局订阅者已创建，话题: /global_chatter")

    print("开始订阅...")
    ast_ros_middleware.spin()


if __name__ == "__main__":
    main()
