#!/usr/bin/env python3
"""使用全局API的发布者示例"""

import threading
import time
import astribot_ros_middleware as ast_ros_middleware
from std_msgs.msg import String


def shutdown_timer():
    time.sleep(3)
    print("3秒已到，程序即将退出...")
    ast_ros_middleware.shutdown()


def main():
    ast_ros_middleware.init(node_name="global_publisher")

    timer_thread = threading.Thread(target=shutdown_timer, daemon=True)
    timer_thread.start()

    publisher = ast_ros_middleware.create_publisher(
        String, "global_chatter", qos_profile=10
    )
    print("全局发布者已创建，话题: /global_chatter")

    rate = ast_ros_middleware.create_rate(10)

    try:
        count = 0
        while ast_ros_middleware.ok():
            msg = String()
            msg.data = f"Global message {count}"
            publisher.publish(msg)
            print(f"发布: {msg.data}")
            count += 1
            rate.sleep()
    except KeyboardInterrupt:
        print("中断发布")

    ast_ros_middleware.shutdown()


if __name__ == "__main__":
    main()
