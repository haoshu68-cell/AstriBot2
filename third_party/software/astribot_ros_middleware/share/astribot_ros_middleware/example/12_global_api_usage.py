#!/usr/bin/env python3
"""全局API使用示例"""

import time
import astribot_ros_middleware as ast_ros_middleware


def main():
    # 1. 检测ROS版本
    ros_version = ast_ros_middleware.get_ros_version()
    print(f"ROS版本: ROS{ros_version}")

    # 2. 初始化ROS
    ast_ros_middleware.init(node_name="api_example", anonymous=True)

    # 3. 检查ROS运行状态
    print(f"ok(): {ast_ros_middleware.ok()}")
    print(f"is_shutdown(): {ast_ros_middleware.is_shutdown()}")

    # 4. 简单循环
    for i in range(3):
        if ast_ros_middleware.ok():
            print(f"循环 {i+1}")
            time.sleep(0.5)

    # 5. 关闭ROS
    ast_ros_middleware.shutdown()
    print(f"关闭后 is_shutdown(): {ast_ros_middleware.is_shutdown()}")


if __name__ == "__main__":
    main()
