#!/usr/bin/env python3
"""基础节点创建和使用示例"""

import astribot_ros_middleware as ast_ros_middleware

if __name__ == "__main__":
    # 初始化ROS
    ast_ros_middleware.init()

    # 检测ROS版本
    ros_version = ast_ros_middleware.get_ros_version()
    print(f"当前ROS版本: ROS{ros_version}")

    # 创建节点
    node = ast_ros_middleware.Node("basic_node_example")

    # 获取日志器并打印信息
    logger = node.get_logger()
    logger.info("节点已成功创建")
    logger.info(f"节点名称: {node.node_name}")

    # 销毁节点
    node.destroy_node()
    logger.info("节点已销毁")

    # 关闭ROS
    ast_ros_middleware.shutdown()
