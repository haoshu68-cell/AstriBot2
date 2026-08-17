#!/usr/bin/env python3
"""参数使用示例：声明、获取、设置参数"""

import astribot_ros_middleware as ast_ros_middleware

if __name__ == "__main__":
    # 初始化ROS
    ast_ros_middleware.init()

    # 创建节点
    node = ast_ros_middleware.Node("parameter_example")
    logger = node.get_logger()

    # 声明参数
    node.declare_parameter("robot_name", "astribot")
    node.declare_parameter("max_speed", 1.0)
    node.declare_parameter("enable_debug", False)
    logger.info("参数已声明")

    # 获取参数
    robot_name = node.get_parameter("robot_name")
    max_speed = node.get_parameter("max_speed")
    enable_debug = node.get_parameter("enable_debug")

    logger.info(f"robot_name: {robot_name}")
    logger.info(f"max_speed: {max_speed}")
    logger.info(f"enable_debug: {enable_debug}")

    # 设置参数
    node.set_parameter("max_speed", 2.0)
    node.set_parameter("enable_debug", True)
    logger.info("参数已更新")

    # 重新获取参数
    max_speed = node.get_parameter("max_speed")
    enable_debug = node.get_parameter("enable_debug")
    logger.info(f"更新后 max_speed: {max_speed}")
    logger.info(f"更新后 enable_debug: {enable_debug}")

    # 检查参数是否存在
    has_robot_name = node.has_parameter("robot_name")
    has_unknown_param = node.has_parameter("unknown_param")
    logger.info(f"robot_name 存在: {has_robot_name}")
    logger.info(f"unknown_param 存在: {has_unknown_param}")

    # 获取不存在的参数（使用默认值）
    unknown_value = node.get_parameter("unknown_param", default="default_value")
    logger.info(f"unknown_param (默认值): {unknown_value}")

    # 销毁节点
    node.destroy_node()

    # 关闭ROS
    ast_ros_middleware.shutdown()
