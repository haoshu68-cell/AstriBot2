#!/usr/bin/env python3
"""服务端和客户端示例：在同一个节点中创建 service 和 client"""

import threading
import time
import astribot_ros_middleware as ast_ros_middleware
from std_srvs.srv import SetBool


def handle_set_bool(request, response):
    """处理SetBool服务请求"""
    print(f"收到服务请求: data={request.data}")

    # 设置响应
    response.success = True
    response.message = f"接收到的值: {request.data}"

    return response


def call_service_task(client, logger, node):
    """在后台线程调用服务"""
    time.sleep(1)  # 等待服务就绪

    try:
        # 等待服务可用
        logger.info("等待服务可用...")
        if not client.wait_for_service(timeout_sec=5.0):
            logger.error("服务不可用")
            return
        logger.info("服务已就绪")

        # 第一次调用
        request = SetBool.Request()
        request.data = True
        logger.info(f"发送服务请求: data={request.data}")
        response = client.call(request, timeout_sec=5.0)
        if response:
            logger.info(
                f"收到服务响应: success={response.success}, message={response.message}"
            )
        else:
            logger.error("第一次调用失败")

        # 第二次调用
        time.sleep(1)
        request = SetBool.Request()
        request.data = False
        logger.info(f"发送服务请求: data={request.data}")
        response = client.call(request, timeout_sec=5.0)
        if response:
            logger.info(
                f"收到服务响应: success={response.success}, message={response.message}"
            )
        else:
            logger.error(f"第二次调用失败，response={response}")

        logger.info("客户端调用完成，准备退出")
        # 调用完成后停止spin
        time.sleep(0.5)
        ast_ros_middleware.shutdown()

    except Exception as e:
        logger.error(f"服务调用失败: {e}")


if __name__ == "__main__":
    # 初始化ROS
    ast_ros_middleware.init()

    # 创建节点
    node = ast_ros_middleware.Node("service_example")
    logger = node.get_logger()

    # 创建服务端
    server = node.create_service(SetBool, "set_bool_service", handle_set_bool)
    logger.info("服务端已创建，服务名: /set_bool_service")

    # 创建服务客户端
    client = node.create_client(SetBool, "set_bool_service")
    logger.info("服务客户端已创建")

    # 启动客户端调用线程
    client_thread = threading.Thread(
        target=call_service_task, args=(client, logger, node), daemon=True
    )
    client_thread.start()

    try:
        # 主线程spin处理服务请求
        logger.info("开始spin，处理服务请求...")
        node.spin()
        logger.info("示例完成")
    except (KeyboardInterrupt, SystemExit):
        logger.info("程序被中断")
    except Exception as e:
        logger.error(f"程序异常: {e}")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            ast_ros_middleware.shutdown()
        except Exception:
            pass
