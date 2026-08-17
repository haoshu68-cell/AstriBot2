#!/usr/bin/env python3
"""使用全局API的服务端示例"""

import threading
import time
import astribot_ros_middleware as ast_ros_middleware
from std_srvs.srv import SetBool


def handle_service(request, response):
    print(f"收到服务请求: data={request.data}")
    response.success = True
    response.message = f"已处理请求: {request.data}"
    return response


def shutdown_timer():
    time.sleep(10)
    print("3秒已到，程序即将退出...")
    ast_ros_middleware.shutdown()


def main():
    ast_ros_middleware.init(node_name="global_service_server")

    timer_thread = threading.Thread(target=shutdown_timer, daemon=True)
    timer_thread.start()

    ast_ros_middleware.create_service(SetBool, "global_set_bool", handle_service)
    print("全局服务端已创建，服务名: /global_set_bool")

    print("等待服务请求...")
    ast_ros_middleware.spin()


if __name__ == "__main__":
    main()
