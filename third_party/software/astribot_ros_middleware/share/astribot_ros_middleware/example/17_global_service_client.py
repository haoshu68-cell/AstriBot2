#!/usr/bin/env python3
"""使用全局API的服务客户端示例"""

import threading
import time
import astribot_ros_middleware as ast_ros_middleware
from std_srvs.srv import SetBool


def shutdown_timer():
    time.sleep(3)
    print("3秒已到，程序即将退出...")
    ast_ros_middleware.shutdown()


def main():
    ast_ros_middleware.init(node_name="global_service_client")

    timer_thread = threading.Thread(target=shutdown_timer, daemon=True)
    timer_thread.start()

    client = ast_ros_middleware.create_client(SetBool, "global_set_bool")
    print("全局服务客户端已创建，服务名: /global_set_bool")

    print("等待服务可用...")
    if not client.wait_for_service(timeout_sec=5.0):
        print("服务不可用，退出")
        ast_ros_middleware.shutdown()
        return

    request = SetBool.Request()
    request.data = True

    print(f"发送请求: data={request.data}")
    response = client.call(request)

    if response is not None:
        print(f"收到响应: success={response.success}, message={response.message}")
    else:
        print("服务调用失败，未收到响应")

    ast_ros_middleware.shutdown()


if __name__ == "__main__":
    main()
