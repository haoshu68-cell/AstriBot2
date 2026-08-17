#!/usr/bin/env python3
"""测试 logger 集成"""

import time
import threading
import astribot_ros_middleware as ast_ros_middleware
from astribot_ros_middleware.logger.logger import logger, Logger

if __name__ == "__main__":
    # 初始化ROS
    ast_ros_middleware.init()

    print("=" * 60)
    print("测试 1: 不同级别的日志输出")
    print("=" * 60)
    logger.debug("这是DEBUG级别日志")
    logger.info("这是INFO级别日志")
    logger.warning("这是WARNING级别日志")
    logger.error("这是ERROR级别日志")
    logger.critical("这是CRITICAL级别日志")

    print("\n" + "=" * 60)
    print("测试 2: Logger 单例模式验证")
    print("=" * 60)
    logger2 = Logger()
    if logger is logger2:
        logger.info("✓ Logger 单例模式验证通过")
    else:
        logger.warning("✗ Logger 不是单例")

    print("\n" + "=" * 60)
    print("测试 3: 日志级别设置")
    print("=" * 60)
    logger.info("当前使用 INFO 级别")
    logger.debug("这条 DEBUG 日志应该可见")

    logger.set_level("WARNING")
    logger.info("这条 INFO 日志应该不可见")
    logger.warning("这条 WARNING 日志应该可见")

    logger.set_level("DEBUG")
    logger.debug("恢复 DEBUG 级别后，这条日志应该可见")

    print("\n" + "=" * 60)
    print("测试 4: 多线程日志")
    print("=" * 60)

    def thread_logger(thread_id):
        logger.info(f"线程 {thread_id} 开始运行")
        time.sleep(0.1)
        logger.info(f"线程 {thread_id} 完成任务")

    threads = []
    for i in range(3):
        t = threading.Thread(target=thread_logger, args=(i,))
        threads.append(t)
        t.start()

    for t in threads:
        t.join()

    print("\n" + "=" * 60)
    print("测试完成")
    print("=" * 60)

    # 关闭ROS
    ast_ros_middleware.shutdown()
