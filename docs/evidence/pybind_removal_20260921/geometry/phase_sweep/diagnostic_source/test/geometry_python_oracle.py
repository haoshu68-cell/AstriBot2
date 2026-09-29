#!/usr/bin/env python3
"""Test-only driver for the retained Python comparison class; never installed."""
import rclpy
from astribot_s1_robot_geometry.node import GeometryNode


def main():
    rclpy.init()
    node = GeometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
