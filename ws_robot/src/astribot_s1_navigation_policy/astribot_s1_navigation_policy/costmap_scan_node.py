"""Adapt valid no-return beams to Nav2's strict range-cutoff projection."""
import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from .protection import costmap_clearing_ranges

class CostmapScanAdapter(Node):
    def __init__(self):
        super().__init__('navigation_costmap_scan_adapter')
        self.declare_parameter('scan_topic','/scan_from_cloud')
        self.declare_parameter('max_marking_range_m',5.5)
        self.output=self.create_publisher(LaserScan,'/navigation_policy/costmap_scan',qos_profile_sensor_data)
        self.create_subscription(LaserScan,self.get_parameter('scan_topic').value,self.scan,qos_profile_sensor_data)

    def scan(self,msg):
        try:
            ranges=costmap_clearing_ranges(msg.ranges,msg.range_max,self.get_parameter('max_marking_range_m').value)
        except ValueError as exc:
            self.get_logger().error(str(exc),throttle_duration_sec=5.)
            return
        converted=copy.deepcopy(msg)
        converted.ranges=ranges
        self.output.publish(converted)

def main():
    rclpy.init();node=CostmapScanAdapter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
