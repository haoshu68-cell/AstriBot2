import os

# ROS 导入
try:
    import rospy
except ImportError:
    rospy = None

try:
    import rclpy
    from rclpy.node import Node
except ImportError:
    rclpy = None


class ROSWrapper:
    """A unified ROS Wrapper that is compatible with ROS1 and ROS2."""

    def __init__(self, node_name="ros_wrapper_node"):
        """Initializes the ROS node, detects ROS1 or ROS2 environment."""
        self.ros_version = self._detect_ros_version()
        if self.ros_version == 1:
            rospy.init_node(node_name, anonymous=True)
            print("ROS1 environment initialized successfully.")
        elif self.ros_version == 2:
            rclpy.init()
            self.node = Node(node_name)
            print("ROS2 environment initialized successfully.")
        else:
            raise EnvironmentError("No valid ROS environment detected. Set ROS_DISTRO variable.")

    @staticmethod
    def _detect_ros_version():
        """Detects the ROS version based on the ROS_DISTRO environment variable."""
        ros_distro = os.getenv("ROS_DISTRO")
        if ros_distro and any(distro in ros_distro for distro in ["noetic", "melodic"]):
            return 1
        elif ros_distro and any(distro in ros_distro for distro in ["foxy", "galactic", "humble"]):
            return 2
        return None

    def create_publisher(self, topic_name, msg_type, queue_size=10):
        """Creates a publisher for any message type."""
        if self.ros_version == 1:
            return rospy.Publisher(topic_name, msg_type, queue_size=queue_size)
        elif self.ros_version == 2:
            return self.node.create_publisher(msg_type, topic_name, queue_size)

    def create_subscriber(self, topic_name, msg_type, callback):
        """Creates a subscriber for any message type."""
        if self.ros_version == 1:
            return rospy.Subscriber(topic_name, msg_type, callback)
        elif self.ros_version == 2:
            return self.node.create_subscription(msg_type, topic_name, callback, 10)

    def create_service(self, service_name, srv_type, callback):
        """Creates a service (server) for any service type."""
        if self.ros_version == 1:
            return rospy.Service(service_name, srv_type, callback)
        elif self.ros_version == 2:
            return self.node.create_service(srv_type, service_name, callback)

    def create_client(self, service_name, srv_type):
        """Creates a client for any service type."""
        if self.ros_version == 1:
            rospy.wait_for_service(service_name)
            return rospy.ServiceProxy(service_name, srv_type)
        elif self.ros_version == 2:
            client = self.node.create_client(srv_type, service_name)
            while not client.wait_for_service(timeout_sec=1.0):
                print(f"Waiting for service '{service_name}'...")
            return client

    def call_service(self, client, request):
        """Calls a service (client) for any service type."""
        if self.ros_version == 1:
            return client(request)
        elif self.ros_version == 2:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            return future.result()

    def spin(self):
        """Executes spin for ROS1 and ROS2."""
        if self.ros_version == 1:
            rospy.spin()
        elif self.ros_version == 2:
            rclpy.spin(self.node)

    def shutdown(self):
        """Shuts down the ROS node."""
        if self.ros_version == 1:
            rospy.signal_shutdown("Shutting down ROSWrapper.")
        elif self.ros_version == 2:
            self.node.destroy_node()
            rclpy.shutdown()




def main():
    from astribot_sdk.core.common.ros_wrapper import ROSWrapper
    from std_msgs.msg import String
    from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

    """Main function demonstrating ROSWrapper usage for publisher, subscriber, and service."""

    ros_wrapper = ROSWrapper(node_name="ros_wrapper_demo")

    # --- 发布消息示例 ---
    publisher = ros_wrapper.create_publisher("/example_topic", String)

    def callback(msg):
        print(f"Received message: {msg.data}")

    # --- 订阅消息示例 ---
    ros_wrapper.create_subscriber("/example_topic", String, callback)

    # --- 服务端示例 (Server) ---
    def handle_service(request):
        print("Received a service request.")
        response = TriggerResponse(success=True, message="Service handled successfully.")
        return response

    ros_wrapper.create_service("/example_service", Trigger, handle_service)

    # --- 服务客户端示例 (Client) ---
    client = ros_wrapper.create_client("/example_service", Trigger)

    def call_service_example():
        request = TriggerRequest()
        response = ros_wrapper.call_service(client, request)
        print(f"Service response: {response.message}")

    call_service_example()

    print("Publishing messages to '/example_topic'...")

    try:
        if ros_wrapper.ros_version == 1:
            rate = rospy.Rate(1)
            while not rospy.is_shutdown():
                msg = String(data="Hello from ROS1 and ROS2 unified!")
                publisher.publish(msg)
                rate.sleep()
        else:
            from time import sleep
            count = 0
            while rclpy.ok():
                msg = String()
                msg.data = f"Hello from ROS2! Count: {count}"
                publisher.publish(msg)
                print(f"Published: {msg.data}")
                sleep(1)
                count += 1
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        ros_wrapper.shutdown()


if __name__ == "__main__":
    main()
