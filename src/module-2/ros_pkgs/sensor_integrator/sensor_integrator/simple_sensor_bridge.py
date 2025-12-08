import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan # Example message type

class SimpleSensorBridge(Node):
    def __init__(self):
        super().__init__('simple_sensor_bridge')
        self.subscription = self.create_subscription(
            LaserScan,
            '/gazebo_lidar/scan', # Assuming Gazebo publishes on this topic
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(LaserScan, '/integrated_lidar/scan', 10)
        self.get_logger().info('Simple Sensor Bridge node started. Bridging /gazebo_lidar/scan to /integrated_lidar/scan.')

    def listener_callback(self, msg):
        # In a real scenario, you might process or reformat the message here
        self.publisher.publish(msg)
        self.get_logger().debug('Bridged a LaserScan message.')

def main(args=None):
    rclpy.init(args=args)
    simple_sensor_bridge = SimpleSensorBridge()
    rclpy.spin(simple_sensor_bridge)
    simple_sensor_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
