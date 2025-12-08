# src/module-1/examples/chapter-2/ros_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('robot_controller_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot_command',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Robot Controller Subscriber node started. Waiting for commands.')

    def listener_callback(self, msg):
        command = msg.data
        if "AI_AGENT_COMMAND:" in command:
            actual_command = command.replace("AI_AGENT_COMMAND: ", "").strip()
            self.get_logger().info(f'Received AI command: "{actual_command}"')
            # Here you would typically execute the command on the robot
            if actual_command == "move_forward":
                self.get_logger().info("Executing: Moving robot forward...")
            elif actual_command == "turn_left":
                self.get_logger().info("Executing: Turning robot left...")
            elif actual_command == "turn_right":
                self.get_logger().info("Executing: Turning robot right...")
            elif actual_command == "stop":
                self.get_logger().info("Executing: Stopping robot...")
            else:
                self.get_logger().info(f"Unknown command: {actual_command}")
        else:
            self.get_logger().info(f'Received unexpected message: "{command}"')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
