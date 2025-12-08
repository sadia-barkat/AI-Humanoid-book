# src/module-1/examples/chapter-2/agent_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class AgentPublisher(Node):

    def __init__(self):
        super().__init__('agent_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_command', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.commands = ["move_forward", "turn_left", "turn_right", "stop"]
        self.get_logger().info('Agent Publisher node started. Publishing random commands.')

    def timer_callback(self):
        command = random.choice(self.commands)
        msg = String()
        msg.data = f'AI_AGENT_COMMAND: {command}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing command: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    agent_publisher = AgentPublisher()

    rclpy.spin(agent_publisher)

    agent_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
