# Chapter 2: AI-to-ROS Bridge (rclpy) - Connecting Intelligence to Robotics

This chapter focuses on integrating Python-based AI agents with the ROS 2 ecosystem. We will explore `rclpy`, the Python client library for ROS 2, to create a bridge that allows our intelligent agents to send commands to, and receive data from, a robotic system.

## 2.1 Introduction to `rclpy`

`rclpy` is the official Python client library for ROS 2. It provides a Pythonic interface to all the core ROS 2 concepts we discussed in Chapter 1 (Nodes, Topics, Services, Actions). Using `rclpy`, developers can leverage the rich Python ecosystem (e.g., machine learning libraries, data processing tools) directly within their ROS 2 applications.

### Why `rclpy` for AI agents?
-   **Ease of Use**: Python's syntax is often more concise and quicker for prototyping.
-   **Rich Ecosystem**: Access to powerful AI/ML libraries like TensorFlow, PyTorch, scikit-learn.
-   **Rapid Development**: Faster iteration cycles compared to C++.

## 2.2 Python Agent → ROS 2 Command Flow

A typical flow for an AI agent interacting with ROS 2 involves:
1.  **Perception Input**: AI agent receives data (e.g., sensor readings, processed features) from ROS 2 topics.
2.  **Decision Making**: AI agent processes the input and makes a decision (e.g., "move forward", "grasp object").
3.  **Action Output**: AI agent sends commands (e.g., motor velocities, manipulation goals) back to ROS 2 topics or services.

## 2.3 Minimal Publish/Subscribe Examples with `rclpy`

Building on the publish-subscribe model from Chapter 1, we will now implement Python nodes that simulate an AI agent sending commands and receiving feedback.

### 2.3.1 Setting Up Your Python Environment

It is highly recommended to use a Python virtual environment to manage dependencies for your AI agents.

```bash
python3 -m venv venv
source venv/bin/activate  # On Windows, use `.\venv\Scripts\activate`
pip install rclpy
```
*Note: Ensure your ROS 2 environment is sourced before activating the virtual environment if you are facing issues.*

### 2.3.2 The "Agent Publisher" Node

This node will simulate an AI agent publishing commands (e.g., simple movement instructions) to a ROS 2 topic.

```python
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
```

### 2.3.3 The "ROS 2 Subscriber" Node

This node will simulate a ROS 2 robot controller component subscribing to commands from the AI agent.

```python
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
```

### 2.3.4 Running the Examples

To run these examples, you will need two separate terminals.

**Terminal 1 (for the AI Agent Publisher):**
1. Navigate to `src/module-1/examples/chapter-2/`.
2. Activate your Python virtual environment (if you created one).
3. Run the publisher node:
   ```bash
   python3 agent_publisher.py
   ```
   You should see the agent publishing random commands.

**Terminal 2 (for the ROS 2 Robot Controller Subscriber):**
1. Navigate to `src/module-1/examples/chapter-2/`.
2. Activate your Python virtual environment (if you created one).
3. Run the subscriber node:
   ```bash
   python3 ros_subscriber.py
   ```
   You should see the robot controller receiving and interpreting the commands.

You can also use ROS 2 CLI tools (e.g., `ros2 topic echo /robot_command`) in a third terminal to inspect the messages being exchanged.
