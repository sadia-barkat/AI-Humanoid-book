# Chapter 1: ROS 2 Basics - The Robotic Nervous System

This chapter introduces the fundamental concepts of ROS 2, which serves as the "nervous system" for modern robotics applications. We will cover the core communication mechanisms that allow different parts of a robot's software to interact seamlessly.

## 1.1 What is ROS 2?

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of building complex robot applications across a wide variety of robotic platforms.

### Key Features:
-   **Distributed**: Designed for multi-robot systems and complex deployments.
-   **Real-time Capable**: Built with quality of service (QoS) settings to support real-time constraints.
-   **Secure**: Includes security features for authentication and encryption.
-   **Multi-Platform**: Supports Linux, Windows, and macOS.

## 1.2 Core ROS 2 Concepts

At the heart of ROS 2 are several key concepts that facilitate inter-process communication:

### 1.2.1 Nodes
Nodes are individual processes that perform computation. In a robotic system, each node is typically responsible for a single module, such as a sensor driver, a motor controller, or a navigation algorithm.

-   **Example**: A node might read data from a camera, another might control a robotic arm, and a third might process commands from a human operator.

### 1.2.2 Topics
Topics are named buses over which nodes exchange messages asynchronously. This is a publish-subscribe communication model, where publishers send messages to a topic, and subscribers receive messages from that topic.

-   **Analogy**: Think of it like a radio station. Publishers broadcast information on a specific frequency (topic), and anyone tuned into that frequency (subscribers) can receive the information.
-   **Use Case**: Streaming sensor data (e.g., camera images, LiDAR scans), robot joint states, or telemetry.

### 1.2.3 Services
Services are a request-response communication model. A client sends a request to a service, and the service performs an action and sends back a response. This is used for calls that require a definite response and are not typically high-frequency streaming data.

-   **Analogy**: Like making a phone call to request information or a specific action. You expect a direct response.
-   **Use Case**: Triggering a specific action (e.g., "take a picture", "move arm to home position"), querying a map server.

### 1.2.4 Actions
Actions are an extension of services that provide long-running, cancellable, and preemptable tasks. They offer feedback during execution and allow for goals to be canceled or modified.

-   **Analogy**: Similar to ordering a pizza. You send an order (goal), get updates on its status (feedback), and can cancel it if needed (preempt).
-   **Use Case**: Complex navigation tasks (e.g., "go to kitchen"), manipulating objects (e.g., "pick up block").

## 1.3 How Humanoid Robots Use ROS 2 for Communication

In humanoid robotics, ROS 2 enables a modular and distributed approach to control. Different nodes can handle:

-   **Perception**: Processing camera, LiDAR, and IMU data to understand the environment.
-   **Motion Control**: Sending commands to motors for walking, balancing, or arm movements.
-   **Planning**: Generating trajectories for navigation or manipulation.
-   **Human-Robot Interaction**: Processing voice commands or gestures.

By using topics, services, and actions, these different functionalities can communicate and coordinate to achieve complex behaviors.

## 1.4 Examples: Talker and Listener (ROS 2 Python)

In this section, we will create simple ROS 2 Python nodes to demonstrate the publish-subscribe mechanism using `rclpy` (the Python client library for ROS 2).

### 1.4.1 Setting Up Your Workspace

Ensure you have a ROS 2 workspace set up. If not, please refer to the official ROS 2 documentation for setting up a Humble workspace.

### 1.4.2 The "Talker" Node (Publisher)

This node will continuously publish "Hello World" messages to a topic.

```python
# src/module-1/examples/chapter-1/talker.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 1.4.3 The "Listener" Node (Subscriber)

This node will subscribe to the topic created by the talker and print any messages it receives.

```python
# src/module-1/examples/chapter-1/listener.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 1.4.4 Running the Examples

To run these examples, you will need two separate terminals.

**Terminal 1 (for the Talker node):**
1. Navigate to your workspace (if necessary, source your ROS 2 environment).
2. Run the talker node:
   ```bash
   ros2 run YOUR_PACKAGE_NAME talker_node_executable # Replace with your package and executable name
   ```
   *Note: If you have not created a ROS 2 package for these examples, you can navigate directly to the `src/module-1/examples/chapter-1/` directory and run the Python script using `python3 talker.py`. However, for a proper ROS 2 setup, it is recommended to create a package.*

**Terminal 2 (for the Listener node):**
1. Navigate to your workspace (if necessary, source your ROS 2 environment).
2. Run the listener node:
   ```bash
   ros2 run YOUR_PACKAGE_NAME listener_node_executable # Replace with your package and executable name
   ```
   *Or, if running directly:*
   ```bash
   python3 listener.py
   ```
You should see the "talker" publishing messages and the "listener" receiving them.

### 1.4.5 Inspecting with ROS 2 CLI Tools

ROS 2 provides powerful command-line interface (CLI) tools to inspect and interact with your ROS graph.

While the talker and listener nodes are running:

1.  **List active nodes**:
    ```bash
    ros2 node list
    ```
    You should see `/minimal_publisher` and `/minimal_subscriber`.

2.  **List active topics**:
    ```bash
    ros2 topic list
    ```
    You should see `/topic`.

3.  **Echo topic messages**:
    ```bash
    ros2 topic echo /topic
    ```
    This command will print all messages being published on `/topic`, allowing you to observe the communication directly.

4.  **Show node information**:
    ```bash
    ros2 node info /minimal_publisher
    ros2 node info /minimal_subscriber
    ```
    This provides details about each node, including the topics they publish/subscribe to.

These CLI tools are invaluable for debugging and understanding the flow of data in your ROS 2 system.
