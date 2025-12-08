# src/module-3/nav2_configs/tests/test_nav2_execution.py
import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped # For sending navigation goals
from nav_msgs.msg import Odometry # For checking robot pose
import time

# This test is a placeholder and requires a running Isaac Sim + Nav2 environment
# to be fully functional. It's designed to illustrate the concept.

class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10) # Nav2 goal topic
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_pose = None
        self.get_logger().info('Nav2 Goal Sender node started.')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def send_goal(self, x, y, yaw_degrees):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        
        # Convert yaw from degrees to quaternion
        yaw_radians = yaw_degrees * (3.14159265359 / 180.0)
        goal_pose.pose.orientation.z = sin(yaw_radians / 2.0)
        goal_pose.pose.orientation.w = cos(yaw_radians / 2.0)
        
        self.publisher.publish(goal_pose)
        self.get_logger().info(f'Published Nav2 goal: x={x}, y={y}, yaw={yaw_degrees} degrees.')

@pytest.mark.skip(reason="Requires a running Isaac Sim + Nav2 environment for full functional test.")
def test_nav2_path_planning_and_execution():
    """
    Placeholder test to verify Nav2 path planning and goal execution in Isaac Sim.
    This test assumes Isaac Sim is running with a robot, VSLAM is active, and Nav2 is configured.
    """
    print("Running placeholder test for Nav2 path planning and execution...")

    # In a real test, you would:
    # 1. Launch Isaac Sim with a robot and configured environment.
    # 2. Launch Isaac ROS VSLAM for localization and mapping.
    # 3. Launch Nav2 stack.
    # 4. Create a goal sender node (like Nav2GoalSender) and send a goal.
    # 5. Monitor robot's `/odom` topic to verify it reaches the goal.
    # 6. Check Nav2 logs for path planning success/failure.

    rclpy.init(args=None)
    goal_sender = Nav2GoalSender()
    
    # Simulate sending a goal
    goal_x, goal_y, goal_yaw = 2.0, 1.0, 90.0
    goal_sender.send_goal(goal_x, goal_y, goal_yaw)

    # Wait for a reasonable time for the robot to reach the goal (placeholder)
    time.sleep(15) 

    # In a real test, you'd check `goal_sender.current_pose` against `goal_x`, `goal_y`
    # and compare the current orientation with `goal_yaw`.
    
    rclpy.shutdown()

    # Assert that the robot has approximately reached the goal (placeholder logic)
    # For a placeholder, we'll just fail as it requires real execution.
    assert False, "Nav2 path planning and execution test is a placeholder and requires manual verification or a configured Isaac Sim/Nav2 environment."
