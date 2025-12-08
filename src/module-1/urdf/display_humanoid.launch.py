# src/module-1/urdf/display_humanoid.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the URDF file path
    urdf_file_path = os.path.join(
        os.path.dirname(__file__),
        'humanoid.urdf'
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file_path).read()}]
    )

    # RViz2 node
    rviz_config_dir = os.path.join(get_package_share_directory('rviz_common'), 'rviz')
    # A simple rviz config to display the robot model. Users can create their own.
    # For now, we don't provide a specific config file for the humanoid model.
    # Users will need to add RobotModel display manually.
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('urdf_tutorial'), 'rviz', 'urdf.rviz')], # Example default config
        # Use a more generic rviz config or instruct user to set up.
        # It's difficult to create a universal rviz config for an arbitrary URDF.
        # For simplicity, we can let rviz2 launch and user add RobotModel display.
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])
