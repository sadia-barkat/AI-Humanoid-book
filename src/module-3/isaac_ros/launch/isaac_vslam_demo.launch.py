# src/module-3/isaac_ros/launch/isaac_vslam_demo.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define launch arguments for flexibility
    arg_namespace = DeclareLaunchArgument(
        'namespace', default_value='/',
        description='Namespace for the VSLAM nodes'
    )
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo/Isaac Sim) clock if true'
    )

    # Path to the Isaac ROS VSLAM launch file (example)
    # This path would depend on your Isaac ROS installation and specific VSLAM package
    isaac_ros_vslam_launch_dir = get_package_share_directory('isaac_ros_visual_slam')
    isaac_ros_vslam_launch_file = os.path.join(isaac_ros_vslam_launch_dir, 'launch', 'isaac_ros_visual_slam_launch.py')

    # Placeholder for Isaac Sim launch (if needed to launch from ROS 2)
    # Typically Isaac Sim is launched separately, and ROS 2 nodes connect to it.
    # If this launch file should also launch Isaac Sim, it would involve:
    # 1. An IncludeLaunchDescription for an Isaac Sim bridge package (e.g., isaac_ros_one_off_bridge)
    # 2. Or a simple execute process for the isaac_sim.sh script.

    return LaunchDescription([
        arg_namespace,
        arg_use_sim_time,

        # Placeholder: Include the actual Isaac ROS VSLAM launch file
        # This assumes the VSLAM package is installed and configured in the ROS 2 workspace.
        # Ensure the Isaac Sim camera data is being published to ROS 2 topics that VSLAM expects.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(isaac_ros_vslam_launch_file),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Add other VSLAM-specific arguments here
                'input_base_frame': 'camera_link',
                'input_odom_frame': 'odom',
                'output_frame': 'odom',
                'enable_slam': 'true',
                'enable_localization': 'true',
            }.items(),
        ),

        # Placeholder: RViz2 to visualize the VSLAM output (map, pose)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # arguments=['-d', 'path/to/your/vslam_rviz_config.rviz'], # User would need to create this config
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])
