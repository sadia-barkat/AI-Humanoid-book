# src/module-3/nav2_configs/launch/isaac_nav2_demo.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    # Define launch arguments for flexibility
    arg_namespace = DeclareLaunchArgument(
        'namespace', default_value='/',
        description='Namespace for the Nav2 nodes'
    )
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo/Isaac Sim) clock if true'
    )
    arg_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml'), # Default from nav2_bringup
        description='Full path to the Nav2 parameters file to use'
    )
    
    # Placeholder for Isaac Sim launch (if needed to launch from ROS 2)
    # Typically Isaac Sim is launched separately.

    # Nav2 Bringup launch file
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    return LaunchDescription([
        arg_namespace,
        arg_use_sim_time,
        arg_params_file,

        # Placeholder: Include the actual Isaac ROS VSLAM launch (from Chapter 2)
        # This is needed for localization and mapping for Nav2.
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('isaac_ros_visual_slam'), 'launch', 'isaac_ros_visual_slam_launch.py')),
        #     launch_arguments={'namespace': LaunchConfiguration('namespace'), 'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        # ),

        # Include Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
                'autostart': 'true',
                'robot_model_file': 'path/to/humanoid.urdf', # Placeholder, would come from robot description
                'map_subscribe_transient_local': 'true', # For dynamic maps from VSLAM
            }.items(),
        ),

        # Placeholder: RViz2 to visualize the Nav2 output (map, planned paths, robot pose)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # arguments=['-d', 'path/to/your/nav2_rviz_config.rviz'], # User would need to create this config
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])
