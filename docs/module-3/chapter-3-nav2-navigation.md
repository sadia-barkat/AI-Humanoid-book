# Chapter 3: Nav2 Navigation - Autonomous Movement in Dynamic Worlds

This chapter integrates the powerful Nav2 (Navigation2) stack with an Isaac Sim robot, enabling advanced path planning and autonomous execution of movement tasks in dynamic simulated environments. We will configure Nav2 for a bipedal robot, allowing it to intelligently navigate and avoid obstacles.

## 3.1 Introduction to Nav2

Nav2 is the current ROS 2 navigation stack, providing a complete framework for mobile robot navigation. It replaces the original ROS Navigation Stack and offers improved performance, flexibility, and modularity, making it suitable for complex humanoid navigation challenges.

### Key Components of Nav2:
-   **State Estimator**: Integrates sensor data for accurate robot pose estimation (e.g., from VSLAM).
-   **Global Planner**: Generates a long-term path from start to goal.
-   **Local Planner**: Generates velocity commands to follow the global path and avoid local obstacles.
-   **Recovery Behaviors**: Handles situations where the robot gets stuck or encounters unexpected obstacles.

## 3.2 Path Planning for Bipedal Robots

Nav2 is highly configurable and can be adapted for various robot types. For bipedal robots, specific considerations apply:
-   **Footstep Planning**: Unlike wheeled robots, bipedal robots require discrete footstep planning for stable locomotion. This often involves specialized plugins or external planners integrated with Nav2.
-   **Balance Control**: Nav2 typically outputs velocity commands, which a humanoid's whole-body controller translates into stable foot placements and joint trajectories.
-   **Costmaps**: Configuring costmaps to represent traversable terrain and obstacles, including dynamic obstacles.

## 3.3 Executing Movement in Dynamic Spaces

Integrating Nav2 with Isaac Sim allows us to test navigation algorithms in complex, dynamic environments.
-   **Simulation Integration**: Connect Nav2's output (e.g., cmd_vel) to the robot's controller in Isaac Sim.
-   **Dynamic Obstacles**: Introduce moving obstacles in Isaac Sim to test Nav2's reactive planning capabilities.
-   **Goal Management**: Send navigation goals via ROS 2 actions to the Nav2 stack.

## 3.4 Example: Humanoid Robot Navigation with Nav2 in Isaac Sim

In this section, we will set up Nav2 for our humanoid robot within Isaac Sim, enabling it to plan and execute paths while avoiding dynamic obstacles.

### 3.4.1 Nav2 Configuration Files for a Bipedal Robot

We will use the placeholder Nav2 configuration file created at `src/module-3/nav2_configs/nav2_params.yaml`. This file defines the core parameters for Nav2 components.

### 3.4.2 Integrating Nav2 with Isaac Sim (Launch File)

```python
# src/module-3/nav2_configs/launch/isaac_nav2_demo.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
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
        default_value=os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file to use'
    )
    
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    return LaunchDescription([
        arg_namespace,
        arg_use_sim_time,
        arg_params_file,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
                'autostart': 'true',
                'robot_model_file': 'path/to/humanoid.urdf',
                'map_subscribe_transient_local': 'true',
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])
```

### 3.4.3 Commanding Robot Navigation

To command your robot to navigate using Nav2:

1.  Ensure Isaac Sim is running with your robot and a suitable environment, and both VSLAM and Nav2 are successfully launched (e.g., using the launch file above).
2.  **Set an Initial Pose**: In RViz2, use the "2D Pose Estimate" tool to set the robot's initial pose on the map.
3.  **Send a Navigation Goal**: Use the "2D Goal Pose" tool in RViz2 to select a target location and orientation for your robot. Nav2 will then plan a path and attempt to move the robot to that goal.
4.  **Monitor Progress**: Observe the robot's movement in Isaac Sim and RViz2. You can use ROS 2 CLI tools (e.g., `ros2 topic echo /cmd_vel` or `ros2 topic echo /amcl_pose`) to monitor navigation commands and the robot's estimated pose.
