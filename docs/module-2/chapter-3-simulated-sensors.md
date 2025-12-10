# Chapter 3: Simulated Sensors for Humanoids - Perceiving the Digital World

This chapter delves into the crucial aspect of robot perception in simulation: creating and integrating simulated sensor data. We will explore how to set up virtual LiDAR, depth cameras, and IMUs within Gazebo and/or Unity, and feed their data into a ROS 2 pipeline, enabling the development of advanced perception algorithms without physical hardware.

## 3.1 Importance of Simulated Sensors

Simulated sensors are vital for:
-   **Early Development**: Test perception algorithms before robot hardware is available.
-   **Safe Testing**: Experiment with dangerous scenarios without risk to physical robots or humans.
-   **Reproducibility**: Easily recreate specific sensor readings for debugging and analysis.
-   **Scalability**: Deploy multiple virtual robots with various sensor configurations.

## 3.2 LiDAR, Depth Camera, and IMU Simulation

Different sensors provide different types of environmental data, each with its own simulation challenges and configurations.

### 3.2.1 LiDAR (Light Detection and Ranging)
LiDAR sensors provide 2D or 3D point cloud data, crucial for mapping, navigation, and object detection.
-   **Gazebo**: Uses specialized plugins (e.g., `libgazebo_ros_ray_sensor.so` for `ros_gz_sim`) to simulate laser scans from ray tracing.
-   **Unity**: Can simulate raycasts for LiDAR-like data or use depth cameras with post-processing.

### 3.2.2 Depth Camera
Depth cameras (e.g., Intel RealSense, Microsoft Kinect) provide per-pixel depth information, useful for 3D reconstruction and object manipulation.
-   **Gazebo**: Uses camera plugins that can render depth images.
-   **Unity**: Native depth texture support, often combined with rendering techniques.

### 3.2.3 IMU (Inertial Measurement Unit)
IMUs provide data on orientation, angular velocity, and linear acceleration, essential for robot state estimation and balance.
-   **Gazebo**: Uses plugins (e.g., `libgazebo_ros_imu_sensor.so`) that extract data from the physics engine.
-   **Unity**: Can derive this data from the rigid body physics simulation.

## 3.3 Feeding Sensor Data into the ROS 2 Pipeline

Integrating simulated sensor data into ROS 2 typically involves:
-   **Gazebo-ROS 2 Bridge**: Tools like `ros_gz_bridge` (for `ros_gz_sim`) are used to publish sensor data from Gazebo topics to ROS 2 topics.
-   **Unity-ROS 2 Bridge**: Custom Unity packages (e.g., `ROS-Unity-Core`) provide a bridge for C# scripts to publish data to ROS 2.
-   **ROS 2 Messages**: Simulated data is published using standard ROS 2 message types (e.g., `sensor_msgs/msg/LaserScan`, `sensor_msgs/msg/Image` for depth, `sensor_msgs/msg/Imu`).

## 3.4 Example: Simulated LiDAR and ROS 2 Integration

In this section, we will add a simulated LiDAR sensor to our humanoid model in Gazebo and bridge its data to ROS 2, then visualize it in RViz2.

### 3.4.1 Creating/Adapting a Simulated LiDAR Plugin

We will use the placeholder Gazebo SDF model `src/module-2/gazebo/models/humanoid_sensors/model.sdf` that includes a LiDAR plugin. This plugin enables Gazebo to simulate laser scans.

### 3.4.2 Integrating the Sensor into the Humanoid Model

To integrate the simulated LiDAR into our humanoid:

1.  **Modify `humanoid.urdf`**: Add the LiDAR model to our `humanoid.urdf` by including a new link and joint to attach it to the robot, and then referencing the `humanoid_lidar` model.
    ```xml
    <!-- Example: Attach LiDAR to the head -->
    <joint name="head_to_lidar" type="fixed">
      <parent link="head"/>
      <child link="lidar_link"/>
      <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
    </joint>
    <link name="lidar_link">
      <inertial>
        <mass>0.1</mass>
        <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
      </inertial>
      <visual name="visual">
        <geometry>
          <cylinder radius="0.02" length="0.03"/>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder radius="0.02" length="0.03"/>
        </geometry>
      </collision>
    </link>

    <!-- Include the LiDAR model in the Gazebo world file, attached to the humanoid -->
    ```
    *Note: For simplicity, we are showing an example of how you might include it directly in the URDF, or you can add the LiDAR as a separate model in the Gazebo world file and attach it using `<joint>` elements in the world file itself, or use Xacro for more complex URDFs.*

### 3.4.3 Bridging Data to ROS 2

To bridge the simulated LiDAR data from Gazebo to ROS 2:

1.  **Build `ros_gz_bridge`**: Ensure you have `ros_gz_bridge` installed and built in your ROS 2 workspace.
2.  **Launch Bridge**: Use the `ros_gz_bridge` to create a bridge between the Gazebo topic (`/lidar/scan`) and a ROS 2 topic (`/integrated_lidar/scan`).
    ```bash
    ros2 run ros_gz_bridge lidarsub_bridge --ros-args -r /integrated_lidar/scan:=/lidar/scan
    ```
    *Note: The exact remapping might vary based on your Gazebo plugin's output topic.*
3.  **Use `sensor_integrator` node**: Alternatively, our `sensor_integrator` ROS 2 package provides a node that can subscribe to the raw Gazebo topic and re-publish it on a new ROS 2 topic, potentially with processing.
    ```bash
    # After building your workspace (colcon build) and sourcing it
    ros2 run sensor_integrator sensor_bridge_node
    ```

### 3.4.4 Visualizing Sensor Data in RViz2

To visualize the integrated LiDAR data in RViz2:

1.  Ensure ROS 2 environment is sourced and the LiDAR data is being published on a ROS 2 topic (e.g., `/integrated_lidar/scan`).
2.  Launch RViz2:
    ```bash
    rviz2
    ```
3.  In RViz2:
    *   Set the `Fixed Frame` to a frame connected to your robot (e.g., `base_link` or `lidar_link`).
    *   Add a new display by clicking "Add".
    *   Select "LaserScan" under the `sensor_msgs` plugin.
    *   Set the `Topic` property of the LaserScan display to the ROS 2 topic publishing your LiDAR data (e.g., `/integrated_lidar/scan`).
    *   You should now see the simulated LiDAR scans as a cloud of points in the RViz2 display.
