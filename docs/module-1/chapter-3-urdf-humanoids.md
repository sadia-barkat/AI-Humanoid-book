# Chapter 3: URDF for Humanoids - Describing Your Robot

This chapter introduces the Unified Robot Description Format (URDF), an XML format used in ROS to describe the physical characteristics of a robot. For humanoid robots, URDF is crucial for defining their complex structure, enabling simulation, visualization, and motion planning.

## 3.1 Introduction to URDF

URDF is a standard way to describe a robot's kinematic and dynamic properties. It's an XML file that specifies:
-   **Links**: The rigid bodies of the robot (e.g., torso, upper arm, forearm).
-   **Joints**: The connections between links, defining their type (e.g., revolute, prismatic) and motion limits.
-   **Sensors**: Properties of sensors attached to the robot.
-   **Materials**: Visual properties like color.
-   **Inertial Properties**: Mass and inertia tensors for physics simulation.

### Why URDF is essential for humanoid robots:
-   **Visualization**: Displaying the robot's model in tools like RViz2.
-   **Simulation**: Providing physical properties for simulators like Gazebo.
-   **Motion Planning**: Defining the robot's degrees of freedom and collision geometry.

## 3.2 Defining Links, Joints, and Sensors

A URDF file is structured around `<link>` and `<joint>` tags.

### 3.2.1 Links
Links are the fundamental rigid bodies. Each link has:
-   **Visual**: Describes how the link looks (e.g., mesh file, geometry, color).
-   **Collision**: Defines the collision geometry, often a simplified version of the visual geometry.
-   **Inertial**: Specifies mass, center of mass, and inertia matrix for physics simulation.

### 3.2.2 Joints
Joints connect two links: a `parent` link and a `child` link. Key attributes include:
-   **Type**: `revolute`, `continuous`, `prismatic`, `fixed`, `floating`, `planar`.
-   **Origin**: The pose of the child link relative to the parent.
-   **Axis**: The axis of rotation or translation for the joint.
-   **Limit**: For `revolute` and `prismatic` joints, defines the upper and lower limits of motion.

### 3.2.3 Sensors (through Gazebo/ROS 2 integration)
While URDF itself doesn't directly define sensors with their full functionality, it can attach placeholders or visual representations. Full sensor simulation is typically handled by simulator plugins (e.g., Gazebo plugins) that are referenced in an accompanying SDF (Simulation Description Format) or a Gazebo-specific extension to URDF.

## 3.3 How URDF Supports Simulation and Control

URDF models are parsed by ROS 2 components and simulators to understand the robot's physical characteristics.

-   **Simulation**: Simulators like Gazebo use the inertial and collision properties to accurately model robot behavior in a physics engine.
-   **Control**: Controllers read joint limits and types from the URDF to send appropriate commands to the robot's motors.
-   **State Estimation**: The kinematic chain defined by URDF helps in calculating the robot's pose from joint encoder data.

## 3.4 Example: Simple Humanoid URDF

In this section, we will create a simplified URDF for a humanoid robot, focusing on its basic structure with a torso, head, and two arms.

### 3.4.1 Creating the `humanoid.urdf`

```xml
<?xml version="1.0"?>
<robot name="humanoid">

  <!-- Base Link (Torso) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head Link and Joint -->
  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Right Arm Link and Joint -->
  <joint name="torso_to_right_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 0.15 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_shoulder_to_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0" effort="100" velocity="10"/>
  </joint>

  <link name="right_forearm">
    <visual>
      <geometry>
        <cylinder radius="0.025" length="0.15"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.025" length="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Left Arm Link and Joint (Symmetric to Right) -->
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 -0.15 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_shoulder_to_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0" effort="100" velocity="10"/>
  </joint>

  <link name="left_forearm">
    <visual>
      <geometry>
        <cylinder radius="0.025" length="0.15"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.025" length="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Materials for consistent coloring -->
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>
  <material name="red">
    <color rgba="0.8 0 0 1"/>
  </material>
  <material name="green">
    <color rgba="0 0.8 0 1"/>
  </material>

</robot>
```

### 3.4.2 Validating the URDF

We will use the `check_urdf` tool to ensure our URDF file is syntactically correct and well-formed.

To validate your `humanoid.urdf` file, ensure your ROS 2 environment is sourced, and then run the `check_urdf` command:

```bash
check_urdf src/module-1/urdf/humanoid.urdf
```

A successful validation will show output indicating the robot's structure and no errors. If there are errors, `check_urdf` will provide details to help you correct them.

### 3.4.3 Visualizing the URDF in RViz2

Once validated, we can visualize our humanoid robot in RViz2, a 3D visualizer for ROS 2.

To visualize your humanoid robot in RViz2:

1.  Ensure your ROS 2 environment is sourced.
2.  Launch the display file:
    ```bash
    ros2 launch src/module-1/urdf/display_humanoid.launch.py
    ```
3.  RViz2 should open. If you don't see the robot, you may need to:
    *   Set the `Fixed Frame` in RViz2 to `torso`.
    *   Add a `RobotModel` display type and ensure its `Description Topic` is set to `robot_description`.

This will allow you to see the 3D model of your humanoid robot, verifying your URDF definition.
