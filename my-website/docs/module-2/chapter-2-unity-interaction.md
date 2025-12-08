# Chapter 2: High-Fidelity Interaction in Unity - Building Dynamic Digital Twins

This chapter explores Unity, a versatile real-time 3D development platform, for creating high-fidelity simulations of humanoid robots and interactive environments. We will focus on rendering realistic humanoid motion and setting up basic human-robot interaction scenes, leveraging Unity's robust graphics and physics capabilities.

## 2.1 Introduction to Unity

Unity is a powerful cross-platform game engine and real-time 3D development platform. Beyond game development, it's widely adopted for simulations, architectural visualization, and interactive experiences due to its advanced rendering, physics, and scripting features.

### Key Features:
-   **High-Quality Rendering**: Advanced visual effects, lighting, and materials for realistic scenes.
-   **Integrated Physics**: Built-in physics engine (NVIDIA PhysX) for realistic object interactions.
-   **Scripting (C#)**: Develop custom behaviors and logic using C#.
-   **Asset Store**: A vast marketplace for 3D models, textures, and tools.
-   **ROS Integration**: Community-driven packages (e.g., ROS-Unity Bridge) facilitate communication with ROS/ROS 2.

## 2.2 Rendering Humanoid Motion and Environments

Creating a visually compelling simulation involves:
-   **3D Models**: Importing detailed humanoid robot models (e.g., FBX, URDF converted to Unity assets).
-   **Animations**: Applying realistic animations to humanoid rigs (e.g., from motion capture data, inverse kinematics).
-   **Environments**: Designing rich 3D environments with realistic textures, lighting, and props.
-   **Cameras**: Setting up cameras to view the scene from different perspectives.

## 2.3 Basic Human-Robot Interaction Scenes

Unity's interactivity allows for dynamic human-robot interaction scenarios:
-   **User Input**: Using keyboard, mouse, or VR controllers to interact with the robot or environment.
-   **UI Elements**: Creating in-sim user interfaces for commanding the robot or displaying information.
-   **Scripted Behaviors**: Programming robot responses to user actions or environmental changes.

## 2.4 Example: Animated Humanoid in Unity

In this section, we will create a simple Unity project, import a basic humanoid model, and set up an animation (e.g., a wave gesture) to demonstrate high-fidelity motion.

### 2.4.1 Setting Up a Unity Project

1.  **Install Unity Hub and Unity Editor**: If you haven't already, download and install Unity Hub and a Unity Editor LTS version (e.g., 2022.3.x) from the official Unity website.
2.  **Create a New Project**:
    *   Open Unity Hub.
    *   Click "New Project".
    *   Select a 3D Core template.
    *   Set the project name to `HumanoidSimulation` and the location to `src/module-2/unity/`.
    *   Click "Create Project".

### 2.4.2 Importing a Basic Humanoid Model

1.  **Obtain a Humanoid Model**: You can use a generic humanoid model from the Unity Asset Store (e.g., a free character model) or convert a URDF model (from Module 1) into a Unity-compatible format (e.g., FBX) using tools like `urdf_importer` for Unity.
2.  **Import Model**: Drag and drop the model file (e.g., `.fbx`) into the `Assets` folder within your Unity project.
3.  **Place in Scene**: Drag the imported humanoid model from the Project window into your current scene's Hierarchy window. Position it at (0,0,0) with rotation (0,0,0).

### 2.4.3 Setting Up a Simple Animation

1.  **Obtain Animation Clips**: You can find free humanoid animation clips (e.g., "wave," "idle") on the Unity Asset Store or platforms like Mixamo.
2.  **Import Animation**: Import the animation clip(s) into your Unity project's `Assets` folder.
3.  **Set up Animator**:
    *   Select your humanoid model in the Hierarchy.
    *   In the Inspector, ensure it has an `Animator` component. If not, add one.
    *   Create a new Animator Controller (right-click in Project window -> Create -> Animator Controller) and name it `HumanoidAnimatorController`.
    *   Drag the animation clip (e.g., "WaveAnimation") from the Project window into the Animator window.
    *   Drag `HumanoidAnimatorController` onto the `Controller` slot of the `Animator` component in your humanoid model's Inspector.

### 2.4.4 Running the Scene

1.  **Save Scene**: Save your current scene (File -> Save As...) as `Assets/Scenes/HumanoidScene.unity`.
2.  **Play Simulation**: Click the "Play" button at the top of the Unity Editor.
3.  **Observe**: Your humanoid robot should appear in the scene and perform the configured animation.
