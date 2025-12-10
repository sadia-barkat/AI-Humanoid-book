# Chapter 1: Isaac Sim Basics - Building Photorealistic Robot Worlds

This chapter introduces NVIDIA Isaac Sim, a powerful robotics simulation platform built on NVIDIA Omniverse. We will learn how to create photorealistic 3D scenes, integrate robot models, and generate high-quality synthetic vision data—a critical component for training perception AI in robotics.

## 1.1 Introduction to NVIDIA Isaac Sim

NVIDIA Isaac Sim is a scalable and physically accurate robotics simulation application that makes it easy to create physically accurate virtual robot worlds and generate synthetic data for training AI models. It is built on NVIDIA Omniverse, a platform for connecting and building 3D applications.

### Key Features:
-   **Photorealistic Rendering**: Leverages NVIDIA RTX technology for realistic visuals.
-   **Physically Accurate Simulation**: Employs NVIDIA PhysX for accurate rigid body dynamics.
-   **Synthetic Data Generation**: Generates ground-truth data (e.g., RGB, depth, segmentation, bounding boxes) for AI training.
-   **ROS/ROS 2 Integration**: Seamlessly connects with ROS ecosystems.
-   **Python Scripting**: Automate and customize simulations using Python.

## 1.2 Photorealistic Scenes and Asset Management

Creating compelling and effective simulations starts with well-designed scenes.

### 1.2.1 USD (Universal Scene Description)
Isaac Sim uses USD as its core scene description format. USD is a powerful, extensible open-source 3D scene description technology developed by Pixar. It allows for modularity, layering, and collaboration in building complex virtual environments.
-   **Assets**: Individual models, textures, and materials (e.g., robot models, furniture, obstacles).
-   **Scenes**: Compositions of assets arranged in a virtual environment.

### 1.2.2 Building a Simple Scene
We will learn how to:
-   Load existing 3D assets from Isaac Sim's library.
-   Import custom robot models (e.g., USD versions of our humanoid).
-   Arrange objects, set up lighting, and define camera views.

## 1.3 Synthetic Vision Data Generation

Synthetic data is crucial for training robust perception models, especially when real-world data is scarce or expensive to collect. Isaac Sim can generate various types of synthetic data:
-   **RGB Images**: Photorealistic camera feeds.
-   **Depth Images**: Distance from the camera to objects.
-   **Segmentation Maps**: Categorical labels for each pixel (e.g., "robot," "table," "wall").
-   **Bounding Boxes**: 2D and 3D bounding box annotations for object detection.

## 1.4 Example: Generating Synthetic Data from a Simple Scene

In this section, we will create a basic Isaac Sim scene with our humanoid robot and a camera, then use Python scripting to capture synthetic RGB and depth data.

### 1.4.1 Setting Up Isaac Sim

1.  **Install Isaac Sim**: Follow the official NVIDIA documentation to install Isaac Sim, including Omniverse Launcher and the Isaac Sim application. Ensure your system meets the GPU requirements.
2.  **Launch Isaac Sim**: Start Isaac Sim either through the Omniverse Launcher or by running the `isaac_sim.sh` script (typically found in your Isaac Sim installation directory).

### 1.4.2 Creating a Simple USD Scene

We will use the placeholder USD scene file created at `src/module-3/isaac_sim/scenes/simple_photorealistic_scene.usd`. This file needs to be prepared in the Isaac Sim editor or via Python scripting.

### 1.4.3 Generating Synthetic Vision Data

```python
# src/module-3/isaac_sim/scripts/synthetic_data_generator.py
import os
import argparse
# import carb
# from omni.isaac.kit import SimulationApp
# from omni.isaac.core import World
# from omni.isaac.synthetic_utils import SyntheticDataHelper

def generate_synthetic_data(scene_path, output_dir, num_frames=10):
    """
    Placeholder function to demonstrate synthetic data generation workflow in Isaac Sim.
    
    Args:
        scene_path (str): Path to the USD scene file.
        output_dir (str): Directory to save generated data.
        num_frames (int): Number of frames to capture.
    """
    print(f"Placeholder: Attempting to generate synthetic data from {scene_path}")
    print(f"Placeholder: Output will be saved to {output_dir}")

    # In a real Isaac Sim script, you would:
    # 1. Initialize the SimulationApp.
    # 2. Acquire a World instance.
    # 3. Load the USD scene (World.load_stage(scene_path)).
    # 4. Create and configure a camera sensor in the scene.
    # 5. Initialize SyntheticDataHelper (sd_helper = SyntheticDataHelper()).
    # 6. Set up render product for RGB, Depth, etc.
    # 7. Loop to step the simulation and capture data.
    # 8. Clean up (kit.close()).
    
    # Simulate data generation by creating placeholder files
    os.makedirs(output_dir, exist_ok=True)
    for i in range(num_frames):
        rgb_file = os.path.join(output_dir, f"rgb_{i:04d}.png")
        depth_file = os.path.join(output_dir, f"depth_{i:04d}.npy")
        
        with open(rgb_file, 'w') as f:
            f.write(f"Placeholder RGB image data for frame {i}\n")
        with open(depth_file, 'w') as f:
            f.write(f"Placeholder Depth data for frame {i}\n")
            
    print(f"Placeholder: Generated {num_frames} RGB and Depth placeholder files in {output_dir}")
    print("This script needs to be run within an Isaac Sim environment to be fully functional.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Generate synthetic data from Isaac Sim scene.")
    parser.add_argument("--scene", type=str, default="simple_photorealistic_scene.usd",
                        help="USD scene file path relative to src/module-3/isaac_sim/scenes/")
    parser.add_argument("--output", type=str, default="output_data",
                        help="Output directory for synthetic data (relative to script location).")
    parser.add_argument("--frames", type=int, default=10,
                        help="Number of frames to generate.")
    
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    scene_abs_path = os.path.join(script_dir, "../../scenes/", args.scene)
    output_abs_dir = os.path.join(script_dir, args.output)

    generate_synthetic_data(scene_abs_path, output_abs_dir, args.frames)
    print("\nNote: To run this script fully, you need to execute it in an Isaac Sim Python environment.")
```

### 1.4.4 Launching Isaac Sim and Running the Script

To launch Isaac Sim and run your data generation script:

1.  **Launch Isaac Sim**: Ensure Isaac Sim is running. You can launch it with the Omniverse Launcher, or by running `isaac_sim.sh`.
2.  **Open the Scene**: In Isaac Sim's UI, open your `simple_photorealistic_scene.usd` file.
3.  **Run the Python Script**: You can execute the `synthetic_data_generator.py` script:
    *   **From Isaac Sim's Script Editor**: Copy-paste and run the script directly in Isaac Sim's built-in script editor (Window -> Script Editor).
    *   **From a Standalone Terminal**: Run the script from a terminal after sourcing the Isaac Sim Python environment (e.g., `python /path/to/your_script.py`).

    ```bash
    # Example command to run from standalone terminal (after sourcing Isaac Sim env)
    python src/module-3/isaac_sim/scripts/synthetic_data_generator.py --scene simple_photorealistic_scene.usd --output ./output_data --frames 5
    ```
    (Note: The script needs to be adapted to actually interact with the running Isaac Sim instance for real data generation).
