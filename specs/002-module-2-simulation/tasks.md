# Tasks: Module 2: The Digital Twin (Gazebo & Unity)

**Input**: Design documents from `/specs/002-module-2-simulation/`
**Prerequisites**: plan.md, spec.md

## Phase 1: Setup (Shared Infrastructure)

- [X] T001 Create the directory structure for Module 2 as outlined in `plan.md` (`my-website/docs/module-2`, `src/module-2/gazebo`, `src/module-2/unity`, `src/module-2/ros_pkgs`).
- [X] T002 Add a new sidebar entry for Module 2 in `my-website/sidebars.ts`.

---

## Phase 2: User Story 1 - Simulating Humanoid Physics in Gazebo

**Goal**: A robotics developer can set up a stable physics simulation of a humanoid robot in Gazebo, including gravity, collisions, and joint constraints.

**Independent Test**: A user can follow the instructions in Chapter 1 to launch a Gazebo simulation where a humanoid robot model is standing and stable under gravity.

### Tests for User Story 1

- [X] T003 [P] [US1] Create a test script to launch a Gazebo world and verify the humanoid model is stable.

### Implementation for User Story 1

- [X] T004 [P] [US1] Write the content for `my-website/docs/module-2/chapter-1-gazebo-physics.md`.
- [X] T005 [P] [US1] Create a sample humanoid URDF model for Gazebo simulation in `src/module-2/gazebo/models/humanoid/model.urdf`.
- [X] T006 [P] [US1] Create a Gazebo world file (`src/module-2/gazebo/worlds/humanoid_stable.world`) with the humanoid model and basic physics.
- [X] T007 [US1] Add instructions to `chapter-1-gazebo-physics.md` on how to launch the Gazebo simulation and verify stability.

---

## Phase 3: User Story 2 - Creating High-Fidelity Interactions in Unity

**Goal**: A simulation engineer can render humanoid motions and environments in Unity and create a basic human-robot interaction scene.

**Independent Test**: A user can follow Chapter 2 to import a humanoid robot and an environment into Unity and create a simple interaction scenario (e.g., the robot waving).

### Tests for User Story 2

- [X] T008 [P] [US2] Create a Unity test script to verify basic humanoid animation playback.

### Implementation for User Story 2

- [X] T009 [P] [US2] Write the content for `my-website/docs/module-2/chapter-2-unity-interaction.md`.
- [X] T010 [P] [US2] Create a Unity project structure in `src/module-2/unity/`.
- [X] T011 [P] [US2] Import a basic humanoid model into the Unity project and set up a simple animation in `src/module-2/unity/Assets/Scenes/HumanoidScene.unity`.
- [X] T012 [US2] Add instructions to `chapter-2-unity-interaction.md` on setting up the Unity environment and playing the animation.

---

## Phase 4: User Story 3 - Simulating Robot Sensors

**Goal**: A perception engineer can simulate common robot sensors (LiDAR, depth camera, IMU) and integrate the data into a ROS 2 pipeline.

**Independent Test**: A user can follow the examples in Chapter 3 to add a simulated sensor to a robot model in Gazebo or Unity and visualize the data in ROS 2.

### Tests for User Story 3

- [X] T013 [P] [US3] Create a ROS 2 test script to subscribe to simulated sensor topics and verify data reception.

### Implementation for User Story 3

- [X] T014 [P] [US3] Write the content for `my-website/docs/module-2/chapter-3-simulated-sensors.md`.
- [X] T015 [P] [US3] Create a Gazebo plugin for a simulated LiDAR sensor (or adapt an existing one) in `src/module-2/gazebo/models/humanoid_sensors/`.
- [X] T016 [P] [US3] Create a ROS 2 package `src/module-2/ros_pkgs/sensor_integrator` to bridge simulated sensor data to ROS 2.
- [X] T017 [US3] Add instructions to `chapter-3-simulated-sensors.md` on how to integrate and visualize simulated sensor data in RViz2.

---

## Phase 5: Polish & Cross-Cutting Concerns

- [X] T018 Review all content for technical accuracy and clarity, especially concerning Gazebo/Unity differences and integration.
- [X] T019 Validate that all code examples and simulation setups are runnable and produce the expected output.
- [X] T020 Ensure all Docusaurus content builds without errors and integrates Module 2 correctly.

---

## Dependencies & Execution Order

- **Phase 1 (Setup)** must be completed before any other phase.
- **Phases 2, 3, and 4 (User Stories)** can be worked on in parallel after Phase 1 is complete.
- **Phase 5 (Polish)** should be done after all other phases are complete.

Within each User Story phase, the **Tests** should be written first.
