# Tasks: Module 1: The Robotic Nervous System (ROS 2)

**Input**: Design documents from `/specs/001-module-1-ros2/`
**Prerequisites**: plan.md, spec.md

## Phase 1: Setup (Shared Infrastructure)

- [X] T001 Create the directory structure for Module 1 as outlined in `plan.md` (`my-website/docs/module-1`, `src/module-1/examples/chapter-1`, `src/module-1/examples/chapter-2`, `src/module-1/urdf`).
- [X] T002 Add a new sidebar entry for Module 1 in `my-website/sidebars.ts`.

---

## Phase 2: User Story 1 - Understanding Core ROS 2 Concepts

**Goal**: A robotics learner can understand the fundamental ROS 2 concepts (Nodes, Topics, Services, Actions) and how they apply to humanoid communication.

**Independent Test**: A user can read Chapter 1, compile and run the associated examples, and correctly describe the function of each core ROS 2 component.

### Tests for User Story 1

- [X] T003 [P] [US1] Create a test script to validate the ROS 2 "talker" example for Chapter 1.
- [X] T004 [P] [US1] Create a test script to validate the ROS 2 "listener" example for Chapter 1.

### Implementation for User Story 1

- [X] T005 [P] [US1] Write the content for `my-website/docs/module-1/chapter-1-ros2-basics.md`, explaining Nodes, Topics, Services, and Actions.
- [X] T006 [P] [US1] Create the ROS 2 "talker" example code in `src/module-1/examples/chapter-1/talker.py`.
- [X] T007 [P] [US1] Create the ROS 2 "listener" example code in `src/module-1/examples/chapter-1/listener.py`.
- [X] T008 [US1] Add instructions to `chapter-1-ros2-basics.md` on how to run the examples and use ROS 2 CLI tools to inspect them.

---

## Phase 3: User Story 2 - Bridging AI and ROS

**Goal**: A developer can write a simple Python agent that communicates with a ROS 2 system using `rclpy`.

**Independent Test**: A user can follow the tutorial in Chapter 2 to write and run a Python script that successfully publishes a message to a ROS 2 topic and subscribes to another.

### Tests for User Story 2

- [X] T009 [P] [US2] Create a test script for the AI-to-ROS publisher example.
- [X] T010 [P] [US2] Create a test script for the AI-to-ROS subscriber example.

### Implementation for User Story 2

- [X] T011 [P] [US2] Write the content for `my-website/docs/module-1/chapter-2-ai-ros-bridge.md`.
- [X] T012 [P] [US2] Create the Python agent publisher example in `src/module-1/examples/chapter-2/agent_publisher.py`.
- [X] T013 [P] [US2] Create the ROS 2 subscriber example in `src/module-1/examples/chapter-2/ros_subscriber.py`.
- [X] T014 [US2] Add instructions to `chapter-2-ai-ros-bridge.md` on how to run the pub/sub examples.

---

## Phase 4: User Story 3 - Defining a Humanoid Structure

**Goal**: A robotics engineer can understand the basics of defining a humanoid robot's structure using URDF.

**Independent Test**: A user can read Chapter 3, understand the provided sample URDF file, and successfully load it into a ROS 2 visualization tool like RViz2.

### Tests for User Story 3

- [X] T015 [P] [US3] Create a validation script that runs `check_urdf` on the humanoid URDF file.

### Implementation for User Story 3

- [X] T016 [P] [US3] Write the content for `my-website/docs/module-1/chapter-3-urdf-humanoids.md`.
- [X] T017 [P] [US3] Create the `humanoid.urdf` file in `src/module-1/urdf/`.
- [X] T018 [US3] Create a ROS 2 launch file to visualize the URDF in RViz2 and add instructions to `chapter-3-urdf-humanoids.md`.

---

## Phase 5: Polish & Cross-Cutting Concerns

- [X] T019 Review all content for technical accuracy and clarity.
- [X] T020 Validate that all code examples are runnable and produce the expected output.
- [X] T021 Ensure all Docusaurus content builds without errors.

---

## Dependencies & Execution Order

- **Phase 1 (Setup)** must be completed before any other phase.
- **Phases 2, 3, and 4 (User Stories)** can be worked on in parallel after Phase 1 is complete.
- **Phase 5 (Polish)** should be done after all other phases are complete.

Within each User Story phase, the **Tests** should be written first.
