# Module 3 Specification: Robot Simulation (Gazebo & Unity)

## Context
This module teaches students how to create a "Digital Twin" of their robot. They will use Gazebo for physics (gravity, collisions) and Unity for high-fidelity rendering.

## Target Audience
Students who have a working ROS 2 workspace (from Module 2) but have never run a 3D simulation.

## Design Constraints
- **Simulator Version:** Gazebo Fortress (formerly Ignition). *Do not use Gazebo Classic.*
- **Bridge:** Use `ros_gz_bridge` for communicating between ROS 2 and Gazebo.
- **Unity:** Use `ROS-TCP-Endpoint` for connecting Unity.
- **Diagrams:** Use Mermaid to explain the `ROS <-> Bridge <-> Gazebo` data flow.

## Deliverables

### 1. `docs/module-3/01-gazebo-environment.md`
- **Title**: The Simulation Lab (Setting up Gazebo)
- **Concept**: Gazebo is not just a visualizer; it is a math engine calculating gravity 1000 times a second.
- **Installation**: Step-by-step for installing `ros-humble-ros-gz`.
- **Practical**: 
  - Command to launch an empty world: `ign gazebo empty.sdf`.
  - Explanation of the GUI (Play/Pause, Entity Tree).

### 2. `docs/module-3/02-urdf-vs-sdf.md`
- **Title**: The Format War (URDF vs. SDF)
- **Concept**: 
  - URDF (Unified Robot Description Format) is for **ROS** (Kinematics).
  - SDF (Simulation Description Format) is for **Gazebo** (World & Physics).
- **The Workflow**: Explain that we write in URDF/Xacro, and ROS converts it to SDF for Gazebo automatically during spawning.
- **Table**: Compare URDF vs SDF (e.g., URDF is tree-structure only; SDF allows cycles/closed-loops).

### 3. `docs/module-3/03-physics-and-worlds.md`
- **Title**: Building the World
- **Concept**: Defining the environment (Sun, Ground Plane, Friction).
- **Practical**: 
  - Creating a `my_world.sdf` file.
  - Adding a "Ground Plane" and a "Crate" (box).
  - **Launch File**: A Python launch file that starts Gazebo and spawns a robot entity (`ros_gz_sim` + `create`).

### 4. `docs/module-3/04-simulating-sensors.md`
- **Title**: Virtual Eyes & Ears (Sensors)
- **Concept**: How to trick the robot into thinking it sees a wall.
- **Technical**: 
  - Adding a `<sensor>` tag to the URDF/SDF.
  - **Lidar**: Explain the Ray Sensor.
  - **Camera**: Explain the Image Sensor.
- **The Bridge**: Crucial section—explain `ros_gz_bridge`.
  - *Example Config*: Mapping `/lidar/points` (Gazebo) to `/scan` (ROS 2).

### 5. `docs/module-3/05-unity-visualization.md`
- **Title**: The Pretty Face (Unity Visualization)
- **Concept**: Why use Unity? (Better graphics for Human-Robot Interaction). Gazebo is for physics; Unity is for showing it to humans.
- **Setup**: 
  - Installing the "Unity Robotics Hub" packages.
  - Setting up the `ROS-TCP-Endpoint` node in ROS 2.
- **Workflow**: 
  1. Export URDF from ROS.
  2. Import URDF into Unity (drag and drop).
  3. Visualize the `/joint_states` topic to make the Unity robot move when the ROS robot moves.