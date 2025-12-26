# Module 4 Specification: The AI-Robot Brain (NVIDIA Isaac)

## Context
This module focuses on "Hardware Acceleration" and "Photorealism." We are moving from Gazebo (CPU physics) to NVIDIA Isaac Sim (GPU physics + Ray Tracing).

## Target Audience
Students with RTX-enabled workstations. They understand basic ROS 2 nodes (from Module 2) but need to make them fast and intelligent.

## Design Constraints
- **Platform:** NVIDIA Omniverse & Isaac Sim.
- **Hardware:** Explicitly mention the requirement for RTX GPUs (Video RAM is key).
- **File Format:** Explain **USD** (Universal Scene Description).
- **Diagrams:** Use Mermaid to show the "Sim-to-Real" workflow.

## Deliverables

### 1. `docs/module-4/01-isaac-sim-omniverse.md`
- **Title**: The Omniverse (Isaac Sim)
- **Concept**: Explain that Omniverse is a platform for "connecting 3D worlds."
- **Key Terms**: 
  - **Nucleus**: The local server that holds files.
  - **USD**: The "HTML of 3D" (Universal Scene Description).
- **Practical**: 
  - How to launch Isaac Sim.
  - Loading a standard "Warehouse" environment from the assets tab.

### 2. `docs/module-4/02-isaac-ros-gems.md`
- **Title**: Supercharging ROS (Isaac ROS GEMs)
- **Concept**: Standard ROS nodes are slow. Isaac ROS nodes use the GPU.
- **Technical**: 
  - Explain **NITROS** (NVIDIA Isaac Transport for ROS)—simulating zero-copy memory transfer.
  - **Practical**: Setting up the `isaac_ros_common` docker container (essential for running these nodes).

### 3. `docs/module-4/03-nav2-and-vslam.md`
- **Title**: Spatial Awareness (VSLAM & Nav2)
- **Concept**: How the robot knows where it is without GPS (using Cameras/Lidar).
- **The Stack**:
  - **Visual SLAM**: Using `isaac_ros_visual_slam`.
  - **Nav2**: The ROS 2 Navigation Stack (Planning a path from A to B).
- **Diagram**: Flowchart: Camera Image -> VSLAM Node -> `/tf` (Map Frame) -> Nav2 -> Motor Controllers.

### 4. `docs/module-4/04-synthetic-data-generation.md`
- **Title**: Training the Brain (Synthetic Data)
- **Concept**: We can't take 1 million photos of a robot in the real world to train an AI. We use Isaac Sim to *generate* them.
- **Tool**: **NVIDIA Replicator**.
- **Practical**: 
  - Explain "Domain Randomization" (changing lights, textures, and camera angles automatically).
  - How to export a dataset for YOLO/Object Detection training.

### 5. `docs/module-4/05-rl-and-sim-to-real.md`
- **Title**: Walking with AI (Reinforcement Learning)
- **Concept**: Traditional robots use math for walking. Modern humanoids learn to walk via trial and error in simulation.
- **Workflow**: 
  1. Train in Isaac Sim (thousands of robots falling down in parallel).
  2. **Sim-to-Real Gap**: The difference between the perfect sim and the messy real world.
  3. Deploy the "Policy" (Neural Network) to the physical Jetson.