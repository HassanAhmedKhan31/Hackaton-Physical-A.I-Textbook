---
sidebar_position: 2
title: The Format War (URDF vs. SDF)
---

# The Format War (URDF vs. SDF)

As a roboticist, you are caught in a format war between two giants: **ROS** and **Gazebo**.

## The Contenders

### 1. URDF (Unified Robot Description Format)
*   **Champion:** ROS 2
*   **Purpose:** Describing the robot's **Kinematics** (how parts are connected).
*   **Strength:** Standard for ROS tools like RViz, `robot_state_publisher`, and MoveIt.
*   **Weakness:** It can only represent **Tree Structures** (Parent -> Child). It cannot handle closed loops (like a four-bar linkage) or detailed world environments.

### 2. SDF (Simulation Description Format)
*   **Champion:** Gazebo
*   **Purpose:** Describing the **Simulation** (Robot + World + Physics).
*   **Strength:** Can describe lighting, terrain, friction, and **Closed Kinematic Chains**.
*   **Weakness:** Not natively understood by many ROS 2 tools.

## Comparison Table

| Feature | URDF | SDF |
| :--- | :--- | :--- |
| **Primary User** | ROS 2 (RViz, MoveIt) | Gazebo |
| **Scope** | Robot Only | Robot + Environment |
| **Structure** | Tree (No Loops) | Graph (Loops Allowed) |
| **Physics** | Basic (Inertia, Collision) | Advanced (Friction, Stiffness, Bounce) |
| **Sensors** | Basic Definition | Full Plugin Configuration |

## The Workflow: Best of Both Worlds

We do **not** want to maintain two separate files for our robot. Instead, we use a conversion workflow.

1.  **Write in URDF (or Xacro):** We define our robot in URDF because ROS is our primary development environment.
2.  **Add Gazebo Tags:** We add special `<gazebo>` tags inside the URDF to specify physics properties (like friction) that URDF doesn't natively support.
3.  **Automatic Conversion:** When we spawn the robot, ROS 2 tools (like `ros_gz_sim`) convert our URDF into SDF in the background before handing it to Gazebo.

### Example: Adding Gazebo Properties to URDF

```xml
<link name="wheel_link">
  <!-- Standard URDF Visual/Collision/Inertia -->
  <visual>...</visual>
  <collision>...</collision>
  <inertial>...</inertial>
</link>

<!-- Gazebo Extension -->
<gazebo reference="wheel_link">
  <mu1>1.0</mu1> <!-- Static Friction -->
  <mu2>1.0</mu2> <!-- Dynamic Friction -->
  <material>Gazebo/Black</material>
</gazebo>
```

By adding these tags, we ensure our robot looks correct in RViz (URDF) and behaves correctly in Gazebo (SDF).