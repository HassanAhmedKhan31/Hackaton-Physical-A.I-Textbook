---
sidebar_position: 5
title: The Body Schema (URDF)
---

# The Body Schema (URDF)

A nervous system is useless without a body. In ROS 2, the robot's physical structure is defined by the **Unified Robot Description Format (URDF)**. This is an XML file that tells the robot:
"Your head is connected to your neck, and your neck is connected to your torso."

## Links and Joints

-   **Link**: A solid part of the robot (e.g., Forearm, Bicep, Torso). It has mass and visual geometry.
-   **Joint**: The connection between two links. It defines how they move relative to each other (e.g., rotate, slide, or fixed).

## The Tree Structure

Crucially, URDF follows a **Tree Structure**. This means:
1.  There is always one **Root Link** (usually the `base_link` or `torso`).
2.  **Parent vs. Child**: Every joint connects a parent link to a child link.
3.  **The One-Parent Rule**: A child link can have **only one parent**. You cannot have two different joints connecting to the same child link. This ensures the kinematic chain is a clear path without loops.

## Practical: Defining a Simple Humanoid

Let's write a simple URDF that defines a Torso, a Neck, and a Head.

Create a file named `simple_humanoid.urdf`.

```xml title="simple_humanoid.urdf"
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- 1. The Torso (Base Link) -->
  <link name="torso">
    <visual>
      <geometry>
        <!-- A cylinder representing the body -->
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- 2. The Head -->
  <link name="head">
    <visual>
      <geometry>
        <!-- A sphere representing the head -->
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>

  <!-- 3. The Neck Joint -->
  <joint name="neck_joint" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <!-- Position the head on top of the torso (0.3 is half torso height) -->
    <origin xyz="0 0 0.45" rpy="0 0 0"/>
  </joint>

</robot>
```

## Visualizing the Body

To see your robot, you use a tool called **RViz2** (ROS Visualization).

Assuming you have the `urdf_tutorial` or standard `robot_state_publisher` packages installed, you can visualize this file. A common way to check it quickly is:

```bash
urdf_to_graphiz simple_humanoid.urdf
```
*(This generates a PDF diagram of your link structure).*

To see it in 3D, you would typically launch a `robot_state_publisher` node and then open RViz, but that requires a launch file which we will cover in Module 2.

## The Mental Model

Think of the URDF as the robot's **Proprioception Schema**. Without this file, the robot's brain doesn't know that rotating the "Neck Motor" moves the "Camera" (Head). The URDF provides the kinematic chain that allows the software to perform complex tasks like "Look at that ball" by calculating exactly how to move the neck joints.