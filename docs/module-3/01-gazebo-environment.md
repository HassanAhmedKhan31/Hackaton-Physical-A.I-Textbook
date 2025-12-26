---
sidebar_position: 1
title: The Simulation Lab (Setting up Gazebo)
---

# The Simulation Lab (Setting up Gazebo)

Welcome to the **Matrix**.

In robotics, testing on hardware is expensive. A bad line of code can burn out a $200 motor or smash a 3D-printed limb. To solve this, we use **Simulation**.

## What is Gazebo?

Gazebo is not just a video game. It is a **Physics Engine**.
When you run a simulation, Gazebo is calculating Newton's laws of motion, collision detection, friction, and inertia—typically 1000 times every second.

**Note:** We are using **Gazebo Fortress** (formerly known as Ignition Gazebo), the modern successor to Gazebo Classic.

## Installation

We need to install the Gazebo simulator and the ROS 2 integration packages.

```bash
sudo apt-get install ros-humble-ros-gz
```

This command installs:
1.  **Gazebo Fortress:** The simulator itself.
2.  **ros_gz_bridge:** The translator between ROS 2 and Gazebo.
3.  **ros_gz_sim:** Tools to manage the simulation from ROS.

## Launching the Lab

Let's verify your installation by launching an empty world.

```bash
ign gazebo empty.sdf
```

### The GUI Tour

When Gazebo opens, you will see:
1.  **The Scene (Center):** The 3D view of the world.
2.  **Entity Tree (Left):** A list of all objects (models, lights, sensors) in the world.
3.  **Play/Pause (Bottom Left):** The most important button! Physics starts "Paused" by default. You must press Play to let gravity take effect.
4.  **Component Inspector (Right):** Shows details (position, mass) of the selected object.

## Why Simulate?

1.  **Safety:** You can't break virtual hardware.
2.  **Speed:** You can run simulations faster than real-time.
3.  **Environment:** You can test your robot on Mars, underwater, or in a burning building without leaving your desk.

In the next section, we will learn how to describe our robot so Gazebo understands it.