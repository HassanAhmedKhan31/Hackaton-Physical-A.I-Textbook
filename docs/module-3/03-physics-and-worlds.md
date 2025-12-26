---
sidebar_position: 3
title: Building the World
---

# Building the World

Before we spawn our robot, we need a world for it to live in. In Gazebo, the world is defined using an **SDF** file.

## Concept: The Environment

A simulation environment isn't just 3D meshes. It defines the laws of physics that govern the space.
*   **Physics:** Gravity (usually -9.8 m/s² on Z), magnetic field, air density.
*   **Lighting:** Sun direction, ambient light.
*   **Entities:** Static objects (ground, walls) and dynamic objects (robots).

## Creating a World File (`my_world.sdf`)

Let's create a simple world with a ground plane and a box (crate).

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="my_simulated_world">
    
    <!-- 1. Physics Engine -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- 2. Sun -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>

    <!-- 3. Ground Plane -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri>
    </include>

    <!-- 4. A Simple Crate -->
    <model name="crate">
      <pose>1 0 0.5 0 0 0</pose> <!-- x y z r p y -->
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1 1 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1 1 1</size></box></geometry>
          <material><ambient>0.8 0.5 0.2 1</ambient></material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

## Launching Gazebo and Spawning a Robot

We use a Python launch file to start Gazebo with our world and then spawn our robot into it. We use the `ros_gz_sim` package for this.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_pkg')
    world_file = os.path.join(pkg_share, 'worlds', 'my_world.sdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')

    return LaunchDescription([
        # 1. Start Gazebo Server with our world
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world_file],
            output='screen'
        ),

        # 2. Spawn the Robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'my_robot',
                '-file', urdf_file,
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1'
            ],
            output='screen'
        )
    ])
```

### Explanation
1.  **`ign gazebo -r`**: Starts the simulator and immediately runs (`-r`) the physics (skipping the pause state).
2.  **`create` executable**: This tool takes a file (URDF or SDF) and spawns it into the running Gazebo instance at a specific coordinate.

---

**Bonus Task:**
Start the 'World Builder Agent' to generate a random maze SDF file for your robot to navigate.