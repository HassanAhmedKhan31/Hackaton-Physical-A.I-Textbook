---
sidebar_position: 2
title: Launch Systems (Waking Up the Robot)
---

# Launch Systems (Waking Up the Robot)

A humanoid robot has many distinct parts: vision, balance, motor control, path planning, and voice recognition. If each of these is a separate Node (as they should be), starting them manually would be a nightmare. You would need to open 20 terminal tabs and type 20 commands every time you turned the robot on.

**Launch Files** are the solution. They act as the "Main Switch" that wakes up the entire system.

## The Concept

A Launch File is a Python script that describes the **configuration** of your system. It tells ROS 2:
1.  Which Nodes to start.
2.  What parameters to pass to them.
3.  Which other Launch Files to include.

## Modern Python Syntax

In older versions of ROS (ROS 1), launch files were XML. In ROS 2, they are **Python**. This gives you the full power of a programming language to manage your robot's startup sequence.

We use two main classes:
*   **`LaunchDescription`**: The container for everything we want to run.
*   **`Node`**: Represents a specific executable to run.

## Example: `bringup.launch.py`

Let's write a launch file that starts a "Brain Node" and a "Motor Node" simultaneously.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the 'Brain' of the robot
        Node(
            package='my_humanoid_control',
            executable='brain_node',
            name='humanoid_brain',
            output='screen',
            parameters=[
                {'intelligence_level': 'high'}
            ]
        ),
        
        # Start the 'Motor Controller'
        Node(
            package='my_humanoid_control',
            executable='motor_driver',
            name='leg_controller',
            output='screen'
        )
    ])
```

### Breakdown
1.  **`package`**: The name of the package where the executable lives.
2.  **`executable`**: The name of the script/binary (defined in `setup.py` entry points).
3.  **`name`**: A specific name for this instance of the node.
4.  **`output='screen'`**: Ensures that `print()` statements from the node appear in your terminal.

## Running the Launch File

Instead of `ros2 run`, we use `ros2 launch`:

```bash
ros2 launch my_humanoid_control bringup.launch.py
```

## Including Other Launch Files

Launch files are composable. You can write a "Master Launch File" that includes the launch files for the arms, legs, and head.

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_vision_pkg'), 'launch', 'camera.launch.py')
        )
    )
    
    return LaunchDescription([
        vision_launch,
        # ... other nodes
    ])
```

This modularity allows different teams to work on different parts of the robot and combine them effortlessly.
