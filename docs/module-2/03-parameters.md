---
sidebar_position: 3
title: Runtime Configuration (Parameters)
---

# Runtime Configuration (Parameters)

Hard-coding values is a bad habit in software engineering. If you hard-code your robot's walking speed to `1.0 m/s`, you have to stop the robot, edit the code, rebuild, and restart just to test `0.5 m/s`.

**Parameters** allow you to change these settings at **runtime** without touching the source code.

## The Concept

A Parameter is a configuration value (integer, float, boolean, string, or list) stored inside a Node.
*   It has a **name** (e.g., `walking_speed`).
*   It has a **type** (e.g., `Double`).
*   It can be read and written to externally.

## Using Parameters in Python

To use parameters, your Node needs to:
1.  **Declare** the parameter (telling ROS 2 that it exists).
2.  **Get** the parameter (reading its value).

```python
class LocomotionNode(Node):
    def __init__(self):
        super().__init__('locomotion_node')
        
        # 1. Declare the parameter with a default value
        self.declare_parameter('walking_speed', 1.0)
        self.declare_parameter('enable_balance_control', True)

        # 2. Use the parameter
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        # Fetch the current value (it might have changed!)
        speed = self.get_parameter('walking_speed').get_parameter_value().double_value
        
        if speed > 2.0:
            self.get_logger().warn('Speed is too high! Be careful.')
        
        # ... generate motor commands based on speed ...
```

## Changing Parameters via CLI

You can inspect and modify parameters from the command line while the node is running.

**List all parameters of a node:**
```bash
ros2 param list /locomotion_node
```

**Get a specific parameter:**
```bash
ros2 param get /locomotion_node walking_speed
# Output: Double value is: 1.0
```

**Set a new value (The Magic Part):**
```bash
ros2 param set /locomotion_node walking_speed 0.5
# Output: Set parameter successful
```

The moment you run the `set` command, the `control_loop` in your Python code will receive `0.5` instead of `1.0`.

## Loading Parameters from a File

For complex robots, you can load hundreds of parameters from a YAML file at launch time:

```yaml
/locomotion_node:
  ros__parameters:
    walking_speed: 0.8
    enable_balance_control: true
    pid_gains: [1.2, 0.05, 0.1]
```

This separates your **Configuration** from your **Code**, a key principle of robust software architecture.
