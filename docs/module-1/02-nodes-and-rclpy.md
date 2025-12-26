---
sidebar_position: 2
title: Neurons & Nodes
---

# Neurons & Nodes (Writing Your First Agent)

In the biological analogy, a **Node** is a neuron or a cluster of neurons dedicated to a single task. In ROS 2 terms, a Node is a single executable process—a Python script—that performs computation.

## Setting Up Your Environment

Before we write code, we need the Python client library for ROS 2: `rclpy` (ROS Client Library for Python).

:::info
Ensure you have sourced your ROS 2 installation (e.g., `source /opt/ros/humble/setup.bash` on Linux or `call C:\dev\ros2_humble\local_setup.bat` on Windows).
:::

## The Anatomy of a ROS 2 Node

We write Nodes as Python classes. This gives us structure and allows the node to maintain its own state (variables).

### Code Example: The "Heartbeat" Node

This node mimics a simple autonomic function: a heartbeat. It does one thing—publish a message saying "Lub-Dub" every second.

```python title="heartbeat_node.py"
import rclpy
from rclpy.node import Node

class HeartbeatNode(Node):
    def __init__(self):
        # 1. Initialize the Node with a name 'heartbeat'
        super().__init__('heartbeat')
        
        # 2. Create a timer that fires every 1.0 seconds
        # This is like a biological pacemaker cell
        self.timer = self.create_timer(1.0, self.beat_callback)
        
        # Counter to keep track of beats
        self.beat_count = 0
        self.get_logger().info('Heartbeat Node Started')

    def beat_callback(self):
        # 3. This function is called every time the timer expires
        self.beat_count += 1
        self.get_logger().info(f'Lub-Dub: Beat #{self.beat_count}')

def main(args=None):
    # Initialize the ROS 2 communication layer
    rclpy.init(args=args)
    
    # Create the node
    node = HeartbeatNode()
    
    # Spin the node so it stays alive and processes callbacks
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Breaking It Down

1.  **`class HeartbeatNode(Node):`**: We inherit from the base `Node` class. This gives our Python class all the superpowers of a ROS 2 entity.
2.  **`super().__init__('heartbeat')`**: Every node needs a unique name on the network.
3.  **`create_timer(1.0, self.beat_callback)`**: This is how we schedule tasks. Instead of a `while True:` loop (which blocks everything), we register a callback. ROS 2 calls *us* when the time is right.
4.  **`rclpy.spin(node)`**: This keeps the program running. It puts the node in a loop where it waits for events (like timers or messages) and executes the associated callbacks.

## Running the Node

Save this file and run it with Python:

```bash
python3 heartbeat_node.py
```

You should see:
```text
[INFO] [heartbeat]: Heartbeat Node Started
[INFO] [heartbeat]: Lub-Dub: Beat #1
[INFO] [heartbeat]: Lub-Dub: Beat #2
...
```

Congratulations! You have just created the first spark of life in your robot's nervous system.