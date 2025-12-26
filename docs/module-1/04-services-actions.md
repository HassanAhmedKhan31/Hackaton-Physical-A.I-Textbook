---
sidebar_position: 4
title: Reflexes & Requests (Services)
---

# Reflexes & Requests (Services)

Topics are great for continuous data (sensor streams, heartbeats), like listening to a radio. But sometimes you need to ask a specific question and get a specific answer.

-   **Topic**: "Here is the current joint angle... here is the current joint angle..." (Stream)
-   **Service**: "Calculate the joint angle needed to reach coordinate X, Y." -> "Okay, the angle is 45 degrees." (Request/Response)

## The Synchronous Nature

Services are **synchronous**. When a node calls a service, it usually waits (blocks) until it gets an answer. This is like a function call `calculate_angle()`, but the function is running on a completely different computer (or node).

## Practical: Joint Angle Calculator

We will implement a simple service. Standard ROS 2 examples often use `AddTwoInts`. To make it relevant, let's frame it as a **Joint Angle Calculator** that adds a base offset to a requested movement.

### The Service Server

This node offers the service. It waits for requests.

*Note: For this example, we will use the standard `example_interfaces` package which contains `AddTwoInts` to avoid creating custom msg definitions just yet.*

```python title="angle_service_server.py"
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class JointAngleServer(Node):

    def __init__(self):
        super().__init__('joint_angle_server')
        # Create the service
        # Service Type: AddTwoInts
        # Service Name: 'calculate_joint_angle'
        self.srv = self.create_service(AddTwoInts, 'calculate_joint_angle', self.calculate_callback)
        self.get_logger().info('Joint Angle Calculator Ready.')

    def calculate_callback(self, request, response):
        # request.a = Base Offset
        # request.b = Desired Movement
        
        response.sum = request.a + request.b
        
        self.get_logger().info(f'Incoming Request: Base={request.a} + Move={request.b}')
        self.get_logger().info(f'Sending Response: Target Angle={response.sum}')
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = JointAngleServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### The Service Client

This node sends a request and waits for the answer.

```python title="angle_service_client.py"
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class JointAngleClient(Node):

    def __init__(self):
        super().__init__('joint_angle_client')
        self.cli = self.create_client(AddTwoInts, 'calculate_joint_angle')
        
        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            
        self.req = AddTwoInts.Request()

    def send_request(self, base, move):
        self.req.a = base
        self.req.b = move
        
        # Asynchronous call, but we will wait for the future object
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    
    node = JointAngleClient()
    
    # Send a request: Base 30 degrees + Movement 15 degrees
    response = node.send_request(30, 15)
    
    node.get_logger().info(
        f'Result: Calculated Target Angle is {response.sum} degrees')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

Use **Services** when you need a confirmation or a specific computation done on demand. Use **Topics** for continuous data flow where missing one packet isn't catastrophic.