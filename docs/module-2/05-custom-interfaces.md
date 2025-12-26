---
sidebar_position: 5
title: Defining the Protocol (Messages)
---

# Defining the Protocol (Messages)

In any language, you need words to communicate. If you want to talk about "Philosophy," you need complex words. If you want to talk about "Rocket Science," you need specific technical terms.

ROS 2 comes with a standard dictionary (`std_msgs`) containing simple words like `String`, `Int32`, and `Bool`. It also has `geometry_msgs` for physics.

But for a **Humanoid Robot**, these aren't enough. How do you send a message that says "Move the left pinky finger to 45 degrees"? You need to invent your own words.

## Custom Interfaces

We can define three types of custom "words":
1.  **Messages (`.msg`)**: For Topics (One-way communication).
2.  **Services (`.srv`)**: For Services (Request/Response).
3.  **Actions (`.action`)**: For Actions (Goal/Feedback/Result).

## Why Create Custom Interfaces?

Imagine trying to send a humanoid's joint states using standard messages. You might try to send an array of 20 floats.
*   **Bad:** `[0.5, 1.2, -0.4, ...]` -> What does the 3rd number mean? Is it the elbow or the knee?
*   **Good:** A custom message `JointState` with fields `name=['elbow', 'knee']` and `position=[0.5, 1.2]`.

## Creating a Message Package

It is **best practice** to keep your interface definitions in a separate package (e.g., `my_robot_interfaces`). This avoids circular dependencies.

### 1. The Definition
Create a file `JointAngles.msg`:

```text
string[] joint_names
float64[] joint_values
bool is_clamped
```

### 2. CMakeLists.txt Configuration

Even if you are using Python for your nodes, your interface package **must** use CMake to generate the language-specific bindings (C, C++, Python).

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointAngles.msg"
  "srv/HandGesture.srv"
)
```

### 3. package.xml Configuration

You also need to tell the build system that this package generates interfaces:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## Using Your Custom Message

Once built, you can import your new "word" just like any standard ROS 2 message:

```python
from my_robot_interfaces.msg import JointAngles

msg = JointAngles()
msg.joint_names = ['neck_pitch', 'neck_yaw']
msg.joint_values = [0.0, 1.57]
publisher.publish(msg)
```

By defining clear protocols, you ensure that the Brain of your robot speaks the same language as the Limbs.
