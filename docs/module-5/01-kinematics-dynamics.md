---
id: kinematics-dynamics    
title: Kinematics & Dynamics
---

import PersonalizedNote from '@site/src/components/PersonalizedContent';

# The Math of Movement (Kinematics)

To make a humanoid robot move, we need to understand how its joints relate to the position of its body in 3D space. This study of motion without considering forces is called **Kinematics**.

## 1. Forward vs. Inverse Kinematics

### Forward Kinematics (FK)
Imagine you are a robot controller. You know the exact angle of every servo in your arm. 
* **The Question:** "If I move my shoulder joint by 30 degrees and my elbow joint by 15 degrees, where exactly is my fingertip in space?"
* **The Process:** FK is a straightforward calculation. You start from the base (the shoulder) and add up the lengths and angles of the links until you reach the tip (the end-effector).

### Inverse Kinematics (IK)
Now imagine you want to pick up a coffee mug.
* **The Question:** "I want my hand to be at coordinate (X=0.5, Y=0.2, Z=0.8). What angles do I need to set for my shoulder, elbow, and wrist joints to get there?"
* **The Process:** IK is much harder. There might be multiple ways to reach the same point (try reaching for your coffee mug with your elbow up vs. your elbow down). Our software uses complex solvers to find the most efficient joint angles to reach a target.

<PersonalizedNote background="software">
**Software Engineer Perspective:** Think of the Inverse Kinematics solver as a complex API endpoint. You provide the payload (target (X,Y,Z, Roll, Pitch, Yaw)) and it returns the configuration array (joint angles). You don't need to know the trigonometry inside; you just need to handle the result.
</PersonalizedNote>

<PersonalizedNote background="hardware">
**Hardware/Mechanical Perspective:** IK is where your motor selection matters most. Even if the math works, if your servos don't have enough torque (effort) to reach a specific pose against gravity, the IK solution is useless. Always validate IK outputs against your physical stall torque limits.
</PersonalizedNote>

## 2. Practical: URDF Joint Limits
In Module 1, we built a basic URDF. Now, we must add physical constraints to make the simulation realistic.

In your `.urdf` file, the `<limit>` tag is essential:
```xml
<joint name="elbow_joint" type="revolute">
  ...
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
</joint>