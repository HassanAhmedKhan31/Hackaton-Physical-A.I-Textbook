# Module 5 Specification: Humanoid Robot Development

## Context
This module focuses on the mechanical control of a humanoid form. We move from "rolling" (wheels) to "walking" (legs) and "grabbing" (hands).

## Target Audience
Students who have their simulation running (Module 3) and AI tools ready (Module 4). Now they need to make the robot move like a human.

## Design Constraints
- **Frameworks:** ROS 2 Control, MoveIt 2.
- **Math Level:** Explain "Inverse Kinematics" clearly without getting bogged down in matrix algebra.
- **Visuals:** Use Mermaid graphs to show the "Control Loop" (Sense -> Plan -> Act).

## Deliverables

### 1. `docs/module-5/01-kinematics-dynamics.md`
- **Title**: The Math of Movement (Kinematics)
- **Concept**:
  - **Forward Kinematics (FK)**: "If I move my shoulder 30°, where is my hand?"
  - **Inverse Kinematics (IK)**: "I want my hand *here*. What angles do I need for my shoulder and elbow?"
- **Practical**: Introduction to the **URDF** `<joint>` limits (effort, velocity) and how they affect physics.

### 2. `docs/module-5/02-bipedal-locomotion.md`
- **Title**: The Art of Falling (Bipedal Walking)
- **Concept**: Walking is just "controlled falling."
- **Key Term**: **ZMP** (Zero Moment Point). Explain that to stay upright, the center of pressure must stay within the foot support polygon.
- **Technology**: 
  - Overview of **MPC** (Model Predictive Control) for balance.
  - How `ros2_control` manages the high-frequency motor updates (1000Hz) needed for balance.

### 3. `docs/module-5/03-manipulation-grasping.md`
- **Title**: Grasping the World (MoveIt 2)
- **Concept**: Planning a path for an arm without hitting the table or itself.
- **Tool**: **MoveIt 2** (The standard for manipulation in ROS 2).
- **Practical**: 
  - Setting up the `MoveIt Setup Assistant`.
  - Generating the SRDF (Semantic Robot Description Format) - defining "groups" like `left_arm` and `right_hand`.

### 4. `docs/module-5/04-human-robot-interaction.md`
- **Title**: Social Robotics (HRI)
- **Concept**: Designing interactions that feel "Natural."
- **Topics**: 
  - **Proxemics**: How close should a robot stand to a human?
  - **Gaze**: Why the robot should look at what it is manipulating (Joint Attention).
  - **Safety**: Velocity scaling (moving slower when humans are near).

### 5. `docs/module-5/05-capstone-integration.md`
- **Title**: Assembling the Avatar (The Capstone)
- **Concept**: Bringing it all together.
- **The Loop**: 
  1. **Hear** (Whisper - Module 6 Preview)
  2. **See** (Isaac ROS VSLAM - Module 4)
  3. **Think** (LLM/VLA)
  4. **Walk** (Nav2 + Locomotion Controller)
  5. **Grab** (MoveIt 2)
- **Diagram**: A massive system architecture diagram showing the data flow between all modules.
