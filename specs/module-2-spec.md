# Module 2 Specification: ROS 2 Fundamentals & Architecture

## Context
This is the "Engineering Implementation" module of the "Physical AI & Humanoid Robotics" textbook. We are moving from single scripts to robust software packages.

## Target Audience
Students who know Python but have never built a Linux software package. They are preparing to build the "Brain" of a humanoid robot.

## Design Constraints
- **ROS Version:** ROS 2 Humble (Python).
- **Format:** Docusaurus Markdown (MDX).
- **Diagrams:** Use Mermaid.js for flowcharts.
- **Code Standards:** 
  - All Launch files must be Python-based (`.launch.py`), NOT XML.
  - All Nodes must use Object-Oriented Python (`class MyNode(Node):`).

## Deliverables

### 1. `docs/module-2/01-workspaces-and-packages.md`
- **Title**: The Workspace (The Robot's Home)
- **Concept**: Explain `colcon`, overlay workspaces, and the folder structure (`src`, `build`, `install`, `log`).
- **Analogy**: 
  - Workspace = The Kitchen
  - Package = A specific recipe (e.g., "Leg Control Package")
  - `colcon build` = Cooking the food so it's ready to serve.
- **Practical**: Commands to create a package: `ros2 pkg create --build-type ament_python my_humanoid_control`.

### 2. `docs/module-2/02-launch-files.md`
- **Title**: Launch Systems (Waking Up the Robot)
- **Concept**: We cannot open 20 terminal tabs to run a robot. We use Launch files.
- **Technical**: 
  - Explain `LaunchDescription`, `Node`, and `IncludeLaunchDescription`.
- **Code Example**: A `bringup.launch.py` that starts a "Brain Node" and a "Motor Node" simultaneously.

### 3. `docs/module-2/03-parameters.md`
- **Title**: Runtime Configuration (Parameters)
- **Concept**: Changing settings without changing code (e.g., changing walking speed from 1.0 m/s to 0.5 m/s while the robot is running).
- **Practical**: 
  - How to declare parameters in Python: `self.declare_parameter('speed', 1.0)`.
  - How to change them via CLI: `ros2 param set /robot_node speed 0.5`.

### 4. `docs/module-2/04-actions.md`
- **Title**: Long-Running Tasks (Actions)
- **Concept**: Difference between Services (Instant) and Actions (Long).
- **Analogy**: 
  - Service: "Turn on light" (Click -> Done).
  - Action: "Walk to the kitchen" (Start -> Feedback: 'Walking...' -> Feedback: 'Almost there...' -> Done).
- **Diagram**: A Mermaid sequence diagram showing Goal, Feedback, and Result.
- **Code Example**: A simple "Walk" action server skeleton.

### 5. `docs/module-2/05-custom-interfaces.md`
- **Title**: Defining the Protocol (Messages)
- **Concept**: Creating `.msg`, `.srv`, and `.action` files.
- **Why**: Standard messages (String/Int) aren't enough for humanoids. We need `JointAngles.msg` or `HandGesture.srv`.
- **Practical**: Steps to modify `CMakeLists.txt` (even in Python packages) to generate custom messages.