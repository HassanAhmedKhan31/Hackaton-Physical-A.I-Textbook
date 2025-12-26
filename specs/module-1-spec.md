# Module 1 Specification: The Robotic Nervous System

## Context
We are writing a textbook for "Physical AI & Humanoid Robotics". This module bridges the gap between Python scripts and physical hardware using ROS 2 (Robot Operating System).

## Target Audience
Students with Python experience but zero robotics experience. Use the analogy of the "Human Nervous System" to explain ROS 2 concepts.

## Design Constraints
- Use Docusaurus Markdown (MDX).
- Include Mermaid.js diagrams for all node/topic interactions.
- Code blocks must use Python (`rclpy`).
- Tone: Technical yet accessible, "Engineer-to-Engineer".

## Deliverables (Files to Generate)

### 1. `docs/module-1/01-intro-nervous-system.md`
- **Title**: The Nervous System of the Robot
- **Concept**: Explain that ROS 2 is not an OS (like Windows), but a middleware (like a nervous system) that lets different body parts talk to the brain.
- **Analogy**: 
  - Brain = High-level Logic
  - Nerves = ROS 2 Topics
  - Muscles/Senses = Nodes
- **Diagram**: A high-level diagram showing a Central Node connecting to Sensor Nodes and Actuator Nodes.

### 2. `docs/module-1/02-nodes-and-rclpy.md`
- **Title**: Neurons & Nodes (Writing Your First Agent)
- **Concept**: What is a ROS Node? (An executable process).
- **Practical**: 
  - How to install `rclpy`.
  - Code Example: A simple "MinimalPublisher" in Python that prints "Heartbeat" every second.
  - Explain the `class` structure in Python for ROS 2.

### 3. `docs/module-1/03-topics-and-messages.md`
- **Title**: Signals & Synapses (Topics)
- **Concept**: Publisher/Subscriber model.
- **Analogy**: Broadcasting a radio signal (Publisher) vs. Tuning into a station (Subscriber).
- **Practical**: 
  - Code Example: A "Sensor Node" publishing random temperature data.
  - Code Example: A "Control Node" subscribing to that data and printing a warning if it's too hot.
  - 

### 4. `docs/module-1/04-services-actions.md`
- **Title**: Reflexes & Requests (Services)
- **Concept**: Synchronous vs. Asynchronous communication.
- **Analogy**: Topics are like a live stream (continuous). Services are like a function call or a request/response (e.g., "Open Hand" -> "Hand Opened").
- **Practical**: Creating a simple Service that adds two numbers (classic ROS 2 example) but framed as "Calculate Joint Angle".

### 5. `docs/module-1/05-urdf-humanoid-body.md`
- **Title**: The Body Schema (URDF)
- **Concept**: How the robot knows where its arm is relative to its chest.
- **Technical**: Introduction to XML structure of URDF (Links and Joints).
- **Practical**: 
  - XML snippet defining a simple "Torso" link and a "Head" link connected by a "Neck" joint.
  - Visualization: How to view this in `rviz2`.