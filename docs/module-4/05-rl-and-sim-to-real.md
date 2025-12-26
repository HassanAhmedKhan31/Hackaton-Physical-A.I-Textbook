# Walking with AI (Reinforcement Learning)

Classical robotics uses explicit mathematical equations (Inverse Kinematics) to calculate how a robot should move. However, for complex tasks like humanoid walking, the math becomes incredibly difficult.

Enter **Reinforcement Learning (RL)**. Instead of telling the robot *how* to walk, we define a reward function (e.g., "move forward = +1 point", "fall over = -100 points") and let the robot figure it out.

## 1. The Pilot Analogy
Think of Sim-to-Real transfer like training a pilot.
*   **The Problem:** You cannot put a novice pilot in a real $100M fighter jet. They will crash it immediately.
*   **The Solution:** You put them in a **Flight Simulator**. They can crash 1,000 times in the simulator without costing a dollar or hurting anyone.
*   **The Transfer:** Once they have mastered the simulator, they step into the real cockpit. Because the simulator was realistic, their muscle memory works in the real world.

In our case:
*   **The Pilot:** The AI Policy (Neural Network).
*   **The Simulator:** Isaac Sim (Gym).
*   **The Jet:** The Physical Humanoid Robot.

## 2. Massively Parallel Training
Isaac Sim (via Isaac Gym/Lab) allows us to simulate *thousands* of robots simultaneously on a single GPU. We don't train one robot for 1,000 hours; we train 1,000 robots for 1 hour. They share their experiences, learning exponentially faster than real-time.

## 3. The Sim-to-Real Workflow

```mermaid
graph LR
    A[Define Reward Function] --> B[Isaac Gym / Isaac Lab]
    B --> C{Training Loop}
    C -->|Crash| B
    C -->|Success| D[Trained Policy .pth]
    D -->|Sim-to-Real Gap| E[Domain Adaptation]
    E --> F[Deploy to Jetson]
    F --> G[Real Robot Walks]
```

### The Sim-to-Real Gap
The "Gap" refers to the subtle differences between simulation and reality (friction, motor backlash, sensor noise). If the gap is too wide, the robot will walk perfectly in sim but fall in real life. We bridge this gap using the **Domain Randomization** techniques discussed in the previous section, making the policy robust enough to handle the "messy" real world.