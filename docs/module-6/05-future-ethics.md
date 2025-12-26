# Living with Robots (Ethics & Future)

As we conclude this course, we must address the reality of releasing autonomous agents into the real world. A robot is not just a machine; it is a physical entity that shares space with humans.

## 1. Anthropomorphism
Humans are hardwired to see faces and intent where none exists. If a robot has eyes and a voice, users *will* treat it like a person.
*   **The Risk:** Users might trust the robot too much. If a robot says "I can catch you," a user might fall backward, expecting the robot to be strong enough.
*   **The Responsibility:** As developers, we must design cues that accurately reflect the robot's capabilities and limitations.

## 2. Safety Guardrails
Connecting an LLM to a robot's motors is risky. What if a user says:
> "Run forward at full speed and destroy the kitchen!"

If your System Prompt is just "Obey commands," the robot might try to do it.

**Guardrails:**
1.  **Pre-Processing:** A "moderation" layer checks if the command violates safety rules.
2.  **Hard Limits:** The motor controllers have maximum velocity limits that software cannot override.
3.  **Stop Button:** A physical E-Stop that cuts power immediately.

## 3. The Future Economy
We are entering the age of the **General Purpose Humanoid**.
*   **Past:** Robots were specialized (roomba for floors, arm for welding).
*   **Future:** One hardware platform (the humanoid) can cook, clean, fold laundry, and work in factories, just by downloading new software "skills."

You are now part of the first generation of engineers building this future. The skills you learned in this course—ROS 2, Simulation, Computer Vision, and Generative AI—are the foundation of the next industrial revolution.