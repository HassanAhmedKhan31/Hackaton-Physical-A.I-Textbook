# Module 6 Specification: Conversational Robotics

## Context
This is the final module. We are integrating "Generative AI" into the "Physical AI" stack. The goal is to move from buttons and joysticks to natural language voice commands.

## Target Audience
Students who have a working robot (simulated or real) and now want to talk to it.

## Design Constraints
- **APIs:** Use OpenAI API (Whisper for Audio, GPT-4o for Logic).
- **Latency:** Explicitly discuss the "Latency Trap" (Cloud API delay vs. Real-time robot needs).
- **Safety:** How to prevent the LLM from hallucinating dangerous commands.

## Deliverables

### 1. `docs/module-6/01-intro-conversational-robotics.md`
- **Title**: The Voice Interface (VUI)
- **Concept**: Why voice is the natural interface for humanoids.
- **Architecture**:
  - Audio Input (Mic) -> STT (Whisper) -> LLM (GPT) -> TTS (Text-to-Speech) -> Audio Output (Speaker).
- **Diagram**: A Mermaid flowchart showing this pipeline integrated with ROS 2.

### 2. `docs/module-6/02-hearing-with-whisper.md`
- **Title**: The Robot's Ears (Speech Recognition)
- **Concept**: Converting sound waves to text.
- **Technology**: OpenAI Whisper (Cloud vs. Local).
- **Practical**:
  - Python script to record 5 seconds of audio.
  - Sending audio to Whisper API.
  - **ROS 2 Integration**: Creating a `SpeechNode` that publishes the transcribed text to a topic `/human_command`.

### 3. `docs/module-6/03-thinking-with-llms.md`
- **Title**: The Cognitive Core (GPT Integration)
- **Concept**: Giving the robot a "Persona."
- **Prompt Engineering**: Writing a System Prompt specifically for robotics.
  - *Bad Prompt:* "You are a helper."
  - *Good Prompt:* "You are a robot assistant. If asked to move, output JSON coordinates. Do not use flowery language."
- **Practical**: A Python script that subscribes to `/human_command`, sends it to GPT-4, and parses the response.

### 4. `docs/module-6/04-multimodal-interaction.md`
- **Title**: Vision-Language-Action (VLA)
- **Concept**: Combining what the robot *sees* with what it *hears*.
- **Scenario**: User points at a cup and says "Pick that up." The robot needs vision to know what "that" is.
- **Technique**: Feeding the image frame from the robot's camera into GPT-4o (Vision) alongside the user's text prompt.

### 5. `docs/module-6/05-future-ethics.md`
- **Title**: Living with Robots (Ethics & Future)
- **Concept**: As robots become "social," humans form emotional bonds.
- **Topics**:
  - **Anthropomorphism**: The tendency to treat robots like people.
  - **Safety**: What if someone tells the robot to "Destroy the kitchen"? (Guardrails).
  - **The Future**: Autonomous humanoid economy.