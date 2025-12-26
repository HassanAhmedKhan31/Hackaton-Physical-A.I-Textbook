# The Cognitive Core (GPT Integration)

Now that the robot can hear "Move forward," how does it know *how* to move? It needs a brain to translate natural language into robot instructions. We use a **Large Language Model (LLM)** like GPT-4.

## 1. The Challenge: Unstructured Text
If you ask ChatGPT "Move forward," it might reply:
> "Sure! I'd be happy to help you with that. I am initiating my motors now to proceed in a forward direction."

This is **useless** to a robot. A robot needs numbers, velocities, and booleans. It cannot parse a polite conversation.

## 2. Prompt Engineering for Robotics
We must force the LLM to output **Structured Data**. The best format for this is JSON.

### The System Prompt
We define the robot's "Persona" and "Output Format" in the System Prompt.

**Bad Prompt:**
> "You are a robot helper. Answer the user's questions."

**Good Prompt:**
> "You are a humanoid robot controller. You receive natural language commands. You MUST reply with a valid JSON object containing the field 'action' and relevant parameters. Do NOT output markdown, conversational filler, or explanations.
>
> Supported Actions:
> - 'move': requires 'linear_x' (meters) and 'angular_z' (radians).
> - 'stop': no parameters.
> - 'speak': requires 'text' (string)."

## 3. Practical: Parsing JSON
When the user says "Go forward a little bit," the LLM should generate:

```json
{
  "action": "move",
  "linear_x": 0.5,
  "angular_z": 0.0
}
```

### The Code
Our ROS node receives this response and parses it:

```python
import json

response_text = call_gpt4(user_prompt)

try:
    command = json.loads(response_text)
    
    if command["action"] == "move":
        # Create Twist message
        msg = Twist()
        msg.linear.x = command["linear_x"]
        msg.angular.z = command["angular_z"]
        publisher.publish(msg)
        
except json.JSONDecodeError:
    print("Error: LLM returned invalid JSON.")
```

By strictly enforcing JSON output, we bridge the gap between the vague world of language and the precise world of control theory.