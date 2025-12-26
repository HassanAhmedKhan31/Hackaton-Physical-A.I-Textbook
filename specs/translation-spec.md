# Translation Feature Specification

## Context
Adding an endpoint to the existing FastAPI chatbot to translate technical robotics content into Urdu using GPT-4o.

## Deliverables

### Update `chatbot/app.py`
- Add endpoint `POST /translate`.
- **Input:** `{ "text": "..." }`
- **Logic:**
  - Call OpenAI GPT-4o-mini.
  - **System Prompt:** "You are a technical translator. Translate the following Robotics text into Urdu. Keep technical terms like 'ROS 2', 'Lidar', 'Node' in English/Roman script where appropriate for clarity."
- **Output:** `{ "translation": "..." }`