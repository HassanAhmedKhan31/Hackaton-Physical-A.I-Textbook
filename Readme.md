# 🤖 Physical AI & Humanoid Robotics Textbook

### *The World's First AI-Native, Adaptive Technical Textbook*

![Status](https://img.shields.io/badge/Status-Hackathon_MVP-success) ![Stack](https://img.shields.io/badge/Tech-Docusaurus_|_FastAPI_|_Node.js-blue)

> **The Problem:** Traditional engineering textbooks are static. A software engineer needs different analogies than a hardware engineer to understand complex robotics concepts like ROS 2 Nodes.
>
> **The Solution:** This platform uses **Generative AI & User Context** to rewrite the textbook in real-time, matching the reader's background, while providing a RAG-based AI tutor for 24/7 assistance.

---

## 🚀 Key Features

### 1. 👤 Role-Based Personalization
* **Dynamic Content:** The textbook adapts its explanations based on your profile.
    * **Software Engineers:** See concepts explained using *Microservices, APIs, and Pub/Sub*.
    * **Hardware Engineers:** See concepts explained using *Wiring Harnesses, ECUs, and Signals*.
* **Tech Stack:** `Better-Auth` (Session Management) + `React Context`.

### 2. 🧠 AI Professor (RAG Agent)
* **Context-Aware Q&A:** Ask questions like *"How do I debug a URDF file?"* and the AI answers using *only* the verified textbook content, avoiding hallucinations.
* **Tech Stack:** Python `FastAPI`, `LangChain`, `Qdrant` (Vector DB), `OpenAI`.

### 3. 🌐 Accessibility (Urdu Translation)
* **Real-time Translation:** Instantly converts complex technical English into **Urdu** to democratize robotics education.
* **Tech Stack:** Google Translate API / GPT-4o.

---

## 🛠️ System Architecture

This project follows a **Microservices Architecture** running on 3 parallel ports:

| Service | Port | Description |
| :--- | :--- | :--- |
| **Frontend** | `3000` | The Docusaurus User Interface (React/TypeScript). |
| **Auth Server** | `4000` | Node.js Backend handling Signups, Logins, and User Profiles. |
| **AI Backend** | `8000` | Python FastAPI handling Chatbot and Vector Search. |

---

## ⚙️ Installation & Setup

### Prerequisites
* **Node.js** (v18 or higher)
* **Python** (v3.10 or higher)
* **PostgreSQL Database** (Neon DB recommended)
* **OpenAI API Key**

### Step 1: Setup the Auth Server (Port 4000)
*Handles user accounts and stores whether you are a Hardware or Software engineer.*

```bash
cd auth-server
npm install
# Create a .env file with: DATABASE_URL="postgresql://..."
npm run dev
Step 2: Setup the AI Backend (Port 8000)
Handles the Chatbot and Translation features.

Bash

cd chatbot
python -m venv venv
# Activate Venv:
# Windows: venv\Scripts\activate
# Mac/Linux: source venv/bin/activate

pip install -r requirements.txt
# Create a .env file with: OPENAI_API_KEY="sk-..."
uvicorn app:app --reload --port 8000
Step 3: Setup the Frontend (Port 3000)
The main interactive textbook website.

Bash

cd my-ai-book
npm install
npm start
🏃‍♂️ How to Run the Demo
Important: You must have 3 Terminal Windows open, running the commands above simultaneously.

Open the App: Go to http://localhost:3000.

Test Personalization:

Click "Login / Signup" (Top Right).

Create an account as a "Software Engineer".

Read Module 1 (Notice the software analogies).

Logout and sign up as a "Hardware Engineer".

Refresh the page. The content changes to hardware concepts!

Test the AI Chatbot:

Click the Green Robot Icon (Bottom Right).

Ask: "What is the difference between a Topic and a Service?"

The AI will answer using the book's context.

🤝 Tech Stack Details
Frontend Framework: Docusaurus (React, TypeScript)

Authentication: Better-Auth, Express.js

Database: PostgreSQL (Neon Serverless), Qdrant (Vector Search)

AI/LLM: OpenAI GPT-4o, LangChain

Styling: CSS Modules, Infima

🏆 Hackathon Submission
Event: Physical AI & Humanoid Robotics Hackathon

Created by: Hassan Ahmed Khan