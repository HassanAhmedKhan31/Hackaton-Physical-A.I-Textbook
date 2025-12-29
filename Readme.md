# 🤖 Physical AI & Humanoid Robotics Textbook

### The World's First AI-Native, Adaptive Technical Textbook

## 📖 Overview

**The Problem:** Traditional engineering textbooks are static. A software engineer needs different analogies than a hardware engineer to understand complex robotics concepts like ROS 2 Nodes.

**The Solution:** This platform uses **Generative AI & User Context** to rewrite the textbook in real-time, matching the reader's background, while providing a RAG-based AI tutor for 24/7 assistance.

---

## 🚀 Key Features

### 1. 👤 Role-Based Personalization
* **Dynamic Content:** The textbook adapts its explanations based on your profile.
    * **Software Engineers:** See concepts explained using Microservices, APIs, and Pub/Sub.
    * **Hardware Engineers:** See concepts explained using Wiring Harnesses, ECUs, and Signals.
* **Tech Stack:** Better-Auth (Session Management) + React Context.

### 2. 🧠 AI Professor (RAG Agent)
* **Context-Aware Q&A:** Ask questions like *"How do I debug a URDF file?"* and the AI answers using **only** the verified textbook content, avoiding hallucinations.
* **Tech Stack:** Python FastAPI, LangChain, Qdrant (Vector DB), OpenAI.

### 3. 🌐 Accessibility & UI
* **Urdu Translation:** Real-time translation to democratize robotics education.
* **Dark/Light Mode:** Fully responsive UI with a toggleable dark mode for comfortable reading.

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
Handles user accounts and stores whether you are a Hardware or Software engineer.

```bash
cd auth-server
npm install
npm run dev
cd chatbot

### Create Virtual Environment (Optional but recommended)
python -m venv venv

### Activate Venv (Windows: venv\Scripts\activate | Mac/Linux: source venv/bin/activate)

pip install -r requirements.txt
uvicorn app:app --reload --port 8000

Step 3: Setup the Frontend (Port 3000)
The main interactive textbook website.

Bash

cd frontend
npm install
npm start

🏃‍♂️ How to Run the Demo
Important: You must have 3 Terminal Windows open, running the commands above simultaneously.

Open the App: Go to http://localhost:3000.

### Test Personalization:
Read Module 1 (Notice the software analogies).

Logout and sign up as a "Hardware Engineer".

Refresh the page. The content changes to hardware concepts!

Test the AI Chatbot:

Click the Green Robot Icon (Bottom Right).

Ask: "What is the difference between a Topic and a Service?"

The AI will answer using the book's context.

📂 Project Structure
Plaintext

my-ai-book/
├── auth-server/               # 🔐 Auth Backend (Node.js/Express)
├── chatbot/                   # 🧠 AI Backend (Python/FastAPI)
│   ├── app.py                 # Main Python server file
│   └── rag_engine.py          # Logic for finding answers
├── frontend/                  # 🖥️ Frontend Website (Docusaurus)
│   ├── docs/                  # Textbook Content (Markdown)
│   ├── src/
│   │   ├── components/        # React Components
│   │   └── pages/             # Home Page
│   └── docusaurus.config.js   # Config file
└── README.md


🤝 Tech Stack Details
Frontend Framework: Docusaurus (React, TypeScript)

Authentication: Better-Auth, Express.js

Database: PostgreSQL (Neon Serverless), Qdrant (Vector Search)

AI/LLM: OpenAI GPT-4o, LangChain

Styling: CSS Modules, Infima

🏆 Hackathon Submission
Event: Physical AI & Humanoid Robotics Hackathon

Created by: Hassan Ahmed Khan

Built with ❤️ for the Robotics Community.
