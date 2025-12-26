# Chatbot Specification: The "Professor" RAG Agent

## Context
We are building a RAG (Retrieval-Augmented Generation) chatbot for the "Physical AI & Humanoid Robotics" textbook. The chatbot serves as a teaching assistant.

## Tech Stack (Strict Requirements)
- **Framework:** FastAPI (Python).
- **Vector DB:** Qdrant Cloud (Free Tier).
- **Database:** Neon Serverless Postgres (for chat history).
- **LLM:** OpenAI GPT-4o-mini (via `openai` SDK).
- **Documentation Source:** All Markdown files in the `docs/` folder.

## Deliverables

### 1. `chatbot/ingest.py` (The Learner)
- **Goal:** Read all `.md` files in `docs/`, split them into chunks, and upload them to Qdrant.
- **Logic:**
  1. Iterate through `docs/module-1` to `docs/module-6`.
  2. Use `RecursiveCharacterTextSplitter` (chunk size ~1000 chars).
  3. Generate embeddings using `text-embedding-3-small`.
  4. Upsert vectors to Qdrant collection named `physical_ai_textbook`.

### 2. `chatbot/rag_engine.py` (The Brain)
- **Goal:** Search the book and generate answers.
- **Function:** `generate_answer(user_query: str)`
- **Logic:**
  1. Convert user query to vector.
  2. Search Qdrant for top 3 relevant chunks.
  3. Construct a prompt: "You are a Robotics Professor. Answer based ONLY on this context: {context}".
  4. Return the answer.

### 3. `chatbot/app.py` (The API)
- **Goal:** Expose the brain to the web.
- **Endpoint:** `POST /chat`
- **Input:** JSON `{ "message": "How do I calibrate a magnetometer?" }`
- **Output:** JSON `{ "response": "To calibrate the BNO055..." }`
- **Database:** Save the conversation log to Neon Postgres using `psycopg2`.

## Bonus Feature: "Select-to-Ask"
- The API must handle an optional parameter: `selected_text`.
- If provided, the context is `selected_text` + retrieved chunks.