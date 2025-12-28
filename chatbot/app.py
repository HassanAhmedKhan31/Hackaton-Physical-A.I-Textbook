import os
import psycopg2
from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
from rag_engine import generate_answer, translate_to_urdu

# Load environment variables
load_dotenv()

app = FastAPI()

# Add CORS Middleware (Crucial for your React Frontend to talk to this)
# In app.py
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # ✅ Change this to "*" to allow any connection
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Database Configuration
NEON_DB_URL = os.getenv("NEON_DB_URL")

# Request Models
class ChatRequest(BaseModel):
    message: str
    selected_text: Optional[str] = None

class TranslateRequest(BaseModel):
    text: str

# Initialize Database Table
def init_db():
    if not NEON_DB_URL:
        print("Warning: NEON_DB_URL not set. Database logging disabled.")
        return

    try:
        conn = psycopg2.connect(NEON_DB_URL)
        cursor = conn.cursor()
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS chat_history (
                id SERIAL PRIMARY KEY,
                user_message TEXT NOT NULL,
                selected_text TEXT,
                bot_response TEXT NOT NULL,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
        """)
        conn.commit()
        cursor.close()
        conn.close()
        print("Database initialized successfully.")
    except Exception as e:
        print(f"Database initialization failed: {e}")

# Run DB init on startup
init_db()

def log_conversation(user_msg, selected, bot_resp):
    if not NEON_DB_URL:
        return
    try:
        conn = psycopg2.connect(NEON_DB_URL)
        cursor = conn.cursor()
        cursor.execute(
            "INSERT INTO chat_history (user_message, selected_text, bot_response) VALUES (%s, %s, %s)",
            (user_msg, selected, bot_resp)
        )
        conn.commit()
        cursor.close()
        conn.close()
    except Exception as e:
        print(f"Failed to log conversation: {e}")

@app.post("/chat")
async def chat_endpoint(request: ChatRequest):
    if not request.message:
        raise HTTPException(status_code=400, detail="Message cannot be empty")
    
    # Generate Answer
    response_text = generate_answer(request.message, request.selected_text)
    
    # Log to DB
    log_conversation(request.message, request.selected_text, response_text)
    
    return {"response": response_text}

@app.post("/translate")
async def translate_endpoint(request: TranslateRequest):
    if not request.text:
        raise HTTPException(status_code=400, detail="Text to translate cannot be empty")
    
    # Generate Translation
    translation = translate_to_urdu(request.text)
    
    return {"translation": translation}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)