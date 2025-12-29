from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
# Import the brain functions we fixed earlier
from rag_engine import generate_answer

app = FastAPI()

# ✅ FIX: Allow the website (localhost:3000) to talk to this server
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allows all connections (Safety off for hackathon)
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class Query(BaseModel):
    text: str

@app.get("/")
def home():
    return {"status": "Active", "message": "Brain is running!"}

@app.post("/chat")
async def chat(query: Query):
    try:
        print(f"📩 Received: {query.text}") # Debug print
        response = generate_answer(query.text)
        print(f"📤 Sending: {response}")    # Debug print
        return {"reply": response}
    except Exception as e:
        print(f"❌ Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))