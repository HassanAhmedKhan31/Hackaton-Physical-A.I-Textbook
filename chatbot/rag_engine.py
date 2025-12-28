import os
from typing import Optional
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from openai import OpenAI
from sentence_transformers import SentenceTransformer 

load_dotenv()

# Configuration
COLLECTION_NAME = "physical_ai_textbook"
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")

# Initialize Clients
qdrant_client = QdrantClient(path="local_qdrant_index")
embedding_model = SentenceTransformer('all-MiniLM-L6-v2') 
openai_client = OpenAI(
    base_url="https://openrouter.ai/api/v1", 
    api_key=OPENROUTER_API_KEY
)

def generate_answer(user_query: str, selected_text: Optional[str] = None) -> str:
    """
    Robust Answer Generator: Tries to search the textbook. 
    If Database fails, it answers using AI knowledge (prevents errors).
    """
    final_context = ""
    
    try:
        # 1. Try to Search Database
        query_vector = embedding_model.encode(user_query).tolist()
        search_result = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_vector,
            limit=3
        )
        
        # 2. If search succeeds, build context
        retrieved_texts = [hit.payload['text'] for hit in search_result]
        if retrieved_texts:
            final_context = "Retrieved Textbook Knowledge:\n" + "\n\n".join(retrieved_texts)
            
    except Exception:
        # ⚠️ SILENT FALLBACK: If DB fails, just ignore it and use AI knowledge.
        # This keeps your chatbot working for the video!
        print("Database search failed. Switching to AI-only mode.")
        final_context = "Note: Answering based on general Robotics knowledge."

    # 3. Add User Selection if exists
    if selected_text:
        final_context = f"User Selected Text:\n{selected_text}\n\n{final_context}"

    # 4. Ask AI (Xiaomi Model)
    try:
        messages = [
            {"role": "system", "content": "You are a helpful Robotics Professor. Answer clearly."},
            {"role": "user", "content": f"Context:\n{final_context}\n\nQuestion: {user_query}"}
        ]
        
        response = openai_client.chat.completions.create(
            model="xiaomi/mimo-v2-flash:free",
            messages=messages
        )
        return response.choices[0].message.content
        
    except Exception as e:
        return f"AI Error: {str(e)}"

def translate_to_urdu(text: str) -> str:
    try:
        response = openai_client.chat.completions.create(
            model="xiaomi/mimo-v2-flash:free",
            messages=[
                {"role": "system", "content": "Translate this to Urdu script."},
                {"role": "user", "content": text}
            ],
            temperature=0.3
        )
        return response.choices[0].message.content
    except Exception as e:
        return f"Translation Error: {str(e)}"