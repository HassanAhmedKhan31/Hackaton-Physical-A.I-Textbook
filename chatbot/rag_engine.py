import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from openai import OpenAI
from sentence_transformers import SentenceTransformer 

load_dotenv()

# Setup Clients
# We wrap Qdrant in a try/except so the app doesn't crash if it fails
try:
    qdrant_client = QdrantClient(path="local_qdrant_index")
    embedding_model = SentenceTransformer('all-MiniLM-L6-v2') 
    db_active = True
except Exception as e:
    print(f"⚠️ Warning: Database not working. Switching to AI-only mode. ({e})")
    db_active = False

client = OpenAI(
    base_url="https://openrouter.ai/api/v1", 
    api_key=os.getenv("OPENROUTER_API_KEY")
)

COLLECTION_NAME = "physical_ai_textbook"

def generate_answer(query: str):
    context = ""
    
    # 1. Try to get context from the book (if DB is active)
    if db_active:
        try:
            vector = embedding_model.encode(query).tolist()
            search_result = qdrant_client.search(
                collection_name=COLLECTION_NAME,
                query_vector=vector,
                limit=3
            )
            retrieved_text = "\n".join([hit.payload['text'] for hit in search_result])
            context = f"Use this textbook context if relevant:\n{retrieved_text}\n\n"
        except Exception as e:
            print(f"Search failed: {e}")
            # We ignore the error and proceed without context

    # 2. Ask the AI (Context + Query)
    try:
        response = client.chat.completions.create(
            model="xiaomi/mimo-v2-flash:free", # or your preferred model
            messages=[
                {"role": "system", "content": "You are a helpful AI Professor. Answer the student clearly."},
                {"role": "user", "content": context + f"Question: {query}"}
            ]
        )
        return response.choices[0].message.content
    except Exception as e:
        return f"AI Error: {str(e)}"

def translate_to_urdu(text: str):
    # (Keep your translation code here)
    return text