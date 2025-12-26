import os
from typing import Optional
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from openai import OpenAI

# Load environment variables
load_dotenv()

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
COLLECTION_NAME = "physical_ai_textbook"

# Initialize Clients
# Check if keys are present to avoid immediate crash on import if .env is missing
if QDRANT_URL and OPENAI_API_KEY:
    qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    openai_client = OpenAI(api_key=OPENAI_API_KEY)
else:
    qdrant_client = None
    openai_client = None

def get_embedding(text: str):
    response = openai_client.embeddings.create(
        input=text,
        model="text-embedding-3-small"
    )
    return response.data[0].embedding

def generate_answer(user_query: str, selected_text: Optional[str] = None) -> str:
    """
    Retrieves context from Qdrant and generates an answer using GPT-4o-mini.
    """
    if not qdrant_client or not openai_client:
        return "System Error: API keys not configured."

    # 1. Convert user query to vector
    query_vector = get_embedding(user_query)

    # 2. Search Qdrant for top 3 relevant chunks
    search_result = qdrant_client.search(
        collection_name=COLLECTION_NAME,
        query_vector=query_vector,
        limit=3
    )

    # 3. Construct Context
    retrieved_context = "\n\n".join([hit.payload['text'] for hit in search_result])
    
    final_context = f"Retrieved Knowledge:\n{retrieved_context}"
    
    if selected_text:
        final_context = f"User Selected Text:\n{selected_text}\n\n{final_context}"

    # 4. Construct Prompt
    system_prompt = (
        "You are a Robotics Professor for the course 'Physical AI & Humanoid Robotics'. "
        "Answer the user's question based ONLY on the provided context below. "
        "If the answer is not in the context, say 'I cannot find that information in the textbook'. "
        "Be concise and educational."
    )

    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": f"Context:\n{final_context}\n\nQuestion: {user_query}"}
    ]

    # 5. Call LLM
    response = openai_client.chat.completions.create(
        model="gpt-4o-mini",
        messages=messages,
        temperature=0.3
    )

    return response.choices[0].message.content

def translate_to_urdu(text: str) -> str:
    """
    Translates the provided text (Markdown or plain) into Urdu using GPT-4o-mini.
    """
    if not openai_client:
        return "System Error: OpenAI API key not configured."

    system_prompt = (
        "You are a technical translator. Translate the following Robotics text into Urdu. "
        "Keep technical terms like 'ROS 2', 'Lidar', 'Node' in English/Roman script where appropriate for clarity."
    )

    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": text}
    ]

    response = openai_client.chat.completions.create(
        model="gpt-4o-mini",
        messages=messages,
        temperature=0.3
    )

    return response.choices[0].message.content