import os
import glob
import uuid
from typing import List
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
from openai import OpenAI
from langchain.text_splitter import RecursiveCharacterTextSplitter

# Load environment variables
load_dotenv()

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
COLLECTION_NAME = "physical_ai_textbook"
DOCS_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "docs")

# Initialize Clients
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
openai_client = OpenAI(api_key=OPENAI_API_KEY)

def get_embedding(text: str) -> List[float]:
    """Generates embedding using text-embedding-3-small."""
    response = openai_client.embeddings.create(
        input=text,
        model="text-embedding-3-small"
    )
    return response.data[0].embedding

def ingest_docs():
    print(f"Scanning for documents in {DOCS_DIR}...")
    
    # Check if collection exists, recreate if needed (for fresh ingest)
    collections = qdrant_client.get_collections()
    collection_names = [c.name for c in collections.collections]
    
    if COLLECTION_NAME not in collection_names:
        print(f"Creating collection: {COLLECTION_NAME}")
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
        )
    
    # Text splitter configuration
    splitter = RecursiveCharacterTextSplitter(
        chunk_size=1000,
        chunk_overlap=100,
        separators=["\n\n", "\n", " ", ""]
    )

    # Walk through modules
    md_files = glob.glob(os.path.join(DOCS_DIR, "**", "*.md"), recursive=True)
    
    points = []
    
    for file_path in md_files:
        print(f"Processing: {file_path}")
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()
            
        chunks = splitter.split_text(content)
        
        for i, chunk in enumerate(chunks):
            vector = get_embedding(chunk)
            payload = {
                "source": os.path.basename(file_path),
                "text": chunk,
                "chunk_index": i
            }
            
            points.append(models.PointStruct(
                id=str(uuid.uuid4()),
                vector=vector,
                payload=payload
            ))
            
    # Upsert to Qdrant
    if points:
        print(f"Upserting {len(points)} chunks to Qdrant...")
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )
        print("Ingestion complete!")
    else:
        print("No documents found to ingest.")

if __name__ == "__main__":
    if not OPENAI_API_KEY or not QDRANT_URL:
        print("Error: Missing API Keys in .env file.")
    else:
        ingest_docs()