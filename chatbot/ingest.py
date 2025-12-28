import os
import glob
import uuid
from typing import List
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
from langchain_text_splitters import RecursiveCharacterTextSplitter
from sentence_transformers import SentenceTransformer # ✅ Local Embeddings

load_dotenv()

COLLECTION_NAME = "physical_ai_textbook"
DOCS_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "docs")

# Initialize Local Embedding Model (Free, runs on CPU)
embedding_model = SentenceTransformer('all-MiniLM-L6-v2') 

# Initialize Local Database
qdrant_client = QdrantClient(path="local_qdrant_index")

def get_embedding(text: str) -> List[float]:
    # ✅ Converts text to numbers using your computer, not API
    return embedding_model.encode(text).tolist()

def ingest_docs():
    print(f"Scanning for documents in {DOCS_DIR}...")
    
    # Check/Create collection
    collections = qdrant_client.get_collections()
    collection_names = [c.name for c in collections.collections]
    
    if COLLECTION_NAME not in collection_names:
        print(f"Creating collection: {COLLECTION_NAME}")
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            # ✅ SIZE 384 is for MiniLM model
            vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE),
        )
    
    splitter = RecursiveCharacterTextSplitter(chunk_size=1000, chunk_overlap=100)
    md_files = glob.glob(os.path.join(DOCS_DIR, "**", "*.md"), recursive=True)
    points = []
    
    for file_path in md_files:
        print(f"Processing: {file_path}")
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()
            chunks = splitter.split_text(content)
            for i, chunk in enumerate(chunks):
                vector = get_embedding(chunk)
                points.append(models.PointStruct(
                    id=str(uuid.uuid4()),
                    vector=vector,
                    payload={"source": os.path.basename(file_path), "text": chunk}
                ))
        except Exception as e:
            print(f"Error reading {file_path}: {e}")
            
    if points:
        print(f"Upserting {len(points)} chunks...")
        qdrant_client.upsert(collection_name=COLLECTION_NAME, points=points)
        print("Ingestion complete!")

if __name__ == "__main__":
    ingest_docs()