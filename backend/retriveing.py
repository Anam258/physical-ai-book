import os
import cohere
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load keys from .env
load_dotenv()

# Config
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
COLLECTION_NAME = "humanoid_ai_book"  # <--- FIXED NAME (Matches ingest.py)

# Clients
cohere_client = cohere.Client(COHERE_API_KEY)
qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

def get_embedding(text):
    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",
        texts=[text],
    )
    return response.embeddings[0]

def retrieve(query):
    """
    Searches the Qdrant database for relevant text chunks.
    """
    print(f"Searching for: {query}...") # Debugging ke liye
    embedding = get_embedding(query)
    
    result = qdrant.query_points(
        collection_name=COLLECTION_NAME,
        query=embedding,
        limit=5
    )
    
    # Text wapis return karo
    found_texts = [point.payload["text"] for point in result.points]
    return "\n\n".join(found_texts)

# Test run
if __name__ == "__main__":
    print(retrieve("What is Physical AI?"))