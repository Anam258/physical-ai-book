import os
import requests
import xml.etree.ElementTree as ET
import trafilatura
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
from dotenv import load_dotenv

# Load Environment Variables (Keys)
load_dotenv()

# -------------------------------------
# CONFIG
# -------------------------------------
SITEMAP_URL = "https://anam258.github.io/physical-ai-book/sitemap.xml"
COLLECTION_NAME = "humanoid_ai_book"
EMBED_MODEL = "embed-english-v3.0"

# Get Keys from .env
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

# Initialize Clients
cohere_client = cohere.Client(COHERE_API_KEY)
qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY, timeout=60)

# -------------------------------------
# Step 1 — Extract URLs from sitemap
# -------------------------------------
def get_all_urls(sitemap_url):
    print(f"Fetching sitemap from: {sitemap_url}")
    try:
        xml = requests.get(sitemap_url).text
        root = ET.fromstring(xml)
    except Exception as e:
        print(f"Error fetching sitemap: {e}")
        return []

    urls = []
    for child in root:
        loc_tag = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
        if loc_tag is not None:
            urls.append(loc_tag.text)

    print(f"\nFOUND {len(urls)} URLs:")
    return urls

# -------------------------------------
# Step 2 — Download page + extract text
# -------------------------------------
def extract_text_from_url(url):
    try:
        html = requests.get(url).text
        text = trafilatura.extract(html)
        if not text:
            print("[WARNING] No text extracted from:", url)
        return text
    except Exception as e:
        print(f"Error extracting {url}: {e}")
        return None

# -------------------------------------
# Step 3 — Chunk the text 
# -------------------------------------
def chunk_text(text, max_chars=1000):
    chunks = []
    while len(text) > max_chars:
        # Find the last period to split nicely
        split_pos = text[:max_chars].rfind(". ")
        
        if split_pos <= 0:
            split_pos = max_chars  # Force split kar do
            
        chunks.append(text[:split_pos].strip())
        text = text[split_pos:].strip() # <-- Yeh .strip() loop ko todega
    
    if text:
        chunks.append(text.strip())
    return chunks
# -------------------------------------
# Step 4 — Create embedding
# -------------------------------------
def embed(text):
    response = cohere_client.embed(
        model=EMBED_MODEL,
        input_type="search_query",
        texts=[text],
    )
    return response.embeddings[0]

# -------------------------------------
# Step 5 — Store in Qdrant
# -------------------------------------
def create_collection():
    print(f"\nCreating Qdrant collection: {COLLECTION_NAME}...")
    qdrant.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
            size=1024, # Cohere v3 dimension
            distance=Distance.COSINE
        )
    )

def save_chunk_to_qdrant(chunk, chunk_id, url):
    vector = embed(chunk)
    qdrant.upsert(
        collection_name=COLLECTION_NAME,
        points=[
            PointStruct(
                id=chunk_id,
                vector=vector,
                payload={
                    "url": url,
                    "text": chunk,
                    "chunk_id": chunk_id
                }
            )
        ]
    )

# -------------------------------------
# MAIN INGESTION PIPELINE
# -------------------------------------
def ingest_book():
    urls = get_all_urls(SITEMAP_URL)
    if not urls:
        print("No URLs found. Exiting.")
        return

    create_collection()

    global_id = 1
    for url in urls:
        print(f"\nProcessing: {url}")
        text = extract_text_from_url(url)

        if not text:
            continue

        chunks = chunk_text(text)
        for ch in chunks:
            save_chunk_to_qdrant(ch, global_id, url)
            print(f"Saved chunk {global_id}", end="\r")
            global_id += 1

    print(f"\n\n✔️ Ingestion completed! Total chunks: {global_id - 1}")

if __name__ == "__main__":
    ingest_book()