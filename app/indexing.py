# Content indexing script - reads markdown files, chunks them, and uploads to Qdrant
import os
import sys
from pathlib import Path
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from dotenv import load_dotenv
import hashlib

# Load environment variables
load_dotenv()

# Initialize clients - REPLACE placeholders in .env with real API keys
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "sk-....")
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

if OPENAI_API_KEY == "sk-REPLACE_ME" or OPENAI_API_KEY == "":
    print("❌ ERROR: OPENAI_API_KEY not set in .env file")
    print("Please edit .env and add your OpenAI API key from https://platform.openai.com/api-keys")
    sys.exit(1)

try:
    openai_client = OpenAI(api_key=OPENAI_API_KEY)
    qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
except Exception as e:
    print(f"❌ ERROR: Failed to initialize clients: {str(e)}")
    print("Please check your .env configuration")
    sys.exit(1)

COLLECTION_NAME = "panaversity_docs"
EMBEDDING_MODEL = "text-embedding-3-small"
EMBEDDING_DIMENSION = 1536

def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> list[str]:
    """Split text into overlapping chunks"""
    words = text.split()
    chunks = []
    
    for i in range(0, len(words), chunk_size - overlap):
        chunk = " ".join(words[i:i + chunk_size])
        if chunk.strip():
            chunks.append(chunk)
    
    return chunks

def read_markdown_files(docs_dir: str) -> list[dict]:
    """Read all markdown files from docs directory"""
    docs_path = Path(docs_dir)
    documents = []
    
    for md_file in docs_path.rglob("*.md"):
        # Skip Urdu files for now
        if ".ur.md" in str(md_file):
            continue
            
        with open(md_file, 'r', encoding='utf-8') as f:
            content = f.read()
            
        # Remove frontmatter
        if content.startswith("---"):
            parts = content.split("---", 2)
            if len(parts) >= 3:
                content = parts[2].strip()
        
        documents.append({
            "content": content,
            "source": str(md_file.relative_to(docs_path.parent))
        })
    
    return documents

def create_collection():
    """Create Qdrant collection if it doesn't exist"""
    try:
        qdrant_client.get_collection(COLLECTION_NAME)
        print(f"✓ Collection '{COLLECTION_NAME}' already exists")
    except:
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(size=EMBEDDING_DIMENSION, distance=Distance.COSINE)
        )
        print(f"✓ Created collection '{COLLECTION_NAME}'")

def index_documents(documents: list[dict]):
    """Index documents into Qdrant with embeddings"""
    print(f"\n📚 Indexing {len(documents)} documents...")
    
    all_points = []
    point_id = 0
    
    for doc in documents:
        print(f"\n📄 Processing: {doc['source']}")
        
        # Chunk the document
        chunks = chunk_text(doc['content'])
        print(f"  → Created {len(chunks)} chunks")
        
        for i, chunk in enumerate(chunks):
            try:
                # Generate embedding
                response = openai_client.embeddings.create(
                    model=EMBEDDING_MODEL,
                    input=chunk
                )
                embedding = response.data[0].embedding
                
                # Create point for Qdrant
                point = PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "text": chunk,
                        "source": doc['source'],
                        "chunk_index": i
                    }
                )
                all_points.append(point)
                point_id += 1
                
                if (i + 1) % 5 == 0:
                    print(f"  → Processed {i + 1}/{len(chunks)} chunks")
                    
            except Exception as e:
                print(f"  ❌ Error processing chunk {i}: {str(e)}")
                continue
    
    # Upload to Qdrant in batches
    print(f"\n⬆️  Uploading {len(all_points)} vectors to Qdrant...")
    batch_size = 100
    for i in range(0, len(all_points), batch_size):
        batch = all_points[i:i + batch_size]
        qdrant_client.upsert(collection_name=COLLECTION_NAME, points=batch)
        print(f"  → Uploaded batch {i//batch_size + 1}/{(len(all_points) + batch_size - 1)//batch_size}")
    
    print(f"\n✅ Successfully indexed {len(all_points)} chunks!")

def main():
    print("🚀 Starting content indexing for Panaversity Hackathon")
    print("=" * 60)
    
    # Find docs directory
    docs_dir = Path(__file__).parent.parent / "website" / "docs"
    
    if not docs_dir.exists():
        print(f"❌ ERROR: Docs directory not found at {docs_dir}")
        sys.exit(1)
    
    print(f"📁 Docs directory: {docs_dir}")
    
    # Create collection
    create_collection()
    
    # Read documents
    documents = read_markdown_files(str(docs_dir))
    
    if not documents:
        print("❌ No markdown files found")
        sys.exit(1)
    
    # Index documents
    index_documents(documents)
    
    print("\n" + "=" * 60)
    print("✅ Indexing complete!")

if __name__ == "__main__":
    main()