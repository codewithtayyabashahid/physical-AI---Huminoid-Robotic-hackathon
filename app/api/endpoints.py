
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional
import os
from openai import OpenAI
from qdrant_client import QdrantClient

router = APIRouter()

# Initialize clients with environment variables
# Replace these placeholders with real credentials in your .env file
openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY", "sk-...."))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL", "http://localhost:6333"),
    api_key=os.getenv("QDRANT_API_KEY")
)

COLLECTION_NAME = "panaversity_docs"

# Request/Response models
class QueryRequest(BaseModel):
    question: str

class SelectionQueryRequest(BaseModel):
    question: str
    selected_text: str

class QueryResponse(BaseModel):
    answer: str
    sources: Optional[list] = []

class TranslationResponse(BaseModel):
    content: str

class SignupRequest(BaseModel):
    email: str
    password: str
    name: str

class LoginRequest(BaseModel):
    email: str
    password: str

class PersonalizeRequest(BaseModel):
    learningLevel: str
    interests: list

@router.post("/query", response_model=QueryResponse)
async def query_book(request: QueryRequest):
    """
    Query the entire book using RAG - searches Qdrant for relevant chunks
    and uses OpenAI to generate answer
    """
    if not request.question.strip():
        raise HTTPException(status_code=400, detail="Question cannot be empty")
    
    try:
        # Generate embedding for the question
        embedding_response = openai_client.embeddings.create(
            model="text-embedding-3-small",
            input=request.question
        )
        question_embedding = embedding_response.data[0].embedding
        
        # Search Qdrant for relevant chunks
        search_results = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=question_embedding,
            limit=5
        )
        
        # Extract context from search results
        context_chunks = []
        sources = []
        for result in search_results:
            context_chunks.append(result.payload.get("text", ""))
            sources.append(result.payload.get("source", ""))
        
        context = "\n\n".join(context_chunks)
        
        # Generate answer using OpenAI
        completion = openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": "You are an expert in Physical AI and Humanoid Robotics. Answer questions based on the provided context from the book."},
                {"role": "user", "content": f"Context:\n{context}\n\nQuestion: {request.question}\n\nProvide a detailed answer based on the context above."}
            ],
            temperature=0.7,
            max_tokens=500
        )
        
        answer = completion.choices[0].message.content
        
        return QueryResponse(answer=answer, sources=list(set(sources)))
        
    except Exception as e:
        print(f"Error in query_book: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Failed to process query: {str(e)}")

@router.post("/query/selection", response_model=QueryResponse)
async def query_selection(request: SelectionQueryRequest):
    """
    Answer question about ONLY the selected text - does not search external docs.
    This ensures the answer is strictly based on what the user highlighted.
    """
    if not request.question.strip():
        raise HTTPException(status_code=400, detail="Question cannot be empty")
    
    if not request.selected_text.strip():
        raise HTTPException(status_code=400, detail="Selected text cannot be empty")
    
    try:
        # Generate answer using ONLY the selected text as context
        completion = openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": "You are an expert in Physical AI and Humanoid Robotics. Answer questions based ONLY on the selected text provided. Do not use external knowledge."},
                {"role": "user", "content": f"Selected Text:\n{request.selected_text}\n\nQuestion: {request.question}\n\nAnswer based ONLY on the selected text above."}
            ],
            temperature=0.7,
            max_tokens=300
        )
        
        answer = completion.choices[0].message.content
        
        return QueryResponse(answer=answer, sources=["Selected Text"])
        
    except Exception as e:
        print(f"Error in query_selection: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Failed to process selection query: {str(e)}")

@router.get("/translate/urdu", response_model=TranslationResponse)
async def translate_to_urdu(chapter: str = "01-physical-ai"):
    """
    Fetch Urdu translation of a chapter - in production, reads from .ur.md files
    """
    try:
        # In a real implementation, read the .ur.md file corresponding to the chapter
        # For demo purposes, returning placeholder
        urdu_file_path = f"../website/docs/chapters/{chapter}.ur.md"
        
        if os.path.exists(urdu_file_path):
            with open(urdu_file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            return TranslationResponse(content=content)
        else:
            # Fallback demo content
            return TranslationResponse(
                content="یہ ایک مثال ہے۔ فزیکل اے آئی مصنوعی ذہانت اور روبوٹکس کا امتزاج ہے۔"
            )
            
    except Exception as e:
        print(f"Error in translate_to_urdu: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to load Urdu content")

@router.post("/signup")
async def signup(request: SignupRequest):
    """User signup endpoint - stores credentials in PostgreSQL"""
    # TODO: Implement actual user creation with password hashing
    # For demo, returning success
    return {"message": "User created successfully", "email": request.email}

@router.post("/login")
async def login(request: LoginRequest):
    """User login endpoint - validates credentials and returns JWT token"""
    # TODO: Implement actual authentication with JWT
    # For demo, returning mock token
    return {"token": "mock_jwt_token", "email": request.email}

@router.post("/personalize")
async def personalize(request: PersonalizeRequest):
    """Save user personalization preferences"""
    # TODO: Store preferences in database
    # For demo, returning success
    return {"message": "Preferences saved", "preferences": request.dict()}