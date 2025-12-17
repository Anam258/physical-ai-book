from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from agent import run_agent
from dotenv import load_dotenv
from datetime import datetime
import traceback


load_dotenv()

# Utility function for structured logging
def log_timestamp():
    return datetime.utcnow().isoformat() + "Z"
# Initialize App
app = FastAPI(title="Physical AI Tutor API")

# Setup CORS 
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Hackathon ke liye '*' safe hai
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request Model (Input format)
class ChatRequest(BaseModel):
    question: str

class TranslateRequest(BaseModel):
    text: str

# PersonalizeRequest Pydantic model removed to allow flexible JSON keys (role, background, userRole)
# Frontend sends various key names - see research.md for decision rationale
# class PersonalizeRequest(BaseModel):
#     topic: str
#     role: str

@app.get("/")
def home():
    return {"message": "Physical AI Tutor API is Running! 🤖"}

@app.post("/chat")
async def chat_endpoint(request: Request): # 'ChatRequest' hata kar 'Request' kar diya
    timestamp = log_timestamp()
    try:
        data = await request.json()
        print(f"[{timestamp}] INFO /chat [received] Body: {data}")

        question = data.get("question") or data.get("message")
        
        if not question:
            print(f"[{timestamp}] WARNING /chat - No question found in request")
            return {"answer": "I didn't receive a question. Please try again!"}

        print(f"[{timestamp}] INFO /chat [calling_agent] Query: {question[:100]}...")
        response = await run_agent(question)
        
        result = {
            "answer": response,
            "response": response,
            "content": response,
            "status": "success"
        }
        
        print(f"[{timestamp}] INFO /chat [responding] Success!")
        return result

    except Exception as e:
        print(f"[{timestamp}] ERROR /chat [error] {str(e)}")
        print(f"[{timestamp}] ERROR /chat [traceback] {traceback.format_exc()}")
        # Error aane par frontend crash na ho, isliye friendly message bhejien
        return {
            "answer": "❌ Error: My brain is a bit foggy on the server. Please check Render logs.",
            "error": str(e)
        }
    
@app.post("/translate")
async def translate_endpoint(request: TranslateRequest):
    print(f"📩 Received Translation Request: {request.text[:50]}...")

    try:
        # Format prompt with translation instruction
        query = f"Translate this technical text to formal Urdu, keeping English keywords: {request.text}"
        # Call existing agent
        response = await run_agent(query)
        return {"translation": response}
    except Exception as e:
        print(f"❌ Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/personalize")
async def personalize_endpoint(request: Request):
    try:
        # Get raw JSON
        data = await request.json()

        # Log for debugging
        print(f"📩 Received Personalization Request: {data}")

        # Extract topic (required)
        topic = data.get("topic")
        
        # Role with fallback logic
        role = data.get("role") or data.get("background") or data.get("userRole") or "student"

        if not topic:
            raise HTTPException(status_code=422, detail="Missing required field: topic")

        print(f"📝 Extracted - Topic: '{topic}', Role: '{role}'")

        # Format prompt
        prompt = f"Explain {topic} for someone with {role} background in 2-3 sentences using relevant analogies."
        
        response = await run_agent(prompt)
        
        return {
            "analogy": response,
            "explanation": response,  
            "response": response,     
            "content": response,      
            "answer": response        
        }

    except Exception as e:
        print(f"❌ Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))
if __name__ == "__main__":
    import uvicorn
    import os
    port = int(os.environ.get("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)