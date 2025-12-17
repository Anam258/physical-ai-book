from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from agent import run_agent  

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

@app.get("/")
def home():
    return {"message": "Physical AI Tutor API is Running! 🤖"}

@app.post("/chat")
async def chat_endpoint(request: ChatRequest):
    print(f"📩 Received Query: {request.question}")
    
    try:
        # call Agent 
        response = await run_agent(request.question)
        return {"answer": response}
    except Exception as e:
        print(f"❌ Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    #for Localhost 8000 
    uvicorn.run(app, host="0.0.0.0", port=8000)