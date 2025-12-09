import os
import asyncio
from dotenv import load_dotenv
import google.generativeai as genai
from retriveing import retrieve  # Import retrieval tool

# Load environment variables
load_dotenv()

# ---------------------------------------------------------
# 1. SETUP GEMINI (NATIVE GOOGLE SDK)
# ---------------------------------------------------------
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

if not GEMINI_API_KEY:
    raise ValueError("❌ GEMINI_API_KEY missing in .env")

# Configure Google AI
genai.configure(api_key=GEMINI_API_KEY)

# ---------------------------------------------------------
# 2. AGENT LOGIC
# ---------------------------------------------------------
SYSTEM_PROMPT = """
You are an expert AI Tutor for the 'Physical AI & Humanoid Robotics' handbook.

YOUR GOAL:
Answer student questions accurately using strictly the provided book content.

INSTRUCTIONS:
1.  You will be provided with CONTEXT text from the handbook.
2.  Answer the user's question using *only* that context.
3.  If the answer is found, explain it clearly with code examples if available.
4.  If the answer is NOT in the context, strictly say: "I cannot answer this based on the handbook context."

TONE:
Professional, educational, and concise. Use Markdown for formatting.
"""

async def run_agent(user_query: str):
    print(f"\n🤖 Thinking about: '{user_query}'...")
    
    # 1.  (Retrieve)
    print("📚 Searching handbook...")
    try:
        context_text = retrieve(user_query)
    except Exception as e:
        print(f"Retrieval Error: {e}")
        return "Error searching the handbook."
    
    if not context_text:
        return "I couldn't find any relevant information in the handbook."

    # 2. Make Answers from gemini (Generate)
    print("💡 Generating answer...")
    
    # Initialize Model with System Instruction
    model = genai.GenerativeModel(
        model_name="gemini-2.5-flash",
        system_instruction=SYSTEM_PROMPT
    )

    # Chat with the model
    response = model.generate_content(
        f"CONTEXT:\n{context_text}\n\nQUESTION:\n{user_query}"
    )
    
    return response.text

# ---------------------------------------------------------
# 3. TEST RUNNER
# ---------------------------------------------------------
if __name__ == "__main__":
    # Test question
    question = "What is the difference between Sim-to-Real and Digital Twin?"
    
    # Run the function
    answer = asyncio.run(run_agent(question))
    
    print("\n" + "="*40)
    print("FINAL ANSWER:")
    print("="*40 + "\n")
    print(answer)