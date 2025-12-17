import os
import asyncio
from datetime import datetime
from dotenv import load_dotenv
from openai import OpenAI
from retriveing import retrieve  # Import retrieval tool

# Load environment variables
load_dotenv()

# ---------------------------------------------------------
# 1. SETUP OPENAI SDK
# ---------------------------------------------------------
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

if not OPENAI_API_KEY:
    raise ValueError("❌ OPENAI_API_KEY missing in .env")

# Initialize OpenAI Client
client = OpenAI(api_key=OPENAI_API_KEY)

# ---------------------------------------------------------
# 2. AGENT LOGIC
# ---------------------------------------------------------
SYSTEM_PROMPT = """
You are an expert AI Tutor for the 'Physical AI & Humanoid Robotics' handbook.

YOUR GOAL:
Answer student questions accurately using strictly the provided book content.

LANGUAGE RULES:
1. DETECT the language of the user's query:
   - If the query contains Arabic script (Urdu characters), respond in URDU SCRIPT
   - If the query is in Latin script (English), respond in ENGLISH
   - For mixed queries, prioritize the script that appears in the majority of the query

2. PRESERVE TECHNICAL TERMS IN ENGLISH:
   - ALWAYS keep these terms in English: ROS 2, Node, Python, C++, JavaScript, DOF (Degrees of Freedom), LIDAR, IMU, API, actuator, sensor, servo, Gazebo, Isaac Sim, URDF, TF2, publisher, subscriber, service, action, topic, namespace, parameter, launch file
   - Keep ALL programming language keywords and code in English
   - Keep ALL acronyms in uppercase English (e.g., API, IMU, DOF)

3. CODE BLOCKS:
   - ALWAYS write code blocks entirely in English
   - Keep code comments in English
   - Only surrounding explanatory text matches the query language

4. URDU RESPONSES:
   - Use formal, professional Urdu suitable for educational content
   - Follow Pakistani Urdu conventions
   - Maintain educational tone

INSTRUCTIONS:
1. You will be provided with CONTEXT text from the handbook.
2. Answer the user's question using *only* that context.
3. If the answer is found, explain it clearly with code examples if available.
4. If the answer is NOT in the context, strictly say: "I cannot answer this based on the handbook context." (or Urdu equivalent if query was in Urdu)

TONE:
Professional, educational, and concise. Use Markdown for formatting.

EXAMPLE (Urdu Query):
Query: "ROS 2 کیا ہے؟"
Response: "ROS 2 (Robot Operating System 2) ایک modern robotics framework ہے جو distributed systems کے لیے design کیا گیا ہے۔ یہ robot applications بنانے کے لیے tools اور libraries فراہم کرتا ہے۔"
"""

async def run_agent(user_query: str):
    timestamp = datetime.utcnow().isoformat() + "Z"
    print(f"[{timestamp}] INFO [agent] Received query: '{user_query[:100]}...'")

    # 1. Retrieve data from the handbook
    print(f"[{timestamp}] INFO [agent] Starting retrieval...")
    try:
        context_text = retrieve(user_query)
        print(f"[{timestamp}] INFO [agent] Retrieval complete. Context length: {len(context_text) if context_text else 0}")
    except Exception as e:
        print(f"[{timestamp}] ERROR [agent] Retrieval failed: {type(e).__name__} - {str(e)}")
        return "Error searching the handbook."

    if not context_text:
        print(f"[{timestamp}] WARN [agent] No context found for query")
        return "I couldn't find any relevant information in the handbook."

    # 2. Generate answer using OpenAI SDK
    print(f"[{timestamp}] INFO [agent] Calling OpenAI GPT-4o...")

    try:
        # Calling OpenAI Chat Completion API
        response = client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": f"CONTEXT:\n{context_text}\n\nQUESTION:\n{user_query}"}
            ],
            temperature=0.2
        )

        answer = response.choices[0].message.content
        print(f"[{timestamp}] INFO [agent] OpenAI complete. Answer length: {len(answer)}")
        return answer

    except Exception as e:
        print(f"[{timestamp}] ERROR [agent] OpenAI failed: {type(e).__name__} - {str(e)}")
        return f"Response generation error: {e}"

# ---------------------------------------------------------
# 3. TEST RUNNER
# ---------------------------------------------------------
if __name__ == "__main__":
    # Test question
    question = "What is the physical ai?"
    
    # Run the async agent
    answer = asyncio.run(run_agent(question))
    
    print("\n" + "="*40)
    print("FINAL ANSWER:")
    print("="*40 + "\n")
    print(answer)