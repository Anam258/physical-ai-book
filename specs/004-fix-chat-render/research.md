# Research: Fix RAG Chatbot Deployment on Render

**Feature**: 004-fix-chat-render
**Date**: 2025-12-18
**Purpose**: Investigate root causes of /chat endpoint failure on Render while /translate and /personalize work

## Research Questions

### 1. How does Render start Python applications?

**Decision**: Render uses the start command specified in render.yaml or dashboard settings, NOT the `if __name__ == "__main__"` block.

**Rationale**:
- Render deployments typically use explicit commands like `uvicorn server:app --host 0.0.0.0 --port $PORT`
- The `if __name__ == "__main__"` block at lines 100-104 in `server.py` only executes when running `python server.py` directly
- This explains why PORT binding works (lines 102-103) but may not be the issue since translation/personalization endpoints work

**Alternatives Considered**:
- Using Gunicorn instead of uvicorn - rejected because uvicorn is already working for other endpoints
- Modifying the main block - rejected because Render likely doesn't execute it

**Action Required**: Verify Render's actual start command and ensure it matches the working endpoints' pattern

---

### 2. Why would one endpoint fail while others succeed?

**Decision**: The issue is NOT in the endpoint handler code itself - all three endpoints use identical patterns.

**Analysis of Code Patterns**:

**/chat endpoint (lines 38-48)**:
```python
@app.post("/chat")
async def chat_endpoint(request: ChatRequest):
    print(f"📩 Received Query: {request.question}")
    try:
        response = await run_agent(request.question)
        return {"answer": response}
    except Exception as e:
        print(f"❌ Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))
```

**/translate endpoint (lines 50-62)** - WORKS ON RENDER:
```python
@app.post("/translate")
async def translate_endpoint(request: TranslateRequest):
    print(f"📩 Received Translation Request: {request.text[:50]}...")
    try:
        query = f"Translate this technical text to formal Urdu, keeping English keywords: {request.text}"
        response = await run_agent(query)
        return {"translation": response}
    except Exception as e:
        print(f"❌ Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))
```

**Key Observation**: Both use `await run_agent()`, both have try/except, both return JSON. The ONLY differences are:
1. Request model: `ChatRequest` vs `TranslateRequest`
2. Response key: `answer` vs `translation`
3. Prompt formatting: Direct query vs formatted string

**Rationale**: Since the code patterns are identical, the issue must be:
- **Environment-specific error** in the retrieval system that only affects certain types of queries
- **Silent failure** in error handling that doesn't propagate to Render logs
- **Timeout** in the RAG retrieval that's longer for /chat queries
- **Model validation issue** with ChatRequest vs TranslateRequest

**Most Likely Cause**: The Pydantic validation for `ChatRequest` (line 22-23) expects `question: str`, but the frontend sends `question` field correctly (line 114 in index.js). However, there may be additional fields in the request body that cause validation to fail silently.

---

### 3. Render Debugging Best Practices

**Decision**: Implement comprehensive logging at every stage of the request lifecycle.

**Best Practices Identified**:
1. **Log request body**: Add logging to see exact JSON received
2. **Log after validation**: Confirm Pydantic didn't reject the request
3. **Log retrieval results**: Verify Qdrant returns context
4. **Log OpenAI calls**: Verify GPT-4o generates responses
5. **Log before return**: Confirm response structure before sending

**Rationale**: The current logging (lines 40, 47, 52, 61, 71, 82, 98) only logs on entry and error. Missing logs for:
- Successful retrieval completion
- OpenAI API call success
- Response structure validation

**Implementation Pattern**:
```python
@app.post("/chat")
async def chat_endpoint(request: ChatRequest):
    print(f"📩 [CHAT] Received Query: {request.question}")
    print(f"📦 [CHAT] Request body: {request.dict()}")

    try:
        print(f"🔄 [CHAT] Calling run_agent...")
        response = await run_agent(request.question)
        print(f"✅ [CHAT] Agent returned: {response[:100]}...")

        result = {"answer": response}
        print(f"📤 [CHAT] Sending response with keys: {result.keys()}")
        return result
    except Exception as e:
        print(f"❌ [CHAT] Error: {e}")
        import traceback
        print(f"📋 [CHAT] Traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=str(e))
```

---

### 4. Environment Variable Handling

**Decision**: Environment variables are correctly loaded via `load_dotenv()` at line 8, and all required keys are validated in `agent.py` lines 13-16.

**Analysis**:
- PORT: Correctly retrieved with fallback (lines 102-103)
- OPENAI_API_KEY: Validated in agent.py with ValueError if missing
- COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY: Required by retriveing.py

**Rationale**: Since /translate and /personalize work, environment variables are correctly configured. The issue is NOT related to missing or invalid environment variables.

---

### 5. Async/Await Pattern Analysis

**Decision**: Async operations are correctly implemented - `run_agent()` is properly awaited in all three endpoints.

**Verification**:
- Line 44: `response = await run_agent(request.question)` ✅
- Line 58: `response = await run_agent(query)` ✅
- Line 87: `response = await run_agent(prompt)` ✅

**Rationale**: No coroutine warnings would occur since all async calls are properly awaited. This is NOT the issue.

---

## Root Cause Hypothesis

After analyzing the code and comparing working vs failing endpoints, the most likely root cause is:

**Primary Hypothesis**: The `/chat` endpoint is actually working correctly in terms of code, but fails due to:
1. **Frontend timeout**: The ChatBot component may have a shorter timeout than translation/personalization
2. **CORS preflight cache**: Browser may have cached a failed CORS preflight for the /chat endpoint specifically
3. **Request payload structure**: Frontend may be sending extra fields that Pydantic validation rejects

**Secondary Hypothesis**: Silent failure in the retrieval system:
- `retrieve()` function in `retriveing.py` may throw exceptions that get caught but return empty context
- Empty context causes OpenAI to return generic responses that frontend interprets as errors
- This only affects /chat queries, not /translate (which doesn't use retrieval in the same way)

**Evidence Supporting Primary Hypothesis**:
- Translation endpoint works (uses same `run_agent` pattern)
- Personalization endpoint works (uses same `run_agent` pattern)
- Only difference is the request model and response key
- Frontend code shows correct structure (line 114: `body: JSON.stringify({ question: message })`)

**Evidence Supporting Secondary Hypothesis**:
- `/chat` is the only endpoint that directly uses user queries for retrieval
- `/translate` and `/personalize` format the query before calling `run_agent`
- Formatted queries might have different retrieval behavior

---

## Recommended Implementation Strategy

### Strategy 1: Enhanced Logging (IMMEDIATE - Required for diagnosis)
Add detailed logging to identify where the request fails in production

### Strategy 2: Error Response Standardization (HIGH PRIORITY)
Ensure all error paths return consistent JSON structure that frontend can parse

### Strategy 3: Request Validation Logging (HIGH PRIORITY)
Log the exact request body received to verify frontend sends correct structure

### Strategy 4: Timeout Handling (MEDIUM PRIORITY)
Add explicit timeout handling for long-running retrieval operations

### Strategy 5: Health Check Endpoint (LOW PRIORITY - Post-fix verification)
Add a simple /health endpoint to verify server is responsive

---

## Decisions Summary

| Area | Decision | Rationale |
|------|----------|-----------|
| **Startup Command** | Verify Render uses uvicorn, not python server.py | Working endpoints prove uvicorn is configured correctly |
| **Root Cause** | Code is likely correct; issue is environment-specific or timeout-related | Identical patterns work for other endpoints |
| **Debugging** | Add comprehensive logging at every request stage | Current logs insufficient for production debugging |
| **Error Handling** | Ensure all exceptions include full traceback in logs | Silent failures prevent diagnosis |
| **Validation** | Log request body before Pydantic validation | Verify frontend sends expected structure |

---

## Open Questions for Verification

1. What is the exact Render start command? (Check render.yaml or dashboard)
2. What do Render logs show when /chat is called? (Need production log access)
3. Does the frontend actually send requests, or does it fail before network call?
4. Are there CORS preflight failures in browser console?
5. What is the actual error message users see? (Generic "API Error" or specific message?)

**Next Steps**: Proceed to Phase 1 to design the logging enhancement and error handling improvements.
