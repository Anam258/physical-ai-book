# Quickstart: Fix RAG Chatbot Deployment on Render

**Feature**: 004-fix-chat-render
**Date**: 2025-12-18
**Audience**: Developers implementing the fix

## Overview

This guide provides the implementation roadmap for fixing the /chat endpoint deployment on Render. The fix focuses on enhanced logging, error handling, and validation to diagnose and resolve the production failure.

## Problem Statement

**Current State**:
- `/chat` endpoint works perfectly on localhost (Windows)
- `/chat` endpoint fails on Render (Linux) with "API Error/No Response"
- `/translate` and `/personalize` endpoints work correctly on Render

**Root Cause Hypothesis**:
Based on code analysis, the most likely causes are:
1. Silent failure in error handling that doesn't propagate to logs
2. Timeout in retrieval operation that frontend doesn't handle gracefully
3. Environment-specific differences in how exceptions are handled

## Implementation Strategy

### Phase 1: Enhanced Logging (Diagnostic)

**Goal**: Add comprehensive logging to identify where requests fail in production

**Files to Modify**:
- `backend/server.py` - Add structured logging to /chat endpoint
- `backend/agent.py` - Add logging to run_agent() function

**Changes Required**:

1. **Add timestamp utility** (top of server.py):
```python
from datetime import datetime

def log_timestamp():
    return datetime.utcnow().isoformat() + "Z"
```

2. **Enhance /chat endpoint logging** (lines 38-48):
```python
@app.post("/chat")
async def chat_endpoint(request: ChatRequest):
    timestamp = log_timestamp()
    print(f"[{timestamp}] INFO /chat [received] Query: {request.question[:100]}...")
    print(f"[{timestamp}] DEBUG /chat [validating] Request body: {request.dict()}")

    try:
        print(f"[{timestamp}] INFO /chat [calling_agent] Starting run_agent...")
        response = await run_agent(request.question)
        print(f"[{timestamp}] INFO /chat [agent_complete] Response length: {len(response)} chars")

        result = {"answer": response}
        print(f"[{timestamp}] INFO /chat [responding] Sending response with keys: {list(result.keys())}")
        return result
    except Exception as e:
        print(f"[{timestamp}] ERROR /chat [error] Exception type: {type(e).__name__}")
        print(f"[{timestamp}] ERROR /chat [error] Message: {str(e)}")
        import traceback
        print(f"[{timestamp}] ERROR /chat [traceback] {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=str(e))
```

3. **Add logging to agent.py** (lines 65-96):
```python
async def run_agent(user_query: str):
    from datetime import datetime
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
```

**Expected Outcome**: After deployment, Render logs will show exactly where the request fails:
- If logs stop at `[calling_agent]`, the issue is in run_agent()
- If logs stop at `[Starting retrieval]`, the issue is in retrieve()
- If logs stop at `[Calling OpenAI GPT-4o]`, the issue is OpenAI API
- If logs show complete cycle, the issue is in response serialization or frontend

---

### Phase 2: Error Response Standardization (Reliability)

**Goal**: Ensure all error paths return consistent, parseable JSON structures

**Changes Required**:

1. **Add ErrorResponse model** (after ChatRequest in server.py):
```python
class ErrorResponse(BaseModel):
    error: str
    message: str
    detail: str = None
    timestamp: str
    endpoint: str
```

2. **Update error handling** to return structured errors:
```python
except Exception as e:
    error_response = {
        "error": "server_error",
        "message": "An error occurred processing your request",
        "detail": str(e),
        "timestamp": log_timestamp(),
        "endpoint": "/chat"
    }
    print(f"[{log_timestamp()}] ERROR /chat [error] Returning error response: {error_response}")
    raise HTTPException(status_code=500, detail=error_response)
```

**Expected Outcome**: Frontend receives parseable error structure even when backend fails

---

### Phase 3: Verification & Testing

**Deployment Steps**:

1. **Commit changes to feature branch**:
```bash
git add backend/server.py backend/agent.py
git commit -m "Add enhanced logging and error handling for /chat endpoint"
git push origin 004-fix-chat-render
```

2. **Merge to main** (triggers Render auto-deploy):
```bash
git checkout main
git merge 004-fix-chat-render
git push origin main
```

3. **Monitor Render logs** immediately after deployment:
   - Open Render dashboard → physical-ai-backend → Logs
   - Send test query from production site: https://Anam258.github.io/physical-ai-book/
   - Watch for structured log entries

4. **Analyze logs** to identify failure point:
   - Look for `ERROR` level logs
   - Check which stage failed: `[received]`, `[calling_agent]`, `[retrieval]`, `[generation]`, or `[responding]`
   - Note the exception type and message

5. **Iterate if needed**:
   - Based on logs, implement targeted fix
   - If retrieval fails: Check Qdrant environment variables
   - If OpenAI fails: Check API key and rate limits
   - If response fails: Check JSON serialization

**Success Criteria**:
- Render logs show complete request cycle from `[received]` to `[responding]`
- Frontend receives `{"answer": "..."}` response within 5 seconds
- No exceptions or errors in Render logs
- `/chat` endpoint works identically to `/translate` and `/personalize`

---

## Files Modified

| File | Lines | Changes |
|------|-------|---------|
| `backend/server.py` | 1-104 | Add timestamp utility, enhance /chat logging, standardize error responses |
| `backend/agent.py` | 65-96 | Add comprehensive logging at each stage of run_agent() |

**Total LOC Impact**: ~30 lines added (logging and error handling)

---

## Rollback Plan

If the fix doesn't work or causes issues:

1. **Revert on Render**:
   - Render dashboard → Deployments → Select previous deployment → "Redeploy"
   - Takes ~2 minutes to roll back

2. **Revert in Git**:
```bash
git checkout main
git revert HEAD
git push origin main
```

3. **Investigate further**:
   - Review captured logs before rollback
   - Open GitHub issue with log excerpts
   - Consider alternative hypotheses

---

## Next Steps After Fix

Once /chat works on production:

1. **Remove excessive logging**: Keep structured format but reduce verbosity
2. **Add performance monitoring**: Track response times per endpoint
3. **Implement rate limiting**: Protect against abuse (if needed)
4. **Add automated tests**: Integration tests for all three endpoints

---

## Dependencies

**Required**:
- Git access to push changes
- Render dashboard access to view logs
- Environment variables correctly configured in Render

**Not Required**:
- No new Python packages
- No frontend changes
- No Qdrant schema changes
- No database migrations

---

## Estimated Time

- **Implementation**: 30 minutes (adding logging and error handling)
- **Testing locally**: 15 minutes (verify logs appear correctly)
- **Deployment**: 5 minutes (git push + Render auto-deploy)
- **Monitoring**: 15 minutes (watch logs, send test queries)
- **Total**: ~1 hour

---

## Support

If issues persist after implementing this fix:

1. **Check Render logs** for the new structured log entries
2. **Review contracts/chat-api.yaml** for expected request/response structure
3. **Compare with working endpoints** (/translate, /personalize)
4. **Open issue** with log excerpts showing failure point

Remember: The goal is not to guess the fix, but to add observability so we can diagnose the real issue from production logs.
