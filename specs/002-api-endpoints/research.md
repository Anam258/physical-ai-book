# Research Findings: Backend API Endpoints

**Feature**: 002-api-endpoints
**Date**: 2025-12-17
**Purpose**: Document technical decisions and patterns for implementing /translate and /personalize endpoints

## Research Questions

### Q1: How should new endpoints integrate with the existing run_agent function?

**Decision**: Pass custom prompt strings to run_agent() without modifying its signature or internal logic

**Rationale**:
- Examined agent.py (lines 65-96): `run_agent(user_query: str)` is async and expects a single string parameter
- Current implementation (lines 84-91) constructs a messages array with SYSTEM_PROMPT and user query
- The function already handles RAG retrieval via `retrieve(user_query)` (line 71) and OpenAI completion (lines 84-91)
- For translation: pass formatted string like "Translate this technical text to formal Urdu, keeping English keywords: {text}"
- For personalization: pass formatted string like "Explain {topic} for someone with {role} background in 2-3 sentences"
- No changes needed to agent.py or SYSTEM_PROMPT - it already supports language detection and technical term preservation (lines 30-50)

**Alternatives Considered**:
1. **Modify run_agent to accept prompt parameter** - Rejected because it would break existing /chat endpoint and violate FR-007 (preserve existing functionality)
2. **Create new agent functions** - Rejected as over-engineering; run_agent is already flexible enough
3. **Bypass agent and call OpenAI directly** - Rejected because it would duplicate RAG logic and violate DRY principle

### Q2: What Pydantic model pattern should be used for request/response validation?

**Decision**: Follow the existing ChatRequest pattern with single BaseModel classes per endpoint

**Rationale**:
- Examined server.py (lines 22-23): Current pattern uses simple Pydantic BaseModel with required fields
- FastAPI automatically validates JSON body against Pydantic models and returns 422 for invalid input
- Pattern:
  ```python
  class TranslateRequest(BaseModel):
      text: str

  class PersonalizeRequest(BaseModel):
      topic: str
      role: str
  ```
- Pydantic already installed (pyproject.toml line 5) and imported (server.py line 3)
- No need for response models since we return dict literals (FastAPI serializes them automatically)

**Alternatives Considered**:
1. **Use TypedDict instead of Pydantic** - Rejected because FastAPI request validation requires Pydantic models
2. **Create response models** - Rejected as unnecessary; dict return values work fine per existing /chat pattern (line 36)
3. **Use single generic Request model with optional fields** - Rejected because it reduces type safety and validation clarity

### Q3: What error handling pattern should be used?

**Decision**: Follow the existing try-except HTTPException pattern from /chat endpoint

**Rationale**:
- Examined server.py /chat endpoint (lines 29-39):
  - Uses try-except block around async run_agent call
  - Catches generic Exception (line 37)
  - Raises HTTPException with status_code=500 and detail=str(e) (line 39)
  - Includes console logging with print statements (lines 31, 38)
- This pattern satisfies FR-010 (handle errors gracefully) and maintains consistency
- FastAPI automatically converts HTTPException to proper HTTP error response
- Pattern:
  ```python
  try:
      response = await run_agent(query)
      return {"translation": response}
  except Exception as e:
      print(f"❌ Error: {e}")
      raise HTTPException(status_code=500, detail=str(e))
  ```

**Alternatives Considered**:
1. **Return error dicts instead of raising exceptions** - Rejected because HTTPException is FastAPI standard and provides proper HTTP status codes
2. **Add specific exception types** - Rejected as over-engineering for this simple API
3. **Use FastAPI exception handlers** - Rejected because inline try-except is sufficient for 2 endpoints

### Q4: How should the translation prompt be formatted to ensure technical term preservation?

**Decision**: Construct explicit instruction string mentioning the LANGUAGE RULES already in SYSTEM_PROMPT

**Rationale**:
- Examined agent.py SYSTEM_PROMPT (lines 24-63): Already contains comprehensive language rules
  - Lines 30-50: LANGUAGE RULES with technical term preservation for ROS 2, Python, Node, etc.
  - Lines 47-49: "Keep ALL programming language keywords and code in English"
  - Example on lines 60-62 shows proper Urdu response with English technical terms
- The SYSTEM_PROMPT is already context-aware; we just need to trigger translation behavior
- Recommended prompt format: `f"Translate this technical text to formal Urdu, keeping English keywords: {request.text}"`
- The agent will apply LANGUAGE RULES automatically based on SYSTEM_PROMPT

**Alternatives Considered**:
1. **Include full technical term list in each prompt** - Rejected as redundant with SYSTEM_PROMPT
2. **Modify SYSTEM_PROMPT to have translation mode** - Rejected to avoid breaking existing /chat behavior
3. **Use separate OpenAI completion without SYSTEM_PROMPT** - Rejected because it would lose RAG context

### Q5: What is the expected response time and how should it be monitored?

**Decision**: Expect 2-5 second response times matching /chat; no special monitoring needed for MVP

**Rationale**:
- Examined agent.py run_agent flow:
  - Line 71: `retrieve(user_query)` - Qdrant vector search (typically <500ms)
  - Lines 84-91: OpenAI GPT-4o completion with temperature=0.2 (typically 1-4s depending on response length)
  - Total observed /chat response time: ~2-5 seconds per backend logs
- Translation and personalization use same run_agent flow, so expect similar timing
- No additional async operations or database calls
- FR-007 and SC-007 require "response time parity" - maintaining current performance is sufficient

**Alternatives Considered**:
1. **Add async caching layer** - Rejected as premature optimization (out of scope per spec.md line 136)
2. **Implement request timeouts** - Rejected because OpenAI SDK has built-in timeout handling
3. **Add Prometheus metrics** - Rejected as out of scope; print statements sufficient for educational platform

## Best Practices Applied

### FastAPI Endpoint Design
- **Source**: FastAPI official docs (https://fastapi.tiangolo.com/tutorial/body/)
- **Pattern**: Use Pydantic models for automatic request validation
- **Application**: Create TranslateRequest and PersonalizeRequest models with required string fields

### Async Function Handling
- **Source**: Python asyncio documentation + FastAPI async guide
- **Pattern**: Use `await` keyword when calling async functions; mark endpoint handlers as `async def`
- **Application**: `async def translate_endpoint(request: TranslateRequest):` with `await run_agent(...)`

### RESTful API Conventions
- **Source**: REST API best practices
- **Pattern**: Use POST for operations that transform/process data (vs GET for retrieval)
- **Application**: Both /translate and /personalize are POST endpoints since they process input and generate responses

### Error Response Structure
- **Source**: FastAPI HTTPException documentation
- **Pattern**: Use status_code=500 for internal server errors, status_code=422 for validation errors (automatic)
- **Application**: Raise HTTPException(status_code=500, detail=str(e)) for run_agent failures

## Implementation Notes

### No New Dependencies Required
All required packages already installed per pyproject.toml:
- ✅ fastapi >= 0.124.0 (web framework)
- ✅ pydantic (validation, bundled with FastAPI)
- ✅ openai >= 2.9.0 (LLM client, used by agent.py)
- ✅ uvicorn >= 0.38.0 (ASGI server)
- ✅ python-dotenv (env vars, already used in server.py line 5)

### Existing Code Reuse
- ✅ `run_agent` function (agent.py line 65) - no modifications needed
- ✅ CORS middleware (server.py lines 13-19) - applies to all routes automatically
- ✅ Error handling pattern (server.py lines 33-39) - copy to new endpoints

### Testing Strategy
- Manual testing via curl/Postman sufficient for MVP
- Integration test would call actual OpenAI API (no mocking needed for educational project)
- Example curl commands to be documented in quickstart.md

## Unresolved Questions

**None** - All technical clarifications resolved through codebase examination and documentation review.
