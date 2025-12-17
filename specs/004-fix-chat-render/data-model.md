# Data Model: Fix RAG Chatbot Deployment on Render

**Feature**: 004-fix-chat-render
**Date**: 2025-12-18
**Purpose**: Define data structures for enhanced logging, error handling, and request/response validation

## Entities

### 1. ChatRequest (Existing - No Changes)

**Purpose**: Input model for /chat endpoint

**Fields**:
- `question`: string (required) - User's query text

**Validation Rules**:
- Must not be empty string
- Must be valid UTF-8 (supports English and Urdu script)
- No maximum length enforced (allows long queries)

**State**: No state transitions (stateless request)

**Location**: `backend/server.py` line 22-23

---

### 2. ChatResponse (Existing - Implicit Structure)

**Purpose**: Output model for /chat endpoint

**Fields**:
- `answer`: string (required) - Generated response from RAG agent

**Validation Rules**:
- Must not be null or empty
- Must be valid JSON-serializable string
- Frontend expects exactly this key name (not `response`, `content`, or other variants)

**Location**: `backend/server.py` line 45

---

### 3. ErrorResponse (New - To Be Implemented)

**Purpose**: Standardized error response structure for all endpoints

**Fields**:
- `error`: string (required) - Error type/category
- `message`: string (required) - User-friendly error message
- `detail`: string (optional) - Technical error details for debugging
- `timestamp`: string (required) - ISO 8601 timestamp of error
- `endpoint`: string (required) - Which endpoint failed (e.g., "/chat")

**Validation Rules**:
- `error` must be one of: "validation_error", "retrieval_error", "generation_error", "server_error", "timeout_error"
- `message` must be user-friendly (no stack traces or technical jargon)
- `detail` should only be included in development mode or for server errors

**Example**:
```json
{
  "error": "retrieval_error",
  "message": "Unable to search the handbook. Please try again.",
  "detail": "QdrantException: Connection timeout to vector database",
  "timestamp": "2025-12-18T10:30:45Z",
  "endpoint": "/chat"
}
```

**Relationships**: Returned by all endpoints on error conditions

---

### 4. LogEntry (New - Implicit Structure)

**Purpose**: Structured logging format for debugging on Render

**Fields**:
- `timestamp`: ISO 8601 datetime
- `level`: string - "INFO", "ERROR", "DEBUG"
- `endpoint`: string - "/chat", "/translate", "/personalize"
- `stage`: string - "received", "validating", "retrieving", "generating", "responding", "error"
- `message`: string - Descriptive log message
- `data`: object (optional) - Additional context (request body, error details, etc.)

**Validation Rules**:
- All logs must include timestamp, level, endpoint, stage, message
- Sensitive data (API keys) must be redacted from `data` field
- User queries should be truncated to 100 characters in logs for privacy

**Example**:
```python
print(f"[{timestamp}] INFO /chat [received] Query: {query[:100]}...")
print(f"[{timestamp}] DEBUG /chat [retrieving] Calling Qdrant with query embedding")
print(f"[{timestamp}] INFO /chat [responding] Success - response length: {len(response)} chars")
print(f"[{timestamp}] ERROR /chat [error] {error_type}: {error_message}")
```

**Location**: To be implemented throughout `backend/server.py` and `backend/agent.py`

---

### 5. AgentContext (Existing - Implicit Structure)

**Purpose**: Context passed to run_agent() function

**Fields**:
- `user_query`: string - Original user question
- `context_text`: string - Retrieved handbook content from Qdrant
- `system_prompt`: string - Agent instructions (includes language detection rules)

**State Transitions**:
1. **Received**: User query arrives at endpoint
2. **Retrieved**: Qdrant returns relevant context chunks
3. **Formatted**: Query and context combined for OpenAI prompt
4. **Generated**: GPT-4o returns response
5. **Returned**: Response sent back to frontend

**Validation Rules**:
- If `context_text` is empty, agent should return "I cannot answer this based on the handbook context."
- Language detection must analyze `user_query` script (Arabic vs Latin)
- Technical terms must remain in English regardless of response language

**Location**: `backend/agent.py` lines 65-96

---

### 6. Environment Configuration (Existing - External)

**Purpose**: Required environment variables for production deployment

**Fields**:
- `PORT`: integer (default: 8000) - Server bind port (dynamic on Render)
- `OPENAI_API_KEY`: string (required) - OpenAI authentication
- `COHERE_API_KEY`: string (required) - Cohere embeddings authentication
- `QDRANT_URL`: string (required) - Qdrant Cloud endpoint
- `QDRANT_API_KEY`: string (required) - Qdrant authentication

**Validation Rules**:
- All API keys must be non-empty strings
- PORT must be valid integer between 1-65535
- Missing required keys should raise ValueError on startup (already implemented in agent.py)

**Location**: `.env` file (not in git), loaded via `load_dotenv()` at `server.py` line 8

---

## Data Flow Diagram

```text
1. Frontend (index.js)
   └─> POST /chat { "question": "What is ROS 2?" }

2. FastAPI Middleware (server.py)
   ├─> CORS validation
   └─> Request logging

3. Pydantic Validation (server.py line 22-23)
   └─> ChatRequest model validates { "question": str }

4. Endpoint Handler (server.py lines 38-48)
   └─> await run_agent(request.question)

5. RAG Agent (agent.py lines 65-96)
   ├─> retrieve(user_query) → context_text
   ├─> client.chat.completions.create() → response
   └─> return response

6. Response Formatting (server.py line 45)
   └─> return { "answer": response }

7. Frontend (index.js line 123)
   └─> const data = await response.json()
   └─> data.answer
```

---

## Validation Matrix

| Entity | Validation Type | Location | Status |
|--------|----------------|----------|--------|
| ChatRequest | Pydantic model | server.py:22-23 | ✅ Existing |
| ChatResponse | Implicit JSON | server.py:45 | ✅ Existing |
| ErrorResponse | HTTPException | server.py:48 | ⚠️ Needs standardization |
| LogEntry | Print statements | server.py, agent.py | ⚠️ Needs structure |
| AgentContext | Function params | agent.py:65 | ✅ Existing |
| Environment Config | os.getenv() | server.py:8, agent.py:13 | ✅ Existing |

---

## Changes Required

### High Priority
1. **Standardize ErrorResponse**: Replace all `HTTPException(status_code=500, detail=str(e))` with structured ErrorResponse model
2. **Add LogEntry structure**: Replace all `print()` statements with structured logging format
3. **Add request body logging**: Log full request body after Pydantic validation to verify frontend sends correct data

### Medium Priority
4. **Add response validation**: Verify response structure before returning to catch serialization errors
5. **Add timeout handling**: Wrap long-running operations (retrieval, OpenAI calls) with explicit timeouts

### Low Priority
6. **Add metrics collection**: Track request count, response time, error rate per endpoint (post-fix monitoring)

---

## Notes

- This is a bug fix, not a feature addition - data model changes are minimal
- Focus is on observability (logging) and reliability (error handling)
- No database schema changes required (Qdrant structure unchanged)
- No frontend changes required (ChatBot component already correct)
