# Research: Fix Personalize Endpoint Validation Error

**Feature**: 003-fix-personalize-validation
**Date**: 2025-12-17
**Research Phase**: Phase 0

## Research Tasks

### 1. FastAPI Request Validation Approaches

**Question**: What are the best approaches to accept flexible JSON keys in FastAPI while maintaining type safety?

**Decision**: Use FastAPI's `Request` object with manual JSON extraction

**Rationale**:
- **Current Problem**: Strict Pydantic model `PersonalizeRequest(BaseModel)` with `topic: str` and `role: str` fields returns 422 error when frontend sends different key names like "background" or "userRole"
- **Solution Benefits**:
  - `from fastapi import Request` allows access to raw request body
  - `await request.json()` returns dict with all keys
  - Manual extraction with `.get()` method provides fallback logic
  - No schema validation errors - accepts any valid JSON
  - Maintains backward compatibility with existing "role" key

**Alternatives Considered**:
1. **Multiple Pydantic Models**: Create separate models for each key variant
   - **Rejected**: Would require union types and complex validation logic, not maintainable
2. **Pydantic Field Aliases**: Use `Field(alias=...)` to map multiple names to one field
   - **Rejected**: Only allows one alias per field, can't handle 3+ variants with priority order
3. **Custom Pydantic Validator**: Use `@validator` decorator to accept multiple keys
   - **Rejected**: Complex implementation, still tied to Pydantic schema, harder to debug

**Implementation Pattern**:
```python
from fastapi import Request

@app.post("/personalize")
async def personalize_endpoint(request: Request):
    # Get raw JSON
    data = await request.json()

    # Manual extraction with priority order
    topic = data.get("topic", "")
    role = data.get("role") or data.get("background") or data.get("userRole") or "student"

    # Existing logic continues...
```

### 2. Error Handling for Missing Required Fields

**Question**: How should the endpoint handle missing `topic` field (required) vs missing role fields (optional)?

**Decision**: Return HTTP 422 for missing `topic`, default to "student" for missing role

**Rationale**:
- `topic` is required for generating explanations - endpoint cannot function without it
- Role fields are optional - reasonable default is "student" for general audience
- Maintains existing FastAPI error handling patterns in backend/server.py (line 45: HTTPException 500)
- Follows spec requirement FR-006: "Handle missing or empty role values gracefully by defaulting to 'student'"

**Implementation Pattern**:
```python
topic = data.get("topic")
if not topic:
    raise HTTPException(status_code=422, detail="Missing required field: topic")
```

### 3. Logging Strategy for Debugging

**Question**: What should be logged to console to help debug frontend/backend key mismatches?

**Decision**: Log the entire received JSON object before processing

**Rationale**:
- Spec requirement FR-003: "System MUST log the received JSON data to console for debugging purposes"
- Full JSON visibility helps identify which keys frontend is sending
- Consistent with existing pattern in server.py (lines 38, 50, 64: print statements)
- No sensitive data in this endpoint (just topic and role strings)

**Implementation Pattern**:
```python
print(f"📩 Received Personalization Request: {data}")
```

### 4. Priority Order for Key Extraction

**Question**: What order should keys be checked to maintain backward compatibility and handle common variants?

**Decision**: Check in order: `role` → `background` → `userRole` → default "student"

**Rationale**:
- **"role"**: Original key from Pydantic model - must check first for backward compatibility
- **"background"**: Common naming convention in educational/profile contexts
- **"userRole"**: Common naming convention in frontend frameworks (camelCase)
- **"student"**: Sensible default for educational platform targeting learners
- Priority order documented in spec FR-002

**Implementation Pattern**:
```python
role = data.get("role") or data.get("background") or data.get("userRole") or "student"
```

**Edge Case Handling**: If multiple keys are present (e.g., both "role" and "background"), the priority order determines which value is used. This is acceptable because:
- Frontend should only send one key variant
- If multiple keys exist, taking the first non-empty value is deterministic
- Logging shows all received keys for debugging conflicts

### 5. Integration with Existing run_agent Function

**Question**: Does the flexible validation approach affect the existing `run_agent` integration?

**Decision**: No changes needed to `run_agent` integration

**Rationale**:
- `run_agent(query: str)` in backend/agent.py (line 65) accepts a string prompt
- Current endpoint already formats a string query: `f"Explain {topic} for someone with {role} background..."`
- Manual extraction simply changes where `role` variable comes from (dict vs Pydantic model)
- Formatting logic remains identical
- Spec requirement FR-004: "Continue using existing run_agent function"

**No Changes Required**:
```python
# Before (with Pydantic):
query = f"Explain {request.topic} for someone with {request.role} background in 2-3 sentences..."

# After (with Request):
query = f"Explain {topic} for someone with {role} background in 2-3 sentences..."
# Only variable source changed, formatting identical
```

## Technology Stack Confirmation

**Backend Framework**: FastAPI 0.124.0+
**Python Version**: 3.12+ (confirmed from backend/pyproject.toml in feature 002)
**Request Handling**: `Request` object from `fastapi` module
**JSON Parsing**: `await request.json()` built-in method
**Error Handling**: `HTTPException` from `fastapi` module

**Dependencies** (no new dependencies needed):
- `fastapi` - already installed
- `pydantic` - keeping BaseModel for other endpoints
- `python-dotenv` - environment variables
- All existing dependencies remain unchanged

## Performance Considerations

**Impact**: Negligible performance difference

**Analysis**:
- Pydantic validation: ~0.1ms overhead for model instantiation and field validation
- Manual extraction: ~0.05ms overhead for dict.get() operations (3-4 calls)
- Difference: < 0.1ms per request
- Existing bottleneck: OpenAI API call (~2-5 seconds per spec SC-007)
- Validation change is <0.01% of total request time

**Conclusion**: Performance requirement SC-007 easily maintained

## Security Considerations

**Question**: Does accepting raw JSON introduce security risks?

**Decision**: Low risk for this specific endpoint

**Analysis**:
- **SQL Injection**: Not applicable - no database queries
- **XSS**: Not applicable - no HTML rendering, JSON response only
- **DoS**: Limited by OpenAI rate limits and Render infrastructure
- **Data Validation**: Still validating topic presence (required field)
- **Type Safety**: Python's dynamic typing handles string extraction safely

**Mitigation**:
- Keep existing try-except error handling (line 73 pattern)
- Validate topic is non-empty string
- OpenAI handles malicious prompts in their API layer
- CORS already configured (lines 13-19)

## Testing Strategy

**Manual Testing** (per spec - no automated tests requested):
1. Test with "role" key (backward compatibility)
2. Test with "background" key (new variant)
3. Test with "userRole" key (new variant)
4. Test with missing role key (default to "student")
5. Test with missing topic key (expect 422 error)
6. Test with empty topic string (expect 422 error)
7. Verify console logging shows all received data

**Test Tool**: curl or Postman with JSON payloads

## Research Outcomes Summary

| Research Area | Outcome | Impact on Implementation |
|--------------|---------|--------------------------|
| Validation Approach | Use `Request` object with manual extraction | Replace Pydantic model with Request parameter |
| Error Handling | 422 for missing topic, "student" default for role | Add topic validation check |
| Logging | Log entire JSON object | Add print statement before processing |
| Key Priority | role → background → userRole → "student" | Use chained `.get()` calls with `or` operator |
| run_agent Integration | No changes needed | Keep existing query formatting |
| Performance | <0.1ms difference, negligible | No optimization needed |
| Security | Low risk | Keep existing error handling |

## Files to Modify

**backend/server.py**:
- Line 28-30: Remove or comment out `PersonalizeRequest` Pydantic model
- Line 1: Add `Request` to FastAPI imports: `from fastapi import FastAPI, HTTPException, Request`
- Line 62-74: Rewrite `personalize_endpoint` function:
  - Change signature: `async def personalize_endpoint(request: Request)`
  - Add: `data = await request.json()`
  - Add: `print(f"📩 Received Personalization Request: {data}")`
  - Add: `topic = data.get("topic")`
  - Add: Topic validation
  - Add: `role = data.get("role") or data.get("background") or data.get("userRole") or "student"`
  - Keep: Existing query formatting and run_agent call

**No Other Files Modified**:
- backend/agent.py: No changes (run_agent signature unchanged)
- backend/retriveing.py: No changes
- backend/main.py: No changes
- frontend/*: Out of scope (backend-only fix)

## Conclusion

All research tasks completed. All "NEEDS CLARIFICATION" items from Technical Context are resolved:
- ✅ Validation approach: Use Request object with manual extraction
- ✅ Error handling: 422 for missing topic, "student" default for role
- ✅ Logging strategy: Log entire JSON object
- ✅ Key priority order: role → background → userRole → "student"
- ✅ Integration impact: None - run_agent unchanged
- ✅ Performance: Negligible impact (<0.1ms)
- ✅ Security: Low risk with existing mitigations

Ready to proceed to Phase 1: Data Model & Contracts.
