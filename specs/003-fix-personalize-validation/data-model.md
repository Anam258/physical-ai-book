# Data Model: Fix Personalize Endpoint Validation Error

**Feature**: 003-fix-personalize-validation
**Date**: 2025-12-17
**Phase**: 1

## Overview

This feature modifies the /personalize endpoint to accept flexible JSON key names for the user's role/background field. The data model shifts from strict Pydantic validation to manual JSON extraction.

## Entities

### PersonalizeRequest (Flexible Input)

**Description**: Incoming personalization request with flexible field naming

**Fields**:
| Field | Type | Required | Variants | Default | Validation |
|-------|------|----------|----------|---------|------------|
| topic | string | Yes | topic | N/A | Non-empty string |
| role | string | No | role, background, userRole | "student" | Any string value |

**Validation Rules**:
1. **Topic Validation**:
   - MUST be present in JSON body
   - MUST be non-empty string
   - If missing or empty → HTTP 422 error
   - Example valid: `"ROS 2"`, `"URDF files"`, `"Servos"`
   - Example invalid: `""`, `null`, missing key

2. **Role Validation** (Flexible):
   - MAY be present with any of three key names: "role", "background", "userRole"
   - Priority order: Check "role" first, then "background", then "userRole"
   - If all missing → default to "student"
   - If empty string → default to "student"
   - Any non-empty string value is valid
   - Example valid: `"Python developer"`, `"Arduino hobbyist"`, `"JavaScript developer"`
   - Example valid: Missing key → defaults to `"student"`

**State Transitions**: None (stateless request/response)

**Relationships**: None (standalone endpoint)

**JSON Schema** (Flexible):
```json
{
  "type": "object",
  "properties": {
    "topic": {
      "type": "string",
      "minLength": 1,
      "description": "The technical topic to explain"
    },
    "role": {
      "type": "string",
      "description": "User's background/role (optional)"
    },
    "background": {
      "type": "string",
      "description": "Alternative key for user's background (optional)"
    },
    "userRole": {
      "type": "string",
      "description": "Alternative key for user's role (optional)"
    }
  },
  "required": ["topic"],
  "additionalProperties": false
}
```

**Example Valid Requests**:
```json
// Original key name (backward compatibility)
{
  "topic": "ROS 2 Nodes",
  "role": "Python developer"
}

// "background" key variant
{
  "topic": "URDF files",
  "background": "Arduino hobbyist"
}

// "userRole" key variant
{
  "topic": "LIDAR sensors",
  "userRole": "JavaScript developer"
}

// Missing role key (defaults to "student")
{
  "topic": "Servos"
}

// Multiple keys present (priority: role wins)
{
  "topic": "IMU sensors",
  "role": "C++ developer",
  "background": "Python developer"
}
// Result: Uses "C++ developer" (role has highest priority)
```

**Example Invalid Requests**:
```json
// Missing topic
{
  "role": "Python developer"
}
// Result: HTTP 422 - Missing required field: topic

// Empty topic
{
  "topic": "",
  "role": "Python developer"
}
// Result: HTTP 422 - Missing required field: topic
```

### PersonalizeResponse (Output)

**Description**: Personalization result returned to frontend

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| explanation | string | Yes | 2-3 sentence contextualized explanation |

**JSON Schema**:
```json
{
  "type": "object",
  "properties": {
    "explanation": {
      "type": "string",
      "description": "Personalized explanation of the topic"
    }
  },
  "required": ["explanation"]
}
```

**Example Response**:
```json
{
  "explanation": "ROS 2 Nodes are like Python modules - independent programs that communicate via topics. For a Python developer, think of each Node as a class instance that can publish/subscribe to message queues. The rclpy library provides a Pythonic interface similar to Flask's request handling."
}
```

## Data Flow

```text
Frontend Request (Flexible Keys)
         ↓
FastAPI Request Object (Raw JSON)
         ↓
Manual Extraction (Priority Order)
         ↓
{
  topic: string (validated non-empty)
  role: string (extracted or defaulted to "student")
}
         ↓
Query Formatting
         ↓
"Explain {topic} for someone with {role} background in 2-3 sentences..."
         ↓
run_agent(query) → OpenAI GPT-4o
         ↓
Response String
         ↓
{"explanation": response}
         ↓
Frontend
```

## Implementation Changes

### Before (Strict Pydantic)

```python
class PersonalizeRequest(BaseModel):
    topic: str
    role: str

@app.post("/personalize")
async def personalize_endpoint(request: PersonalizeRequest):
    print(f"📩 Topic='{request.topic}', Role='{request.role}'")

    try:
        query = f"Explain {request.topic} for someone with {request.role} background..."
        response = await run_agent(query)
        return {"explanation": response}
    except Exception as e:
        print(f"❌ Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))
```

**Problem**: Returns 422 error when frontend sends "background" or "userRole" keys

### After (Flexible Manual Extraction)

```python
# Remove or comment out PersonalizeRequest Pydantic model

@app.post("/personalize")
async def personalize_endpoint(request: Request):
    # Get raw JSON
    data = await request.json()

    # Log for debugging
    print(f"📩 Received Personalization Request: {data}")

    # Extract topic (required)
    topic = data.get("topic")
    if not topic:
        raise HTTPException(status_code=422, detail="Missing required field: topic")

    # Extract role with fallback logic (optional)
    role = data.get("role") or data.get("background") or data.get("userRole") or "student"

    print(f"📝 Extracted - Topic: '{topic}', Role: '{role}'")

    try:
        query = f"Explain {topic} for someone with {role} background in 2-3 sentences using relevant analogies"
        response = await run_agent(query)
        return {"explanation": response}
    except Exception as e:
        print(f"❌ Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))
```

**Solution**: Accepts all key variants, defaults gracefully, logs for debugging

## Edge Case Handling

| Edge Case | Input | Behavior | Rationale |
|-----------|-------|----------|-----------|
| Empty role string | `{"topic": "ROS 2", "role": ""}` | Default to "student" | Empty string is falsy in Python, triggers `or` chain |
| Multiple role keys | `{"topic": "ROS 2", "role": "A", "background": "B"}` | Use "A" (role wins) | Priority order ensures deterministic behavior |
| Missing topic | `{"role": "Python dev"}` | HTTP 422 error | Topic is required for query generation |
| Empty topic | `{"topic": "", "role": "Python dev"}` | HTTP 422 error | Cannot generate meaningful explanation without topic |
| Null values | `{"topic": "ROS 2", "role": null}` | Default to "student" | null is falsy, triggers default |
| Extra fields | `{"topic": "ROS 2", "role": "Python", "foo": "bar"}` | Ignored | Only extract needed fields |
| Malformed JSON | `{topic: ROS 2}` | HTTP 422 from FastAPI | FastAPI's request.json() raises JSONDecodeError |
| Special characters | `{"topic": "C++", "role": "Python/JS developer"}` | Accepted | No character restrictions on strings |

## Type Safety

**Previous Approach** (Pydantic):
- Compile-time type hints via BaseModel
- Runtime validation automatic
- IDE autocomplete for `request.topic`, `request.role`

**New Approach** (Manual Extraction):
- Runtime extraction with explicit checks
- Type safety through validation logic
- IDE autocomplete for `data["topic"]` with dict type hint

**Trade-off**: Losing some IDE convenience for flexibility. Acceptable because:
- Endpoint has only 2 fields (simple extraction)
- Explicit validation makes behavior clear
- Logging shows exact data received
- Frontend compatibility is higher priority than type safety

## Backward Compatibility

**Guaranteed**: Existing frontend clients using "role" key continue to work identically

**Testing**:
```bash
# Original format (must still work)
curl -X POST https://physical-ai-backend-j8q9.onrender.com/personalize \
  -H "Content-Type: application/json" \
  -d '{"topic": "ROS 2", "role": "Python developer"}'

# Expected: HTTP 200 with explanation (no regression)
```

## Related Endpoints (No Changes)

### /chat Endpoint
- **Model**: `ChatRequest(BaseModel)` with `question: str`
- **Status**: Unchanged
- **Rationale**: No frontend compatibility issues reported

### /translate Endpoint
- **Model**: `TranslateRequest(BaseModel)` with `text: str`
- **Status**: Unchanged
- **Rationale**: No frontend compatibility issues reported

## Summary

**Key Change**: Replace strict Pydantic validation with manual JSON extraction

**Benefits**:
- ✅ Accepts multiple key name variants (role, background, userRole)
- ✅ Graceful fallback to "student" default
- ✅ Backward compatible with existing "role" key
- ✅ Enhanced debugging with full JSON logging
- ✅ No new dependencies
- ✅ Minimal code changes (~15 lines)

**Trade-offs**:
- ⚠️ Less type safety (manual validation vs automatic)
- ⚠️ No IDE autocomplete for field access
- ⚠️ Slightly more verbose extraction code

**Validation**: All trade-offs acceptable for this use case (2-field endpoint with flexibility requirement)
