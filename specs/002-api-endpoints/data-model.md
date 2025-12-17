# Data Model: API Request and Response Structures

**Feature**: 002-api-endpoints
**Date**: 2025-12-17
**Purpose**: Define the data structures for /translate and /personalize endpoints

## Overview

This feature introduces four data entities: two request models for incoming data validation and two response structures for outgoing data. All models follow the existing Pydantic pattern established by ChatRequest in server.py.

## Request Models

### TranslateRequest

**Purpose**: Validates incoming translation requests to /translate endpoint

**Fields**:
| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| text | str | Yes | Non-empty string | The technical English text to be translated to formal Urdu |

**Pydantic Definition**:
```python
class TranslateRequest(BaseModel):
    text: str
```

**Validation Rules**:
- ✅ Pydantic automatically validates `text` field is present
- ✅ FastAPI returns HTTP 422 if field is missing or wrong type
- ℹ️ No explicit length validation (OpenAI API has token limits ~8k for GPT-4o context)
- ℹ️ No explicit content filtering (assume educational use case)

**Example Valid Request**:
```json
{
  "text": "ROS 2 is a robot operating system that provides libraries and tools to help build robot applications."
}
```

**Example Invalid Request** (missing text field):
```json
{
  "content": "Some text"
}
```
→ Returns HTTP 422: `{"detail": [{"loc": ["body", "text"], "msg": "field required", "type": "value_error.missing"}]}`

---

### PersonalizeRequest

**Purpose**: Validates incoming personalization requests to /personalize endpoint

**Fields**:
| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| topic | str | Yes | Non-empty string | The robotics/AI topic to be explained |
| role | str | Yes | Non-empty string | The user's background/expertise level |

**Pydantic Definition**:
```python
class PersonalizeRequest(BaseModel):
    topic: str
    role: str
```

**Validation Rules**:
- ✅ Pydantic validates both fields are present and strings
- ✅ FastAPI returns HTTP 422 if either field is missing or wrong type
- ℹ️ No enum restriction on `role` - allow flexible user backgrounds
- ℹ️ No explicit topic validation - trust agent to handle any topic

**Example Valid Request**:
```json
{
  "topic": "URDF files",
  "role": "Python developer"
}
```

**Example Invalid Request** (missing role field):
```json
{
  "topic": "ROS 2 Publishers"
}
```
→ Returns HTTP 422: `{"detail": [{"loc": ["body", "role"], "msg": "field required", "type": "value_error.missing"}]}`

## Response Structures

### TranslateResponse

**Purpose**: Format for successful translation responses

**Structure**:
```python
# Python dict literal (no Pydantic model needed)
{
    "translation": str  # The Urdu translation with preserved English technical terms
}
```

**Example Success Response** (HTTP 200):
```json
{
  "translation": "ROS 2 ایک robot operating system ہے جو libraries اور tools فراہم کرتا ہے جو robot applications بنانے میں مدد کرتے ہیں۔"
}
```

**Characteristics**:
- ✅ Technical terms (ROS 2, robot, operating system, libraries, tools, applications) remain in English
- ✅ Surrounding grammatical structure is formal Urdu
- ✅ Follows LANGUAGE RULES from agent.py SYSTEM_PROMPT (lines 30-50)

**Example Error Response** (HTTP 500):
```json
{
  "detail": "Response generation error: OpenAI API rate limit exceeded"
}
```

---

### PersonalizeResponse

**Purpose**: Format for successful personalization responses

**Structure**:
```python
# Python dict literal (no Pydantic model needed)
{
    "explanation": str  # 2-3 sentence contextualized explanation
}
```

**Example Success Response** (HTTP 200):
```json
{
  "explanation": "URDF files are like XML config files in Python projects - they define your robot's structure using tags and attributes. Just as you'd use dictionaries to represent nested data in Python, URDF uses nested XML elements to describe links (robot parts) and joints (connections)."
}
```

**Characteristics**:
- ✅ 2-3 sentences maximum per specification (FR-006, SC-004)
- ✅ Uses analogies relevant to the user's `role` background
- ✅ Focuses on conceptual understanding, not implementation details

**Example Error Response** (HTTP 500):
```json
{
  "detail": "Error searching the handbook."
}
```

## Entity Relationships

```text
Client Request
     │
     ├──> POST /translate
     │         │
     │         ├──> TranslateRequest (validation)
     │         ├──> run_agent(formatted_query)
     │         └──> TranslateResponse
     │
     └──> POST /personalize
               │
               ├──> PersonalizeRequest (validation)
               ├──> run_agent(formatted_query)
               └──> PersonalizeResponse
```

## State Transitions

**N/A** - These are stateless request-response endpoints. No persistent state, sessions, or workflows.

Each request is independent:
1. Receive request → Validate → Process → Return response
2. No user sessions or conversation history
3. No database updates or side effects

## Validation Summary

| Scenario | Request Model | Validation Result | HTTP Status |
|----------|--------------|-------------------|-------------|
| Valid translation | TranslateRequest with text | ✅ Pass | 200 (on success) or 500 (on agent error) |
| Missing text field | TranslateRequest | ❌ Fail | 422 Unprocessable Entity |
| Valid personalization | PersonalizeRequest with topic + role | ✅ Pass | 200 (on success) or 500 (on agent error) |
| Missing topic field | PersonalizeRequest | ❌ Fail | 422 Unprocessable Entity |
| Missing role field | PersonalizeRequest | ❌ Fail | 422 Unprocessable Entity |
| Wrong field types | Any model | ❌ Fail | 422 Unprocessable Entity |

## Implementation Notes

### Why No Response Models?

Following the existing /chat endpoint pattern (server.py line 36):
```python
return {"answer": response}  # Dict literal, not Pydantic model
```

FastAPI automatically:
- ✅ Serializes dict to JSON
- ✅ Sets Content-Type: application/json
- ✅ Returns HTTP 200 by default

Creating response models would add unnecessary boilerplate without functional benefit for these simple single-field responses.

### Why No Field Length Limits?

- OpenAI GPT-4o has ~8k token context limit (enforced by API)
- For educational platform with trusted users, explicit length validation is unnecessary
- If needed later, add Pydantic field validators:
  ```python
  from pydantic import Field

  class TranslateRequest(BaseModel):
      text: str = Field(..., max_length=5000)
  ```

### Why No Custom Error Messages?

Pydantic's default error messages are clear and include:
- ✅ Field location (`["body", "text"]`)
- ✅ Error type (`value_error.missing`, `type_error.str`)
- ✅ Human-readable message (`field required`)

Custom messages would require extra code without improving UX.
