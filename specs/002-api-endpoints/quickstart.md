# Quickstart Guide: Testing the API Endpoints

**Feature**: 002-api-endpoints
**Date**: 2025-12-17
**Purpose**: Guide for manually testing the /translate and /personalize endpoints

## Prerequisites

### Local Development
- Python 3.12+ installed
- Backend dependencies installed (`pip install -r backend/requirements.txt` or use `uv`)
- Environment variables configured in `backend/.env`:
  ```env
  OPENAI_API_KEY=sk-...
  COHERE_API_KEY=...
  QDRANT_URL=https://...
  QDRANT_API_KEY=...
  ```

### Production Testing
- No setup required
- Base URL: `https://physical-ai-backend-j8q9.onrender.com`

## Starting the Server (Local)

```bash
# Navigate to backend directory
cd backend

# Start the FastAPI server
python server.py

# Server will start on http://localhost:8000
# You should see:
# INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
```

## Test 1: Translation Endpoint

### Using curl (Command Line)

**Request**:
```bash
curl -X POST http://localhost:8000/translate \
  -H "Content-Type: application/json" \
  -d '{"text": "ROS 2 is a robot operating system that provides libraries and tools to help build robot applications."}'
```

**Expected Response** (HTTP 200):
```json
{
  "translation": "ROS 2 ایک robot operating system ہے جو libraries اور tools فراہم کرتا ہے robot applications بنانے میں مدد کے لیے۔"
}
```

**Validation Checks**:
- ✅ HTTP status code is 200
- ✅ Response contains `translation` field
- ✅ Technical terms (ROS 2, robot, operating system, libraries, tools, applications) remain in English
- ✅ Surrounding text is in formal Urdu script
- ✅ Response time is 2-5 seconds (similar to /chat endpoint)

### Using Python requests

```python
import requests

url = "http://localhost:8000/translate"
payload = {
    "text": "ROS 2 is a robot operating system that provides libraries and tools to help build robot applications."
}

response = requests.post(url, json=payload)
print(f"Status: {response.status_code}")
print(f"Response: {response.json()}")
```

### Using Postman

1. **Method**: POST
2. **URL**: `http://localhost:8000/translate`
3. **Headers**:
   - Content-Type: `application/json`
4. **Body** (raw JSON):
   ```json
   {
     "text": "ROS 2 is a robot operating system that provides libraries and tools to help build robot applications."
   }
   ```
5. **Send** → Expect HTTP 200 with translation response

### Edge Cases to Test

**Test 1a: Empty text field**
```bash
curl -X POST http://localhost:8000/translate \
  -H "Content-Type: application/json" \
  -d '{"text": ""}'
```
Expected: HTTP 500 or translation result (Pydantic allows empty strings; OpenAI may handle gracefully)

**Test 1b: Missing text field**
```bash
curl -X POST http://localhost:8000/translate \
  -H "Content-Type: application/json" \
  -d '{}'
```
Expected: HTTP 422 Validation Error
```json
{
  "detail": [
    {
      "loc": ["body", "text"],
      "msg": "field required",
      "type": "value_error.missing"
    }
  ]
}
```

**Test 1c: Text with code blocks**
```bash
curl -X POST http://localhost:8000/translate \
  -H "Content-Type: application/json" \
  -d '{"text": "The publisher sends messages using pub.publish(msg). This is a core ROS 2 pattern."}'
```
Expected: Code reference `pub.publish(msg)` should remain in English

**Test 1d: Already Urdu text**
```bash
curl -X POST http://localhost:8000/translate \
  -H "Content-Type: application/json" \
  -d '{"text": "ROS 2 ایک framework ہے۔"}'
```
Expected: Agent may return it as-is or "back-translate" to English (undefined behavior, acceptable)

## Test 2: Personalization Endpoint

### Using curl (Command Line)

**Request**:
```bash
curl -X POST http://localhost:8000/personalize \
  -H "Content-Type: application/json" \
  -d '{"topic": "URDF files", "role": "Python developer"}'
```

**Expected Response** (HTTP 200):
```json
{
  "explanation": "URDF files are like XML config files in Python projects - they define your robot's structure using tags and attributes. Just as you'd use dictionaries to represent nested data in Python, URDF uses nested XML elements to describe links (robot parts) and joints (connections)."
}
```

**Validation Checks**:
- ✅ HTTP status code is 200
- ✅ Response contains `explanation` field
- ✅ Explanation is 2-3 sentences (per FR-006 and SC-004)
- ✅ Explanation uses analogies relevant to "Python developer" role
- ✅ Response time is 2-5 seconds (similar to /chat endpoint)

### Using Python requests

```python
import requests

url = "http://localhost:8000/personalize"
payload = {
    "topic": "ROS 2 Publishers",
    "role": "Arduino hobbyist"
}

response = requests.post(url, json=payload)
print(f"Status: {response.status_code}")
print(f"Response: {response.json()}")
```

**Expected**: Explanation comparing ROS 2 Publishers to Arduino Serial.print() or similar Arduino concepts

### Using Postman

1. **Method**: POST
2. **URL**: `http://localhost:8000/personalize`
3. **Headers**:
   - Content-Type: `application/json`
4. **Body** (raw JSON):
   ```json
   {
     "topic": "URDF files",
     "role": "Python developer"
   }
   ```
5. **Send** → Expect HTTP 200 with explanation response

### Edge Cases to Test

**Test 2a: Missing role field**
```bash
curl -X POST http://localhost:8000/personalize \
  -H "Content-Type: application/json" \
  -d '{"topic": "ROS 2 Publishers"}'
```
Expected: HTTP 422 Validation Error
```json
{
  "detail": [
    {
      "loc": ["body", "role"],
      "msg": "field required",
      "type": "value_error.missing"
    }
  ]
}
```

**Test 2b: Missing topic field**
```bash
curl -X POST http://localhost:8000/personalize \
  -H "Content-Type: application/json" \
  -d '{"role": "Python developer"}'
```
Expected: HTTP 422 Validation Error

**Test 2c: Unconventional role**
```bash
curl -X POST http://localhost:8000/personalize \
  -H "Content-Type: application/json" \
  -d '{"topic": "LIDAR sensors", "role": "underwater basket weaver"}'
```
Expected: HTTP 200 with best-effort explanation (agent may create creative analogies or say it cannot relate)

**Test 2d: Topic not in handbook**
```bash
curl -X POST http://localhost:8000/personalize \
  -H "Content-Type: application/json" \
  -d '{"topic": "quantum computing", "role": "physicist"}'
```
Expected: HTTP 200 with agent response (may say "I cannot answer this based on the handbook context.")

## Test 3: Verify Existing /chat Endpoint Still Works

**Request**:
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```

**Expected Response** (HTTP 200):
```json
{
  "answer": "ROS 2 (Robot Operating System 2) is a modern robotics framework designed for distributed systems..."
}
```

**Validation**: FR-007 requires existing /chat endpoint functionality is preserved without modification

## Production Testing

Replace `http://localhost:8000` with `https://physical-ai-backend-j8q9.onrender.com` in all curl commands.

**Example**:
```bash
curl -X POST https://physical-ai-backend-j8q9.onrender.com/translate \
  -H "Content-Type: application/json" \
  -d '{"text": "ROS 2 nodes communicate using topics."}'
```

**Note**: First request after inactivity may take 30-60 seconds (Render free tier cold start). Subsequent requests will be fast.

## Troubleshooting

### Error: "404 Not Found"
- **Cause**: Endpoints not yet implemented in server.py
- **Solution**: Complete implementation per tasks.md (from /sp.tasks command)

### Error: "500 Internal Server Error"
- **Cause**: run_agent() failed (OpenAI API error, retrieval error, etc.)
- **Solution**: Check backend console logs for exception details; verify .env variables are set

### Error: "422 Unprocessable Entity"
- **Cause**: Request body missing required fields or wrong types
- **Solution**: Verify JSON structure matches TranslateRequest or PersonalizeRequest schema

### Response is very slow (>10 seconds)
- **Cause**: OpenAI API slow response or network issues
- **Solution**: Check OpenAI status page; verify internet connection; acceptable for MVP

### Translation doesn't preserve technical terms
- **Cause**: Agent may not be following LANGUAGE RULES correctly
- **Solution**: Verify SYSTEM_PROMPT in agent.py has not been modified; check OpenAI model version (should be gpt-4o)

## Success Criteria Validation

After running all tests above, verify:

- ✅ **SC-001**: Frontend requests to /translate receive HTTP 200 with JSON translation
- ✅ **SC-002**: Frontend requests to /personalize receive HTTP 200 with JSON explanation
- ✅ **SC-003**: Translation results preserve English technical keywords
- ✅ **SC-004**: Personalization explanations are 2-3 sentences
- ✅ **SC-005**: Existing /chat endpoint still works (Test 3)
- ✅ **SC-006**: Invalid requests return HTTP 422 (Tests 1b, 2a, 2b)
- ✅ **SC-007**: Response time matches /chat endpoint (~2-5 seconds)

## Next Steps

After manual testing validates all success criteria:

1. Merge feature branch to main
2. Verify Render auto-deploys to production
3. Update frontend components to call new endpoints
4. Add integration tests (optional, out of current scope)
