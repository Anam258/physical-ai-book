# Quickstart: Fix Personalize Endpoint Validation Error

**Feature**: 003-fix-personalize-validation
**Date**: 2025-12-17
**Estimated Time**: 15-20 minutes

## Prerequisites

- Python 3.12+ installed
- Backend dependencies installed (`pip install -r backend/requirements.txt`)
- `.env` file configured with `OPENAI_API_KEY`, `COHERE_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`
- Terminal/Command Prompt
- curl or Postman for API testing

## Quick Start (Local Development)

### 1. Start the Backend Server

```bash
# Navigate to backend directory
cd backend

# Run the server
python server.py
```

**Expected Output**:
```text
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
```

**Note**: First request may take 30-60 seconds if Qdrant cold starts

### 2. Test Original "role" Key (Backward Compatibility)

**Test 1: Verify existing functionality still works**

```bash
curl -X POST http://localhost:8000/personalize \
  -H "Content-Type: application/json" \
  -d '{"topic": "ROS 2 Nodes", "role": "Python developer"}'
```

**Expected Response** (HTTP 200):
```json
{
  "explanation": "ROS 2 Nodes are like Python modules - independent programs that communicate via topics. For a Python developer, think of each Node as a class instance that can publish/subscribe to message queues. The rclpy library provides a Pythonic interface similar to Flask's request handling."
}
```

**Expected Console Log**:
```text
📩 Received Personalization Request: {'topic': 'ROS 2 Nodes', 'role': 'Python developer'}
📝 Extracted - Topic: 'ROS 2 Nodes', Role: 'Python developer'
🤖 Thinking about: 'Explain ROS 2 Nodes for someone with Python developer background...'
📚 Searching handbook...
💡 Generating answer...
```

**Success Criteria**: HTTP 200 response with personalized explanation containing analogies to Python

---

### 3. Test "background" Key (New Variant)

**Test 2: Frontend sends "background" instead of "role"**

```bash
curl -X POST http://localhost:8000/personalize \
  -H "Content-Type: application/json" \
  -d '{"topic": "URDF files", "background": "Arduino hobbyist"}'
```

**Expected Response** (HTTP 200):
```json
{
  "explanation": "URDF files are like Arduino sketch configurations - they define your robot's structure in XML. Just as you declare pin modes in setup(), URDF declares links (body parts) and joints (connections). Think of it as the robot's blueprint that ROS 2 reads to understand your hardware."
}
```

**Expected Console Log**:
```text
📩 Received Personalization Request: {'topic': 'URDF files', 'background': 'Arduino hobbyist'}
📝 Extracted - Topic: 'URDF files', Role: 'Arduino hobbyist'
```

**Success Criteria**: HTTP 200 response (not 422), explanation tailored to Arduino background

---

### 4. Test "userRole" Key (CamelCase Variant)

**Test 3: Frontend sends "userRole" (JavaScript convention)**

```bash
curl -X POST http://localhost:8000/personalize \
  -H "Content-Type: application/json" \
  -d '{"topic": "LIDAR sensors", "userRole": "JavaScript developer"}'
```

**Expected Response** (HTTP 200):
```json
{
  "explanation": "LIDAR sensors are like JavaScript async functions that continuously emit distance measurements. Each scan returns an array of range values (similar to polling multiple Promise results). ROS 2 processes these streams just like handling real-time WebSocket data in Node.js."
}
```

**Expected Console Log**:
```text
📩 Received Personalization Request: {'topic': 'LIDAR sensors', 'userRole': 'JavaScript developer'}
📝 Extracted - Topic: 'LIDAR sensors', Role: 'JavaScript developer'
```

**Success Criteria**: HTTP 200 response (not 422), explanation tailored to JavaScript background

---

### 5. Test Missing Role Key (Default to "student")

**Test 4: Frontend sends only topic (no role key)**

```bash
curl -X POST http://localhost:8000/personalize \
  -H "Content-Type: application/json" \
  -d '{"topic": "Servos"}'
```

**Expected Response** (HTTP 200):
```json
{
  "explanation": "Servos are motors that can move to precise positions and hold them. Unlike regular DC motors that spin continuously, servos can rotate to specific angles (like 90 degrees). They're commonly used in robot joints because you can control exactly how far they turn."
}
```

**Expected Console Log**:
```text
📩 Received Personalization Request: {'topic': 'Servos'}
📝 Extracted - Topic: 'Servos', Role: 'student'
```

**Success Criteria**: HTTP 200 response, general explanation suitable for students (no 422 error)

---

## Edge Case Testing

### Test 5: Missing Topic Field (Should Return 422)

```bash
curl -X POST http://localhost:8000/personalize \
  -H "Content-Type: application/json" \
  -d '{"role": "Python developer"}'
```

**Expected Response** (HTTP 422):
```json
{
  "detail": "Missing required field: topic"
}
```

**Success Criteria**: HTTP 422 error with clear message (not 500 error)

---

### Test 6: Empty Topic Field (Should Return 422)

```bash
curl -X POST http://localhost:8000/personalize \
  -H "Content-Type: application/json" \
  -d '{"topic": "", "role": "Python developer"}'
```

**Expected Response** (HTTP 422):
```json
{
  "detail": "Missing required field: topic"
}
```

**Success Criteria**: HTTP 422 error (empty string treated as missing)

---

### Test 7: Empty Role Field (Should Default to "student")

```bash
curl -X POST http://localhost:8000/personalize \
  -H "Content-Type: application/json" \
  -d '{"topic": "IMU sensors", "role": ""}'
```

**Expected Response** (HTTP 200):
```json
{
  "explanation": "IMU sensors measure acceleration and rotation using accelerometers and gyroscopes. Think of them as the robot's inner ear - they tell the robot which way is up and how fast it's moving. These sensors help robots maintain balance and navigate without external references."
}
```

**Expected Console Log**:
```text
📩 Received Personalization Request: {'topic': 'IMU sensors', 'role': ''}
📝 Extracted - Topic: 'IMU sensors', Role: 'student'
```

**Success Criteria**: HTTP 200 response, defaults to "student" (empty string is falsy)

---

### Test 8: Multiple Role Keys (Priority Order)

```bash
curl -X POST http://localhost:8000/personalize \
  -H "Content-Type: application/json" \
  -d '{"topic": "TF2 transforms", "role": "C++ developer", "background": "Python developer", "userRole": "JavaScript developer"}'
```

**Expected Response** (HTTP 200):
```json
{
  "explanation": "TF2 transforms are like C++ class hierarchies with coordinate frame relationships. Each transform maintains a parent-child relationship similar to inheritance, with matrix operations comparable to operator overloading. The lookup() function acts like dynamic_cast for runtime coordinate conversions."
}
```

**Expected Console Log**:
```text
📩 Received Personalization Request: {'topic': 'TF2 transforms', 'role': 'C++ developer', 'background': 'Python developer', 'userRole': 'JavaScript developer'}
📝 Extracted - Topic: 'TF2 transforms', Role: 'C++ developer'
```

**Success Criteria**: Uses "role" value (C++ developer), ignores others

---

### Test 9: Special Characters in Fields

```bash
curl -X POST http://localhost:8000/personalize \
  -H "Content-Type: application/json" \
  -d '{"topic": "C++ & ROS 2", "role": "Python/JavaScript developer"}'
```

**Expected Response** (HTTP 200):
```json
{
  "explanation": "[Personalized explanation with C++ references]"
}
```

**Success Criteria**: HTTP 200 response, handles special characters correctly

---

### Test 10: Malformed JSON (Should Return 422)

```bash
curl -X POST http://localhost:8000/personalize \
  -H "Content-Type: application/json" \
  -d '{topic: "ROS 2"}'
```

**Expected Response** (HTTP 422):
```json
{
  "detail": "Expecting property name enclosed in double quotes: line 1 column 2 (char 1)"
}
```

**Success Criteria**: HTTP 422 error from FastAPI's JSON parser (before endpoint logic)

---

## Regression Testing

### Test 11: Verify /chat Endpoint Still Works

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```

**Expected Response** (HTTP 200):
```json
{
  "answer": "ROS 2 (Robot Operating System 2) is a modern robotics framework..."
}
```

**Success Criteria**: HTTP 200 response, /chat endpoint unaffected by changes

---

### Test 12: Verify /translate Endpoint Still Works

```bash
curl -X POST http://localhost:8000/translate \
  -H "Content-Type: application/json" \
  -d '{"text": "ROS 2 is a robotics framework"}'
```

**Expected Response** (HTTP 200):
```json
{
  "translation": "ROS 2 ایک robotics framework ہے جو distributed systems کے لیے design کیا گیا ہے..."
}
```

**Success Criteria**: HTTP 200 response, /translate endpoint unaffected by changes

---

## Production Testing (After Deployment)

Replace `http://localhost:8000` with `https://physical-ai-backend-j8q9.onrender.com` in all commands above.

**Example**:
```bash
curl -X POST https://physical-ai-backend-j8q9.onrender.com/personalize \
  -H "Content-Type: application/json" \
  -d '{"topic": "ROS 2 Nodes", "background": "Python developer"}'
```

**Note**: First production request may take 60-90 seconds (Render free tier cold start)

---

## Success Checklist

- [ ] Test 1: Original "role" key works (backward compatibility) ✅
- [ ] Test 2: "background" key returns HTTP 200 (not 422) ✅
- [ ] Test 3: "userRole" key returns HTTP 200 (not 422) ✅
- [ ] Test 4: Missing role key defaults to "student" ✅
- [ ] Test 5: Missing topic returns HTTP 422 ✅
- [ ] Test 6: Empty topic returns HTTP 422 ✅
- [ ] Test 7: Empty role defaults to "student" ✅
- [ ] Test 8: Multiple keys use priority order ✅
- [ ] Test 9: Special characters handled ✅
- [ ] Test 10: Malformed JSON returns 422 ✅
- [ ] Test 11: /chat endpoint unchanged ✅
- [ ] Test 12: /translate endpoint unchanged ✅
- [ ] Console logs show all received data ✅
- [ ] Response times remain ~2-5 seconds ✅

---

## Troubleshooting

### Issue: 500 Error Instead of 200

**Possible Causes**:
- OpenAI API key invalid or quota exceeded
- Qdrant connection failure
- run_agent function error

**Debug**:
```bash
# Check console logs for error messages
# Look for "❌ Error:" lines
# Verify .env file has valid API keys
```

### Issue: No Console Logs Visible

**Possible Causes**:
- Server not running in foreground
- Output buffering

**Solution**:
```bash
# Run with unbuffered output
python -u server.py
```

### Issue: 422 on Valid Request

**Possible Causes**:
- Changes not applied (old code still running)
- Import error (Request not imported)

**Solution**:
```bash
# Restart server
# Ctrl+C to stop
python server.py
```

---

## Next Steps

After all tests pass:

1. **Commit Changes**:
   ```bash
   git add backend/server.py
   git commit -m "Fix: Relax /personalize validation for flexible JSON keys

   - Accept role/background/userRole key variants
   - Default to 'student' when role missing
   - Add full JSON logging for debugging
   - Maintain backward compatibility with 'role' key

   Fixes #003"
   ```

2. **Push to Render**:
   ```bash
   git push origin 003-fix-personalize-validation
   ```

3. **Merge to Main** (after testing):
   ```bash
   git checkout main
   git merge 003-fix-personalize-validation
   git push origin main
   ```

4. **Verify Production Deployment**:
   - Wait 2-3 minutes for Render auto-deploy
   - Run production tests (see Production Testing section above)

5. **Frontend Integration**:
   - Update frontend to use any key variant (role, background, userRole)
   - No frontend changes required if already using one of these keys
   - Remove frontend-side validation that enforces specific key names

---

## Performance Expectations

- **Local Response Time**: 2-5 seconds (OpenAI API call + retrieval)
- **Production Cold Start**: 60-90 seconds (first request only)
- **Production Warm**: 2-5 seconds
- **Console Logging Overhead**: <10ms per request (negligible)

---

## Summary

This quickstart covers:
- ✅ Basic functionality tests (Tests 1-4)
- ✅ Edge case validation (Tests 5-10)
- ✅ Regression prevention (Tests 11-12)
- ✅ Production deployment verification
- ✅ Troubleshooting guide

All tests should pass with HTTP 200 responses (except intentional 422 tests) and console logs showing full JSON data for debugging.
