---
description: "Task list for fixing personalize endpoint validation error"
---

# Tasks: Fix Personalize Endpoint Validation Error

**Input**: Design documents from `/specs/003-fix-personalize-validation/`
**Prerequisites**: plan.md (tech stack), spec.md (user story), research.md (decisions), data-model.md (flexible validation), contracts/openapi.yaml (API spec), quickstart.md (test scenarios)

**Tests**: Not requested in specification. Manual testing via quickstart.md only.

**Organization**: This is a single-story bug fix. Tasks organized sequentially by implementation phase.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: User Story 1 (US1) - Frontend Compatibility with Flexible JSON Keys
- Include exact file paths in descriptions

## Path Conventions

This is a web application project per plan.md:
- Backend: `backend/` (Python 3.12+, FastAPI)
- Frontend: `frontend/` (out of scope for this feature)

## Phase 1: Setup (Verification)

**Purpose**: Verify existing infrastructure is ready for modification

- [x] T001 Verify Python 3.12+ installed and backend environment configured
- [x] T002 Verify FastAPI dependencies present: fastapi, pydantic, python-dotenv, uvicorn in backend/requirements.txt
- [x] T003 Verify backend/server.py exists with current /personalize endpoint (lines 62-74)
- [x] T004 Verify backend/agent.py has run_agent async function unchanged (line 65)
- [x] T005 Read research.md to understand Request object approach and priority order (role → background → userRole → "student")

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: No additional foundational work needed - all infrastructure already exists

**⚠️ CRITICAL**: The existing backend infrastructure is complete. FastAPI is installed, Request object is built-in, run_agent function works, error handling patterns exist.

**Checkpoint**: Foundation ready - implementation can begin immediately

---

## Phase 3: User Story 1 - Frontend Compatibility with Flexible JSON Keys (Priority: P1) 🎯

**Goal**: Fix /personalize endpoint to accept flexible JSON key names (role, background, userRole) instead of strict Pydantic validation that returns 422 errors. Enable frontend compatibility while maintaining backward compatibility with existing "role" key.

**Independent Test**: Send POST requests to /personalize with different JSON key names (role, background, userRole, missing) and verify all return HTTP 200 with personalized explanations instead of 422 errors. Test using quickstart.md Test 1-4 scenarios.

### Implementation for User Story 1

- [x] T006 [US1] Add Request to FastAPI imports in backend/server.py line 1: `from fastapi import FastAPI, HTTPException, Request`
- [x] T007 [US1] Comment out PersonalizeRequest Pydantic model in backend/server.py lines 28-30 (preserve for reference, add comment explaining why removed)
- [x] T008 [US1] Rewrite personalize_endpoint function signature in backend/server.py line 62: Change from `async def personalize_endpoint(request: PersonalizeRequest)` to `async def personalize_endpoint(request: Request)`
- [x] T009 [US1] Add JSON extraction in personalize_endpoint (after line 63): `data = await request.json()`
- [x] T010 [US1] Add logging statement in personalize_endpoint: `print(f"📩 Received Personalization Request: {data}")`
- [x] T011 [US1] Extract topic field in personalize_endpoint: `topic = data.get("topic")`
- [x] T012 [US1] Add topic validation in personalize_endpoint: `if not topic: raise HTTPException(status_code=422, detail="Missing required field: topic")`
- [x] T013 [US1] Extract role field with priority fallback in personalize_endpoint: `role = data.get("role") or data.get("background") or data.get("userRole") or "student"`
- [x] T014 [US1] Add extracted values logging: `print(f"📝 Extracted - Topic: '{topic}', Role: '{role}'")`
- [x] T015 [US1] Update query formatting to use extracted variables (replace `request.topic` with `topic`, `request.role` with `role`)
- [x] T016 [US1] Verify existing run_agent call unchanged: `response = await run_agent(query)`
- [x] T017 [US1] Verify existing error handling preserved: try-except block with HTTPException(status_code=500)
- [x] T018 [US1] Verify existing response format maintained: `return {"explanation": response}`

**Checkpoint**: At this point, the /personalize endpoint should accept all key variants and default gracefully. Test independently per quickstart.md before proceeding.

---

## Phase 4: Manual Testing (User Story 1 Validation)

**Purpose**: Verify all acceptance scenarios from spec.md and quickstart.md

**⚠️ NOTE**: These tasks require backend server running (`python backend/server.py`)

### Basic Functionality Tests (quickstart.md Tests 1-4)

- [ ] T019 [US1] Test /personalize with "role" key (backward compatibility) - quickstart.md Test 1: Send `{"topic": "ROS 2 Nodes", "role": "Python developer"}`, verify HTTP 200 with personalized explanation
- [ ] T020 [US1] Test /personalize with "background" key - quickstart.md Test 2: Send `{"topic": "URDF files", "background": "Arduino hobbyist"}`, verify HTTP 200 (not 422) with Arduino-specific explanation
- [ ] T021 [US1] Test /personalize with "userRole" key - quickstart.md Test 3: Send `{"topic": "LIDAR sensors", "userRole": "JavaScript developer"}`, verify HTTP 200 (not 422) with JavaScript-specific explanation
- [ ] T022 [US1] Test /personalize with missing role key - quickstart.md Test 4: Send `{"topic": "Servos"}`, verify HTTP 200 with general "student" explanation (not 422 error)

### Edge Case Tests (quickstart.md Tests 5-10)

- [ ] T023 [US1] Test missing topic field - quickstart.md Test 5: Send `{"role": "Python developer"}`, verify HTTP 422 with "Missing required field: topic" message
- [ ] T024 [US1] Test empty topic field - quickstart.md Test 6: Send `{"topic": "", "role": "Python developer"}`, verify HTTP 422 error
- [ ] T025 [US1] Test empty role field - quickstart.md Test 7: Send `{"topic": "IMU sensors", "role": ""}`, verify HTTP 200 with "student" default
- [ ] T026 [US1] Test multiple role keys - quickstart.md Test 8: Send `{"topic": "TF2 transforms", "role": "C++ developer", "background": "Python developer"}`, verify HTTP 200 using "C++ developer" (priority order)
- [ ] T027 [US1] Test special characters - quickstart.md Test 9: Send `{"topic": "C++ & ROS 2", "role": "Python/JavaScript developer"}`, verify HTTP 200
- [ ] T028 [US1] Test malformed JSON - quickstart.md Test 10: Send `{topic: "ROS 2"}`, verify HTTP 422 from FastAPI's JSON parser

### Regression Tests (quickstart.md Tests 11-12)

- [ ] T029 Verify /chat endpoint unchanged - quickstart.md Test 11: Send `{"question": "What is ROS 2?"}` to /chat, verify HTTP 200 with answer
- [ ] T030 Verify /translate endpoint unchanged - quickstart.md Test 12: Send `{"text": "ROS 2 is a robotics framework"}` to /translate, verify HTTP 200 with Urdu translation

### Console Logging Validation

- [ ] T031 [US1] Verify console logs show full JSON data for all requests - Check that each request to /personalize prints received JSON object before processing
- [ ] T032 [US1] Verify console logs show extracted topic and role values - Check that print statement shows final extracted values after fallback logic

### Performance Validation

- [ ] T033 [US1] Measure response times for /personalize endpoint - Verify response times remain ~2-5 seconds (OpenAI API call), not significantly degraded by manual extraction

**Checkpoint**: All 15 test scenarios should pass. Console logs should show debugging information. Response times should match existing /chat endpoint performance.

---

## Phase 5: Code Quality & Documentation

**Purpose**: Validation and verification across implementation

- [x] T034 Review backend/server.py for code quality: Verify consistent error handling, logging patterns match existing /chat endpoint style (lines 38, 45), async/await used correctly
- [x] T035 Verify no unintended changes to other files - Run `git diff backend/agent.py backend/retriveing.py backend/main.py` and confirm no output (files unchanged)
- [x] T036 Verify PersonalizeRequest model commented with explanation - Check lines 28-30 have comment like "# Removed to allow flexible JSON keys - see research.md"
- [x] T037 Verify all success criteria from spec.md met: SC-001 through SC-007 validated via test results (T019-T033)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - verification tasks can start immediately in parallel (T001-T005)
- **Foundational (Phase 2)**: N/A - No foundational work needed, existing infrastructure sufficient
- **User Story 1 (Phase 3)**: Can start immediately after Setup verification (T001-T005)
- **Testing (Phase 4)**: Depends on User Story 1 implementation completion (T006-T018)
- **Quality (Phase 5)**: Depends on all tests passing (T019-T033)

### User Story Dependencies

- **User Story 1 (P1)**: INDEPENDENT - This is a single-story bug fix with no dependencies on other features
  - Can be implemented, tested, and deployed standalone
  - Delivers: Flexible JSON key acceptance for /personalize endpoint

### Within User Story 1

**Sequential Flow** (tasks must run in order):
1. T006 (add import) → T007 (comment model) → T008 (change signature) → T009 (add JSON extraction) → T010-T014 (logging and extraction) → T015-T018 (integrate with existing logic)

**Parallel Opportunities**:
- Setup tasks T001-T005 can run in parallel (different verification checks)
- Testing tasks T019-T028 can run in parallel after implementation complete (independent test scenarios)
- Regression tests T029-T030 can run in parallel with User Story 1 tests

### Parallel Opportunities

- **Phase 1**: All setup verification tasks (T001-T005) can run in parallel
- **Phase 3**: Implementation tasks MUST run sequentially (modifying same file, dependencies between changes)
- **Phase 4**: All test scenarios (T019-T028) can run in parallel after implementation
- **Phase 5**: Code quality checks (T034-T037) can run in parallel

---

## Parallel Example: Testing Phase

```bash
# Launch all basic functionality tests in parallel (T019-T022):
Task: "Test /personalize with 'role' key (backward compatibility) - quickstart.md Test 1"
Task: "Test /personalize with 'background' key - quickstart.md Test 2"
Task: "Test /personalize with 'userRole' key - quickstart.md Test 3"
Task: "Test /personalize with missing role key - quickstart.md Test 4"

# Launch all edge case tests in parallel (T023-T028):
Task: "Test missing topic field - quickstart.md Test 5"
Task: "Test empty topic field - quickstart.md Test 6"
Task: "Test empty role field - quickstart.md Test 7"
Task: "Test multiple role keys - quickstart.md Test 8"
Task: "Test special characters - quickstart.md Test 9"
Task: "Test malformed JSON - quickstart.md Test 10"

# Launch regression tests in parallel (T029-T030):
Task: "Verify /chat endpoint unchanged - quickstart.md Test 11"
Task: "Verify /translate endpoint unchanged - quickstart.md Test 12"
```

---

## Implementation Strategy

### Sequential Implementation (Single Developer)

1. Complete Phase 1: Setup (T001-T005) - verify dependencies → ~5 minutes
2. Skip Phase 2: Foundational (no work needed)
3. Complete Phase 3: User Story 1 Implementation (T006-T018) - modify backend/server.py → ~10-15 minutes
4. **CHECKPOINT**: Start backend server (`python backend/server.py`)
5. Complete Phase 4: Manual Testing (T019-T033) - run all quickstart.md tests → ~15-20 minutes
6. Complete Phase 5: Code Quality (T034-T037) - verify code standards and git diff → ~5 minutes
7. **STOP and VALIDATE**: All tests pass, console logs visible, no regressions

**Total Estimated Time**: 35-45 minutes from start to verified completion

### MVP Scope (This Entire Feature)

This is a single-story bug fix with no incremental delivery options. The entire feature (flexible JSON key acceptance) must be implemented together:

1. Complete Setup (T001-T005)
2. Complete Implementation (T006-T018)
3. Complete Testing (T019-T033)
4. Complete Quality Checks (T034-T037)
5. **Commit and Deploy**: All acceptance scenarios validated

**MVP Deliverable**: /personalize endpoint accepts role/background/userRole keys and defaults to "student" when missing, fixing 422 errors reported by frontend.

### Deployment Strategy

After all tasks complete and tests pass:

1. **Commit Changes**:
   ```bash
   git add backend/server.py
   git commit -m "fix: Relax /personalize validation for flexible JSON keys

   - Accept role/background/userRole key variants with priority order
   - Default to 'student' when role missing or empty
   - Add full JSON logging for debugging frontend key mismatches
   - Maintain backward compatibility with existing 'role' key

   Fixes 422 errors when frontend sends alternative key names.
   All 12 test scenarios pass (quickstart.md Tests 1-12).

   Related: 003-fix-personalize-validation"
   ```

2. **Push to Branch**:
   ```bash
   git push origin 003-fix-personalize-validation
   ```

3. **Deploy to Render** (auto-deploys from branch):
   - Wait 2-3 minutes for Render build
   - Run quickstart.md tests against production URL: `https://physical-ai-backend-j8q9.onrender.com`
   - Verify first request (cold start) takes 60-90 seconds, subsequent requests 2-5 seconds

4. **Merge to Main** (after production validation):
   ```bash
   git checkout main
   git merge 003-fix-personalize-validation
   git push origin main
   ```

5. **Frontend Integration**:
   - Notify frontend team endpoint now accepts any key variant
   - Frontend can use background/userRole without changes if already using those keys
   - No frontend code changes required

---

## Notes

### Key Implementation Patterns (from research.md)

**Import Change** (T006):
```python
from fastapi import FastAPI, HTTPException, Request
```

**Model Removal** (T007):
```python
# PersonalizeRequest Pydantic model removed to allow flexible JSON keys
# Frontend sends various key names: role, background, userRole
# See research.md for decision rationale
# class PersonalizeRequest(BaseModel):
#     topic: str
#     role: str
```

**Endpoint Rewrite** (T008-T018):
```python
@app.post("/personalize")
async def personalize_endpoint(request: Request):
    # Get raw JSON
    data = await request.json()

    # Log for debugging (FR-003)
    print(f"📩 Received Personalization Request: {data}")

    # Extract topic (required)
    topic = data.get("topic")
    if not topic:
        raise HTTPException(status_code=422, detail="Missing required field: topic")

    # Extract role with fallback logic (FR-002)
    role = data.get("role") or data.get("background") or data.get("userRole") or "student"

    print(f"📝 Extracted - Topic: '{topic}', Role: '{role}'")

    try:
        # Format query (FR-004)
        query = f"Explain {topic} for someone with {role} background in 2-3 sentences using relevant analogies"
        # Call agent (FR-004)
        response = await run_agent(query)
        # Return response (FR-005)
        return {"explanation": response}
    except Exception as e:
        print(f"❌ Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))
```

### Testing Notes

- No automated tests requested in specification
- Use quickstart.md manual testing procedures with curl
- Test /personalize with Tests 1-10 scenarios (all key variants + edge cases)
- Verify /chat and /translate still work (regression tests 11-12)
- Expected response time: 2-5 seconds (OpenAI GPT-4o completion)
- Console logging should show all received data and extracted values

### Code Quality Checks

- Consistent with existing server.py style (lines 36-74 pattern)
- Print statements for debugging (per existing pattern lines 38, 50, 64)
- Error messages include exception details (line 45, 59, 73 pattern)
- FastAPI decorators (@app.post) match existing pattern
- Async/await used correctly (all handlers are async def)
- No modifications to agent.py, retriveing.py, or other files

### Success Criteria Validation (from spec.md)

- ✅ SC-001: Requests with "background" key return HTTP 200 (T020)
- ✅ SC-002: Requests with "userRole" key return HTTP 200 (T021)
- ✅ SC-003: Requests with "role" key continue to work (T019)
- ✅ SC-004: Requests without role key default to "student" (T022)
- ✅ SC-005: All requests log JSON data to console (T031)
- ✅ SC-006: Existing endpoints function without changes (T029-T030)
- ✅ SC-007: Response times match previous performance (T033)

### Deployment

- After merge to main: Render auto-deploys to https://physical-ai-backend-j8q9.onrender.com
- Test production endpoints using quickstart.md with production URL
- First request may take 60-90s (Render free tier cold start)
- Frontend integration is out of scope for this feature (backend-only fix)

---

## Task Count Summary

- **Phase 1 (Setup)**: 5 tasks (T001-T005)
- **Phase 2 (Foundational)**: 0 tasks (infrastructure exists)
- **Phase 3 (User Story 1 Implementation)**: 13 tasks (T006-T018)
- **Phase 4 (User Story 1 Testing)**: 15 tasks (T019-T033)
- **Phase 5 (Code Quality)**: 4 tasks (T034-T037)

**Total**: 37 tasks

**Parallel Opportunities**:
- 5 tasks can run in parallel (Setup verification: T001-T005)
- 12 tasks can run in parallel (Test scenarios: T019-T028, T029-T030)
- 4 tasks can run in parallel (Quality checks: T034-T037)

**Sequential Requirements**: Implementation tasks (T006-T018) must run sequentially (same file modifications with dependencies)

**MVP Scope**: All 37 tasks (single-story bug fix, cannot be decomposed)

**Estimated Total Time**: 35-45 minutes (implementation + testing + quality checks)
