---
description: "Task list for implementing API endpoints feature"
---

# Tasks: Backend API Endpoints - Translation and Personalization

**Input**: Design documents from `/specs/002-api-endpoints/`
**Prerequisites**: plan.md (tech stack), spec.md (user stories), research.md (decisions), data-model.md (models), contracts/openapi.yaml (API spec)

**Tests**: Not requested in specification. Manual testing via quickstart.md only.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2)
- Include exact file paths in descriptions

## Path Conventions

This is a web application project per plan.md:
- Backend: `backend/` (Python 3.12+, FastAPI)
- Frontend: `frontend/` (out of scope for this feature)

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify project structure and dependencies are ready

- [x] T001 Verify Python 3.12+ installed and backend/pyproject.toml configuration
- [x] T002 Verify all dependencies present: FastAPI 0.124.0+, OpenAI 2.9.0+, Pydantic, uvicorn 0.38.0+ in backend/requirements.txt and backend/pyproject.toml
- [x] T003 Verify backend/agent.py exists with run_agent async function (line 65)
- [x] T004 Verify backend/server.py has existing /chat endpoint pattern to replicate (lines 29-39)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: No additional foundational work needed - all infrastructure already exists

**⚠️ CRITICAL**: The existing backend infrastructure is complete. run_agent function, CORS middleware, error handling patterns, and Pydantic imports are already in place per research.md.

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Technical Text Translation (Priority: P1) 🎯 MVP

**Goal**: Implement POST /translate endpoint that accepts English technical text and returns formal Urdu translation with preserved English technical keywords (ROS 2, Python, Node, etc.)

**Independent Test**: Send POST request to /translate with technical text; verify response contains Urdu translation with English keywords preserved. Test using quickstart.md Test 1 procedures.

### Implementation for User Story 1

- [x] T005 [US1] Add TranslateRequest Pydantic model after ChatRequest class in backend/server.py (class TranslateRequest(BaseModel): text: str)
- [x] T006 [US1] Implement POST /translate endpoint handler in backend/server.py following /chat pattern (lines 29-39):
  - Define async function translate_endpoint(request: TranslateRequest)
  - Format prompt: f"Translate this technical text to formal Urdu, keeping English keywords: {request.text}"
  - Call: response = await run_agent(formatted_prompt)
  - Return: {"translation": response}
  - Wrap in try-except with HTTPException(status_code=500, detail=str(e))
  - Add print statements for logging (follow line 31, 38 pattern)
- [x] T007 [US1] Add @app.post("/translate") decorator above translate_endpoint function in backend/server.py
- [x] T008 [US1] Verify existing CORS middleware (lines 13-19) applies to new /translate endpoint (no changes needed)
- [ ] T009 [US1] Test /translate endpoint using quickstart.md Test 1 scenarios (curl + edge cases)

**Checkpoint**: At this point, User Story 1 should be fully functional. Translation endpoint returns Urdu text with preserved English technical terms. Test independently per acceptance scenarios before proceeding.

---

## Phase 4: User Story 2 - Personalized Learning Explanations (Priority: P2)

**Goal**: Implement POST /personalize endpoint that accepts topic + user role and returns 2-3 sentence contextualized explanation using analogies from user's background

**Independent Test**: Send POST request to /personalize with topic="URDF files" and role="Python developer"; verify response contains 2-3 sentence explanation with relevant analogies. Test using quickstart.md Test 2 procedures.

### Implementation for User Story 2

- [x] T010 [P] [US2] Add PersonalizeRequest Pydantic model after TranslateRequest class in backend/server.py (class PersonalizeRequest(BaseModel): topic: str, role: str)
- [x] T011 [US2] Implement POST /personalize endpoint handler in backend/server.py following /chat pattern (lines 29-39):
  - Define async function personalize_endpoint(request: PersonalizeRequest)
  - Format prompt: f"Explain {request.topic} for someone with {request.role} background in 2-3 sentences using relevant analogies"
  - Call: response = await run_agent(formatted_prompt)
  - Return: {"explanation": response}
  - Wrap in try-except with HTTPException(status_code=500, detail=str(e))
  - Add print statements for logging (follow line 31, 38 pattern)
- [x] T012 [US2] Add @app.post("/personalize") decorator above personalize_endpoint function in backend/server.py
- [x] T013 [US2] Verify existing CORS middleware (lines 13-19) applies to new /personalize endpoint (no changes needed)
- [ ] T014 [US2] Test /personalize endpoint using quickstart.md Test 2 scenarios (curl + edge cases)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently. Personalization endpoint returns contextualized explanations. Test independently per acceptance scenarios.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Validation and verification across all user stories

- [x] T015 Verify existing /chat endpoint still works (FR-007) using quickstart.md Test 3 (no regression)
- [ ] T016 Run all edge case tests from quickstart.md (Test 1a-1d for /translate, Test 2a-2d for /personalize) - REQUIRES MANUAL TESTING
- [ ] T017 Validate all 7 Success Criteria from spec.md - REQUIRES MANUAL TESTING:
  - SC-001: /translate returns HTTP 200 with JSON translation
  - SC-002: /personalize returns HTTP 200 with JSON explanation
  - SC-003: Translation preserves English technical keywords
  - SC-004: Personalization explanations are 2-3 sentences
  - SC-005: /chat endpoint unchanged ✅ VERIFIED
  - SC-006: Invalid requests return HTTP 422
  - SC-007: Response times match /chat (~2-5 seconds)
- [ ] T018 Test backend locally (python backend/server.py) and verify all endpoints respond correctly - REQUIRES MANUAL TESTING
- [x] T019 Review backend/server.py for code quality: consistent error handling, logging, and patterns matching existing /chat endpoint
- [x] T020 Verify no unintended changes to backend/agent.py, backend/retriveing.py, or other files (git diff)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - verification tasks can start immediately in parallel
- **Foundational (Phase 2)**: N/A - No foundational work needed, existing infrastructure sufficient
- **User Story 1 (Phase 3)**: Can start immediately after Setup verification (T001-T004)
- **User Story 2 (Phase 4)**: Can start in parallel with User Story 1 (independent)
  - T010 marked [P] because PersonalizeRequest model can be written at same time as TranslateRequest (T005)
  - T011-T014 must wait for T010 completion
- **Polish (Phase 5)**: Depends on both User Story 1 and User Story 2 completion

### User Story Dependencies

- **User Story 1 (P1)**: INDEPENDENT - No dependencies on User Story 2
  - Can be implemented, tested, and deployed alone as MVP
  - Delivers: Translation endpoint for Urdu accessibility

- **User Story 2 (P2)**: INDEPENDENT - No dependencies on User Story 1
  - Can be implemented in parallel with User Story 1
  - Delivers: Personalization endpoint for adaptive learning
  - Both stories use same infrastructure (run_agent, CORS, error handling) but don't interact

### Within Each User Story

**User Story 1 Sequential Flow**:
1. T005 (add model) → T006 (implement handler) → T007 (add decorator) → T008 (verify CORS) → T009 (test)

**User Story 2 Sequential Flow**:
1. T010 (add model) → T011 (implement handler) → T012 (add decorator) → T013 (verify CORS) → T014 (test)

**Parallel Opportunity**:
- T005 and T010 can run in parallel (different model definitions)
- T006-T009 (US1 implementation) and T011-T014 (US2 implementation) are independent - can be done by different developers simultaneously

### Parallel Opportunities

- **Phase 1**: All setup verification tasks (T001-T004) can run in parallel
- **Phase 3 + Phase 4**: User Story 1 and User Story 2 can be implemented in parallel:
  - Developer A: T005 → T006 → T007 → T008 → T009 (US1 complete)
  - Developer B: T010 → T011 → T012 → T013 → T014 (US2 complete)
- **Phase 5**: Validation tasks T015-T020 should run sequentially (each depends on previous completions)

---

## Parallel Example: User Story 1 + User Story 2

```bash
# Launch models in parallel (different definitions):
Task A: "Add TranslateRequest Pydantic model in backend/server.py"
Task B: "Add PersonalizeRequest Pydantic model in backend/server.py"

# Then implement each story's handler sequentially within that story,
# but both stories proceed in parallel:

Developer 1 (US1):
  T006 → T007 → T008 → T009

Developer 2 (US2):
  T011 → T012 → T013 → T014
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T004) - verify dependencies
2. Skip Phase 2: Foundational (no work needed)
3. Complete Phase 3: User Story 1 (T005-T009)
4. **STOP and VALIDATE**: Test /translate independently using quickstart.md Test 1
5. Verify acceptance scenarios: technical terms preserved, Urdu translation correct, error handling works
6. Deploy/demo if ready - this is minimal viable product for Urdu accessibility

**Estimated Effort**: ~30-40 lines of code, 1-2 hours implementation + testing

### Incremental Delivery

1. Complete Setup (T001-T004) → Foundation verified ✅
2. Add User Story 1 (T005-T009) → Test independently → Deploy/Demo (MVP! 🎯)
3. Add User Story 2 (T010-T014) → Test independently → Deploy/Demo
4. Complete Polish (T015-T020) → Full validation → Final deployment
5. Each story adds value without breaking previous stories

**Total Estimated Effort**: ~60-80 lines of code, 3-4 hours implementation + testing

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup (T001-T004) together - 15 minutes
2. Once verified, split:
   - **Developer A**: User Story 1 (T005-T009) - Translation endpoint
   - **Developer B**: User Story 2 (T010-T014) - Personalization endpoint
3. Both stories complete and test independently - ~2 hours each
4. Reconvene for Polish phase (T015-T020) - 30 minutes validation
5. **Total time**: ~2.5 hours (vs 4 hours sequential)

---

## Notes

### Key Implementation Patterns (from research.md)

**Pydantic Models**:
```python
class TranslateRequest(BaseModel):
    text: str

class PersonalizeRequest(BaseModel):
    topic: str
    role: str
```

**Endpoint Handler Pattern** (replicate from /chat, lines 29-39):
```python
@app.post("/translate")
async def translate_endpoint(request: TranslateRequest):
    print(f"📩 Received Translation Request: {request.text[:50]}...")

    try:
        # Format prompt with instruction
        query = f"Translate this technical text to formal Urdu, keeping English keywords: {request.text}"
        # Call existing agent
        response = await run_agent(query)
        return {"translation": response}
    except Exception as e:
        print(f"❌ Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))
```

**Error Handling**: Follow existing try-except HTTPException pattern (lines 33-39)

**CORS**: Existing middleware (lines 13-19) automatically applies to all routes

**Logging**: Use print statements matching existing pattern (lines 31, 38)

### Testing Notes

- No automated tests requested in specification
- Use quickstart.md manual testing procedures
- Test /translate with Test 1 scenarios (simple text, code blocks, empty fields, missing fields)
- Test /personalize with Test 2 scenarios (different roles, missing fields, unconventional roles)
- Verify /chat still works (regression test)
- Expected response time: 2-5 seconds (OpenAI GPT-4o completion)

### Code Quality Checks

- Consistent with existing server.py style
- Print statements for debugging (per existing pattern)
- Error messages include exception details
- FastAPI decorators (@app.post) match existing pattern
- Async/await used correctly (all handlers are async def)
- No modifications to agent.py, retriveing.py, or other files

### Deployment

- After merge to main: Render auto-deploys to https://physical-ai-backend-j8q9.onrender.com
- Test production endpoints using quickstart.md with production URL
- First request may take 30-60s (Render free tier cold start)
- Frontend integration is out of scope for this feature

---

## Task Count Summary

- **Phase 1 (Setup)**: 4 tasks (T001-T004)
- **Phase 2 (Foundational)**: 0 tasks (infrastructure exists)
- **Phase 3 (User Story 1 - P1)**: 5 tasks (T005-T009)
- **Phase 4 (User Story 2 - P2)**: 5 tasks (T010-T014)
- **Phase 5 (Polish)**: 6 tasks (T015-T020)

**Total**: 20 tasks

**Parallel Opportunities**: 2 tasks can run in parallel (T005 + T010), plus entire User Stories 1 and 2 are independent

**MVP Scope**: Tasks T001-T009 only (Setup + User Story 1) = 9 tasks for minimal viable product
