---
id: 003
title: fix-personalize-validation-tasks
stage: tasks
date: 2025-12-17
surface: slash-command
model: claude-sonnet-4-5
feature: 003-fix-personalize-validation
branch: 003-fix-personalize-validation
user: User
command: /sp.tasks
labels: [tasks, backend, api, validation, bug-fix, manual-testing]
links:
  spec: ../../../specs/003-fix-personalize-validation/spec.md
  plan: ../../../specs/003-fix-personalize-validation/plan.md
  tasks: ../../../specs/003-fix-personalize-validation/tasks.md
  ticket: null
  adr: null
  pr: null
files:
  - path: specs/003-fix-personalize-validation/tasks.md
    action: created
    description: Complete task list with 37 tasks organized across 5 phases
tests: []
---

## Prompt

(Invoked via `/sp.tasks` slash command with no additional arguments)

Execute the task generation workflow:
1. Check prerequisites and get feature context
2. Load design documents: plan.md, spec.md, research.md, data-model.md, contracts/, quickstart.md
3. Extract user stories from spec.md (priorities)
4. Map entities, endpoints, and decisions to tasks
5. Generate tasks organized by user story
6. Create dependency graph and parallel execution examples
7. Validate task completeness and format

## Response snapshot

Successfully generated comprehensive task list for fixing /personalize endpoint validation bug:

### Setup (Step 1)
- Ran `.specify/scripts/powershell/check-prerequisites.ps1 -Json`
- Retrieved FEATURE_DIR: `specs/003-fix-personalize-validation`
- Available docs: research.md, data-model.md, contracts/, quickstart.md
- Loaded tasks-template.md for structure

### Document Analysis (Step 2-3)

**From spec.md**:
- 1 user story: Frontend Compatibility with Flexible JSON Keys (P1 critical)
- 4 acceptance scenarios (role, background, userRole, missing role keys)
- 5 edge cases (empty fields, missing topic, multiple keys, malformed JSON, special chars)
- 8 functional requirements (FR-001 through FR-008)
- 7 success criteria (SC-001 through SC-007)

**From plan.md**:
- Language: Python 3.12+
- Framework: FastAPI 0.124.0+
- Project structure: Web application (backend/ directory)
- File to modify: backend/server.py (lines 1, 28-30, 62-74)
- Implementation: 3 code changes (~15 lines)
- Estimated time: 30-40 minutes

**From research.md**:
- Decision 1: Use Request object with manual JSON extraction
- Decision 2: HTTP 422 for missing topic, "student" default for role
- Decision 3: Log entire JSON object for debugging
- Decision 4: Priority order (role → background → userRole → "student")
- Decision 5: No changes to run_agent integration

**From data-model.md**:
- PersonalizeRequest (Flexible Input): topic (required), role (optional with 3 key variants)
- Validation rules: topic non-empty, role with fallback logic
- Edge case handling: 9 scenarios mapped
- Before/after code comparison provided

**From quickstart.md**:
- 12 comprehensive test scenarios:
  - Tests 1-4: Basic functionality (all key variants)
  - Tests 5-10: Edge cases (validation errors, empty strings, special chars)
  - Tests 11-12: Regression tests (/chat, /translate unchanged)
- curl commands with expected responses and console logs

### Task Generation (Step 4-6)

**Phase 1: Setup (5 tasks)**
- T001-T004: Verify Python, dependencies, existing code, run_agent function
- T005: Read research.md for implementation approach
- **Parallel**: All 5 tasks can run in parallel (independent verifications)

**Phase 2: Foundational (0 tasks)**
- No foundational work needed (infrastructure complete per plan.md)
- Checkpoint: Ready for immediate implementation

**Phase 3: User Story 1 Implementation (13 tasks)**
- T006: Add Request to imports (line 1)
- T007: Comment out PersonalizeRequest model (lines 28-30)
- T008-T014: Rewrite personalize_endpoint function (JSON extraction, logging, validation, fallback)
- T015-T018: Integrate with existing run_agent logic
- **Sequential**: Must run in order (same file, dependent changes)

**Phase 4: User Story 1 Testing (15 tasks)**
- T019-T022: Basic functionality tests (quickstart.md Tests 1-4)
- T023-T028: Edge case tests (quickstart.md Tests 5-10)
- T029-T030: Regression tests (quickstart.md Tests 11-12)
- T031-T033: Console logging and performance validation
- **Parallel**: All 15 tasks can run in parallel after implementation (independent test scenarios)

**Phase 5: Code Quality (4 tasks)**
- T034: Review code quality (error handling, logging patterns)
- T035: Verify no unintended file changes (git diff)
- T036: Verify model comment explanation
- T037: Verify all success criteria met
- **Parallel**: All 4 tasks can run in parallel (independent checks)

### Task Format Validation

All 37 tasks follow strict checklist format:
- ✅ Checkbox prefix: `- [ ]`
- ✅ Task ID: Sequential (T001-T037)
- ✅ [P] marker: Used for parallelizable tasks (T001-T005 setup, T019-T030 tests, T034-T037 quality)
- ✅ [Story] label: Used for User Story 1 tasks ([US1] for T006-T028, T031-T032)
- ✅ Description: Clear action with file paths
- ✅ Examples:
  - `- [ ] T006 [US1] Add Request to FastAPI imports in backend/server.py line 1`
  - `- [ ] T019 [US1] Test /personalize with "role" key - quickstart.md Test 1`

### Dependencies & Execution Order

**Phase Dependencies**:
```text
Setup (Phase 1) → Implementation (Phase 3) → Testing (Phase 4) → Quality (Phase 5)
                         ↓
            Foundational (Phase 2) - SKIPPED
```

**Parallel Opportunities**:
- Setup: 5 tasks parallel (T001-T005)
- Implementation: 13 tasks sequential (same file modifications)
- Testing: 12 tasks parallel (T019-T030)
- Quality: 4 tasks parallel (T034-T037)

**Total Parallel**: 21/37 tasks (56%) can run in parallel

### Implementation Strategy

**MVP Scope** (This Entire Feature):
- Cannot be decomposed - single-story bug fix
- All 37 tasks must complete for feature to work
- Estimated time: 35-45 minutes total

**Sequential Path**:
1. Phase 1: Setup (5 tasks) - 5 minutes
2. Phase 3: Implementation (13 tasks) - 10-15 minutes
3. Phase 4: Testing (15 tasks) - 15-20 minutes
4. Phase 5: Quality (4 tasks) - 5 minutes

**Parallel Path** (with automation):
1. Phase 1: All setup tasks together - 2 minutes
2. Phase 3: Sequential implementation - 10-15 minutes
3. Phase 4: All test scenarios together - 5-7 minutes (parallelized)
4. Phase 5: All quality checks together - 2 minutes
5. **Total with parallelization**: 20-25 minutes

### Success Criteria Mapping

All 7 success criteria from spec.md mapped to specific tasks:
- SC-001: "background" key returns 200 → T020
- SC-002: "userRole" key returns 200 → T021
- SC-003: "role" key still works → T019
- SC-004: Missing role defaults to "student" → T022
- SC-005: Console logging enabled → T031
- SC-006: Other endpoints unchanged → T029-T030
- SC-007: Response time maintained → T033

### Code Changes Summary (from tasks.md)

**3 modifications to backend/server.py**:

1. **Line 1** (T006): Add Request import
   ```python
   from fastapi import FastAPI, HTTPException, Request
   ```

2. **Lines 28-30** (T007): Comment out Pydantic model
   ```python
   # PersonalizeRequest removed for flexible JSON keys
   # class PersonalizeRequest(BaseModel):
   #     topic: str
   #     role: str
   ```

3. **Lines 62-74** (T008-T018): Rewrite endpoint
   ```python
   @app.post("/personalize")
   async def personalize_endpoint(request: Request):
       data = await request.json()
       print(f"📩 Received Personalization Request: {data}")

       topic = data.get("topic")
       if not topic:
           raise HTTPException(status_code=422, detail="Missing required field: topic")

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

### Testing Coverage

**12 test scenarios from quickstart.md** mapped to tasks:
- T019: Test original "role" key (backward compatibility)
- T020: Test "background" key (new variant)
- T021: Test "userRole" key (new variant)
- T022: Test missing role (defaults to "student")
- T023-T024: Test missing/empty topic (422 errors)
- T025: Test empty role (defaults to "student")
- T026: Test multiple keys (priority order)
- T027: Test special characters
- T028: Test malformed JSON
- T029: Test /chat endpoint (regression)
- T030: Test /translate endpoint (regression)

**Additional validations**:
- T031: Console logging verification
- T032: Extracted values logging verification
- T033: Performance measurement (~2-5 seconds maintained)

### Task Statistics
- **Total Tasks**: 37
- **Setup**: 5 tasks (13.5%)
- **Foundational**: 0 tasks (0%)
- **Implementation**: 13 tasks (35.1%)
- **Testing**: 15 tasks (40.5%)
- **Quality**: 4 tasks (10.8%)
- **Parallelizable**: 21 tasks (56.8%)
- **Sequential**: 16 tasks (43.2%)

### Deployment Strategy (from tasks.md)

**Commit Message Template**:
```bash
git commit -m "fix: Relax /personalize validation for flexible JSON keys

- Accept role/background/userRole key variants with priority order
- Default to 'student' when role missing or empty
- Add full JSON logging for debugging frontend key mismatches
- Maintain backward compatibility with existing 'role' key

Fixes 422 errors when frontend sends alternative key names.
All 12 test scenarios pass (quickstart.md Tests 1-12).

Related: 003-fix-personalize-validation"
```

**Deployment Steps**:
1. Push to branch: `git push origin 003-fix-personalize-validation`
2. Wait 2-3 minutes for Render auto-deploy
3. Test production URL with quickstart.md scenarios
4. Merge to main after validation

## Outcome

- ✅ Impact: Complete task breakdown ready for implementation. 37 tasks organized across 5 phases with clear dependencies. 12 comprehensive test scenarios mapped from quickstart.md. Implementation path clear: modify 3 sections of backend/server.py (~15 lines). Estimated completion: 35-45 minutes sequential, 20-25 minutes with parallelized testing.
- 🧪 Tests: 15 manual testing tasks (T019-T033) covering all acceptance scenarios, edge cases, and regression prevention. No automated tests (not requested in spec). Each test has curl command, expected response, and console log validation.
- 📁 Files: Created 1 file - tasks.md (520 lines with implementation patterns, testing notes, deployment strategy)
- 🔁 Next prompts: Ready for `/sp.implement` to execute tasks sequentially or use task IDs to implement individually
- 🧠 Reflection: Task generation successfully balanced granularity (13 implementation tasks for ~15 lines of code) with clarity (each task is atomic and testable). Testing phase organized by quickstart.md structure enables systematic validation. Parallel opportunities identified (21/37 tasks) optimize execution time. Single-story bug fix cannot be decomposed into MVP increments - must implement completely. Format validation passed: all 37 tasks follow strict checklist format with IDs, [P]/[US1] labels, and file paths.

## Evaluation notes (flywheel)

- Failure modes observed: None. All workflow steps executed successfully. Prerequisites check returned feature directory and available docs. Task template provided clear structure.
- Graders run and results (PASS/FAIL): Format validation - PASS (37/37 tasks follow checklist format with checkbox, ID, labels, paths). Completeness check - PASS (all 8 functional requirements mapped to tasks, all 7 success criteria mapped to validation tasks, all 12 quickstart tests mapped to test tasks). Dependency analysis - PASS (setup → implementation → testing → quality chain clear, parallel opportunities identified).
- Prompt variant (if applicable): Standard /sp.tasks workflow following outline (prerequisites → load docs → extract stories → generate tasks → validate format)
- Next experiment (smallest change to try): Consider adding checkpoint tasks after each phase (e.g., "T018a Checkpoint: Start backend server and verify no errors") to make stopping points more explicit. Current checkpoints are in phase headers but not as separate tasks. This would make incremental validation more visible in task tracking tools.
