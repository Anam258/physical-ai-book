# Implementation Plan: Fix Personalize Endpoint Validation Error

**Branch**: `003-fix-personalize-validation` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-fix-personalize-validation/spec.md`

## Summary

Fix the /personalize endpoint in backend/server.py to accept flexible JSON key names for the user's role/background field. The current implementation uses strict Pydantic validation that returns 422 errors when the frontend sends keys like "background" or "userRole" instead of "role". The solution replaces the Pydantic model with FastAPI's Request object and manual JSON extraction with priority-based fallback logic (role → background → userRole → "student" default), enabling frontend compatibility while maintaining backward compatibility with the existing "role" key.

## Technical Context

**Language/Version**: Python 3.12+
**Primary Dependencies**: FastAPI 0.124.0+, Pydantic, python-dotenv, OpenAI SDK, uvicorn 0.38.0+
**Storage**: Qdrant Cloud (vector DB for RAG), no changes needed for this feature
**Testing**: Manual testing via curl/Postman (no automated tests requested in spec)
**Target Platform**: Linux server (Render deployment)
**Project Type**: Web application (backend Python FastAPI + frontend Docusaurus)
**Performance Goals**: Maintain existing ~2-5 seconds response time (OpenAI API call)
**Constraints**:
- Must maintain backward compatibility with existing "role" key
- Must default gracefully to "student" when no role provided
- Must log received JSON for debugging
- No new dependencies allowed
**Scale/Scope**: Single endpoint modification (~15 lines of code), 1 file changed (backend/server.py)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ PASS - Principle VI: Strict Separation of Concerns
- **Check**: Changes confined to `/backend` only?
- **Status**: ✅ PASS - Only modifying `backend/server.py` (lines 1, 28-30, 62-74)
- **Justification**: N/A - No violation

### ✅ PASS - Principle VII: Technology Stack
- **Check**: Using approved backend stack (Python 3.10+, FastAPI)?
- **Status**: ✅ PASS - Python 3.12+ with FastAPI, no new dependencies
- **Justification**: N/A - No violation

### ✅ PASS - Principle VIII: SpecKit Protocol
- **Check**: Started with `/sp.specify` command before implementation?
- **Status**: ✅ PASS - Full workflow followed: /sp.specify → /sp.plan → (next: /sp.tasks → /sp.implement)
- **Justification**: N/A - No violation

### ✅ PASS - Principle V: Continuous Delivery
- **Check**: Changes can be deployed and verified on Render?
- **Status**: ✅ PASS - Backend auto-deploys from main branch to Render, manual testing procedures defined in quickstart.md
- **Justification**: N/A - No violation

**Overall Constitution Status**: ✅ ALL GATES PASS - No violations, proceed with implementation

## Project Structure

### Documentation (this feature)

```text
specs/003-fix-personalize-validation/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output - validation approach research ✅
├── data-model.md        # Phase 1 output - flexible JSON model ✅
├── quickstart.md        # Phase 1 output - testing procedures ✅
├── contracts/           # Phase 1 output ✅
│   └── openapi.yaml     # Updated API contract with flexible keys
├── checklists/          # Specification validation
│   └── requirements.md  # Quality checklist (all items passed)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── server.py            # 🔧 MODIFY: /personalize endpoint (lines 1, 28-30, 62-74)
├── agent.py             # ✅ NO CHANGE: run_agent function unchanged
├── retriveing.py        # ✅ NO CHANGE: Qdrant retrieval unchanged
├── main.py              # ✅ NO CHANGE: Data ingestion unchanged
├── requirements.txt     # ✅ NO CHANGE: No new dependencies
└── .env                 # ✅ NO CHANGE: Existing API keys sufficient

frontend/
└── [out of scope - no frontend changes needed]
```

**Structure Decision**: Web application (Option 2) - Project has separate `backend/` (Python FastAPI) and `frontend/` (Docusaurus) directories. This feature modifies only the backend. The existing structure supports independent backend deployments without frontend changes, enabling this bug fix to roll out independently.

## Complexity Tracking

> **No violations - this section intentionally empty**

All Constitution checks passed. No complexity justification needed.

## Phase 0: Research & Design Decisions ✅ COMPLETE

**Status**: Research completed in `research.md`

### Research Tasks Completed

1. **FastAPI Request Validation Approaches**
   - **Decision**: Use `Request` object with manual JSON extraction
   - **Rationale**: Allows flexible key names, maintains backward compatibility, no new dependencies
   - **Alternatives Rejected**: Multiple Pydantic models (not maintainable), Field aliases (only one alias per field), custom validators (too complex)

2. **Error Handling for Missing Required Fields**
   - **Decision**: HTTP 422 for missing `topic`, default to "student" for missing role
   - **Rationale**: Topic is required for generating explanations, role is optional with sensible default

3. **Logging Strategy for Debugging**
   - **Decision**: Log entire received JSON object before processing
   - **Rationale**: Spec requirement FR-003, helps identify frontend key mismatches, consistent with existing logging pattern

4. **Priority Order for Key Extraction**
   - **Decision**: Check in order: `role` → `background` → `userRole` → default "student"
   - **Rationale**: Maintains backward compatibility ("role" first), covers common naming conventions

5. **Integration with Existing run_agent Function**
   - **Decision**: No changes to `run_agent` integration
   - **Rationale**: Function signature unchanged, only variable source changes (dict vs Pydantic model)

**Key Findings**:
- No new dependencies needed (FastAPI Request object built-in)
- Performance impact negligible (<0.1ms vs existing ~2-5s response time)
- Security risk low (no database, no HTML rendering, OpenAI handles malicious prompts)
- Backward compatibility guaranteed (existing "role" key continues to work)

## Phase 1: Data Model & Contracts ✅ COMPLETE

**Status**: Design artifacts generated

### Artifacts Created

1. **data-model.md** ✅
   - PersonalizeRequest (Flexible Input) entity
   - PersonalizeResponse entity
   - Validation rules for topic (required) and role (optional with fallback)
   - Edge case handling matrix
   - Before/after code comparison

2. **contracts/openapi.yaml** ✅
   - Updated /personalize endpoint contract
   - Request examples for all key variants (role, background, userRole, missing)
   - Response examples (200, 422, 500)
   - Priority order documentation in API description

3. **quickstart.md** ✅
   - 12 comprehensive test scenarios:
     - Tests 1-4: Basic functionality (all key variants)
     - Tests 5-10: Edge cases (missing fields, empty strings, malformed JSON)
     - Tests 11-12: Regression tests (/chat, /translate unchanged)
   - Production testing procedures
   - Troubleshooting guide
   - Success checklist

### Agent Context Update ✅

Ran `.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude` to update CLAUDE.md with:
- Language: Python 3.12+
- Framework: FastAPI
- Database: N/A (no database changes)
- Project type: Web application

## Phase 2: Implementation Strategy

**Note**: This section defines the implementation approach for `/sp.tasks` command to generate actionable tasks

### Implementation Approach

**Complexity**: Low - Single file modification, ~15 lines of code

**Files to Modify**:
1. `backend/server.py` - Only file requiring changes

**Changes Required**:
1. **Line 1**: Add `Request` to FastAPI imports
   ```python
   from fastapi import FastAPI, HTTPException, Request
   ```

2. **Lines 28-30**: Remove/comment out `PersonalizeRequest` Pydantic model
   ```python
   # class PersonalizeRequest(BaseModel):
   #     topic: str
   #     role: str
   ```

3. **Lines 62-74**: Rewrite `personalize_endpoint` function
   - Change signature to accept `Request` object
   - Add `data = await request.json()` to get raw JSON
   - Add logging: `print(f"📩 Received Personalization Request: {data}")`
   - Extract topic with validation
   - Extract role with priority fallback
   - Keep existing query formatting and `run_agent` call

**Estimated Implementation Time**: 10-15 minutes

**Testing Time**: 15-20 minutes (12 test scenarios in quickstart.md)

**Total Time**: 30-40 minutes from implementation to verified deployment

### Risk Assessment

**Low Risk Factors**:
- Single endpoint modification (blast radius: only /personalize affected)
- Backward compatible (existing "role" key continues to work)
- No database changes (stateless endpoint)
- No new dependencies (Request object built-in to FastAPI)
- Existing error handling preserved (try-except pattern maintained)

**Mitigation Strategy**:
- Test backward compatibility first (Test 1 in quickstart.md)
- Verify other endpoints unchanged (Tests 11-12: /chat, /translate regression tests)
- Deploy to Render staging first (test production URL before merging to main)
- Rollback plan: `git revert` commit if issues found in production

### Success Metrics

**Pre-Deployment**:
- ✅ All 12 test scenarios pass in local environment
- ✅ Console logs show full JSON data for debugging
- ✅ Response times remain ~2-5 seconds (performance maintained)

**Post-Deployment**:
- ✅ Production tests pass (replace localhost with Render URL)
- ✅ Frontend integration works with any key variant
- ✅ No 422 errors on valid requests with "background" or "userRole" keys
- ✅ Existing clients using "role" key continue to work

## Dependencies & Constraints

### External Dependencies
- **OpenAI API**: Existing - no changes (run_agent function unchanged)
- **Qdrant Cloud**: Existing - no changes (retrieval logic unchanged)
- **FastAPI**: Existing - no upgrade needed (Request object available in current version)

### Internal Dependencies
- **backend/agent.py**: No changes required (run_agent signature stable)
- **backend/retriveing.py**: No changes required (retrieval logic independent)
- **frontend/***: Out of scope (frontend can use any key variant after deployment)

### Constraints
1. **Backward Compatibility**: MUST support existing "role" key (addressed by priority order)
2. **No New Dependencies**: MUST use existing FastAPI/Python features (addressed by using built-in Request object)
3. **Performance Parity**: MUST maintain ~2-5 second response time (addressed by negligible <0.1ms overhead)
4. **Logging Requirement**: MUST log full JSON for debugging (addressed by print statement before processing)

## Next Steps for `/sp.tasks`

The `/sp.tasks` command will generate a detailed task list based on this plan. Expected task structure:

**Phase 1: Setup & Verification** (2-3 tasks)
- Verify Python 3.12+ and dependencies installed
- Verify backend/server.py current state (lines 1, 28-30, 62-74)
- Verify run_agent function signature unchanged

**Phase 2: Implementation** (4-5 tasks)
- Add `Request` to imports (line 1)
- Comment out PersonalizeRequest Pydantic model (lines 28-30)
- Rewrite personalize_endpoint signature
- Add JSON extraction and logging
- Add topic validation
- Add role extraction with priority fallback
- Keep existing run_agent call and error handling

**Phase 3: Testing** (12 tasks)
- Execute all 12 test scenarios from quickstart.md
- Verify console logs show full JSON data
- Measure response times (<5 seconds)

**Phase 4: Deployment** (3-4 tasks)
- Commit changes with descriptive message
- Push to 003-fix-personalize-validation branch
- Deploy to Render (auto-deploy)
- Run production tests with Render URL

**Total Estimated Tasks**: 20-25 tasks

## Revision History

| Date | Change | Reason |
|------|--------|--------|
| 2025-12-17 | Initial plan created | /sp.plan command execution |
| 2025-12-17 | Research completed (Phase 0) | Resolved all NEEDS CLARIFICATION items |
| 2025-12-17 | Data model & contracts created (Phase 1) | Generated design artifacts |
| 2025-12-17 | Agent context updated | Ran update-agent-context.ps1 for CLAUDE.md |
| 2025-12-17 | Implementation strategy defined (Phase 2) | Ready for /sp.tasks command |

---

**Plan Status**: ✅ COMPLETE - Ready for `/sp.tasks` command to generate actionable task list
