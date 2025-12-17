# Implementation Plan: Backend API Endpoints - Translation and Personalization

**Branch**: `002-api-endpoints` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-api-endpoints/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement two missing POST endpoints in backend/server.py to fix 404 errors: `/translate` for converting technical English text to formal Urdu while preserving English keywords, and `/personalize` for providing context-aware explanations based on user background. Both endpoints leverage the existing `run_agent` async function from agent.py with custom prompts, return JSON responses, and maintain compatibility with existing /chat endpoint and CORS configuration.

## Technical Context

**Language/Version**: Python 3.12+ (requires-python >=3.12 per pyproject.toml)
**Primary Dependencies**: FastAPI 0.124.0+, OpenAI 2.9.0+, Pydantic (already installed), uvicorn 0.38.0+
**Storage**: N/A (endpoints use existing agent.py which queries Qdrant via retriveing.py)
**Testing**: pytest (standard for Python projects, not yet configured in backend/)
**Target Platform**: Linux server (Render deployment at https://physical-ai-backend-j8q9.onrender.com)
**Project Type**: Web application (backend API only, frontend is separate Docusaurus app)
**Performance Goals**: Match existing /chat endpoint performance (~2-5s response time for OpenAI GPT-4o completion)
**Constraints**: Must maintain response time parity with /chat endpoint; preserve existing CORS config (allow_origins=["*"])
**Scale/Scope**: Educational platform supporting ~100-1000 concurrent students; 2 new endpoints, 2 new Pydantic models, ~40-60 lines of code

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle VI: Strict Separation of Concerns ✅ PASS
- **Requirement**: `/backend` reserved for Python FastAPI; no UI code allowed
- **Assessment**: PASS - This feature modifies only backend/server.py to add two POST endpoints. No frontend code changes in this feature scope.
- **Evidence**: FR-001 through FR-011 specify backend endpoints only; frontend integration is out of scope per spec.md

### Principle VII: Technology Stack ✅ PASS
- **Requirement**: Backend must use Python 3.10+, FastAPI, OpenAI Agent SDK
- **Assessment**: PASS - Uses Python 3.12+, FastAPI 0.124.0+, OpenAI SDK 2.9.0+ (already installed)
- **Evidence**: pyproject.toml requires-python >=3.12; dependencies already include fastapi and openai

### Principle VIII: SpecKit Protocol (History Mandate) ✅ PASS
- **Requirement**: Plan before code; verification with build passes
- **Assessment**: PASS - Following /sp.specify → /sp.plan → /sp.tasks workflow; backend has no build step (Python runtime)
- **Evidence**: spec.md created first, now executing /sp.plan command

### Principle I: User-Centric Design ✅ PASS
- **Requirement**: Prioritize intuitive UX and accessibility
- **Assessment**: PASS - Translation endpoint (P1) directly supports Urdu learners per Skill 11; personalization (P2) enhances learning
- **Evidence**: User Story 1 addresses bilingual accessibility; User Story 2 enables adaptive learning

### Principle V: Continuous Delivery ✅ PASS
- **Requirement**: No feature is "done" until deployed and verified
- **Assessment**: PASS - Deployment target is Render (https://physical-ai-backend-j8q9.onrender.com); will require redeploy after merge
- **Evidence**: Backend already deployed on Render; git push to main triggers auto-deploy

**GATE RESULT: ✅ ALL CHECKS PASS - Proceed to Phase 0 Research**

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── server.py           # ✏️ MODIFY: Add /translate and /personalize endpoints
├── agent.py            # ✅ USE AS-IS: Existing run_agent() function
├── retriveing.py       # ✅ USE AS-IS: Qdrant retrieval (used by agent.py)
├── main.py             # ✅ NO CHANGE: Data ingestion script
├── requirements.txt    # ✅ NO CHANGE: All dependencies already present
├── pyproject.toml      # ✅ NO CHANGE: Project metadata
└── __pycache__/        # Generated files

frontend/
└── [OUT OF SCOPE: Frontend integration handled separately]

specs/002-api-endpoints/
├── spec.md             # ✅ COMPLETE: Feature specification
├── plan.md             # 📝 IN PROGRESS: This file
├── research.md         # ⏳ PHASE 0: Research findings
├── data-model.md       # ⏳ PHASE 1: Request/response models
├── quickstart.md       # ⏳ PHASE 1: Testing guide
└── contracts/          # ⏳ PHASE 1: API contracts (OpenAPI)
```

**Structure Decision**: This is a web application (Option 2) with strict backend/frontend separation per Constitution Principle VI. This feature modifies only the backend layer, specifically backend/server.py to add two new FastAPI route handlers. No new files are created; all functionality is added to the existing server.py module following the established pattern of the /chat endpoint.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**N/A** - No constitution violations. All checks passed.
