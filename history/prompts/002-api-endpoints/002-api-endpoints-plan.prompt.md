---
id: 002
title: api-endpoints-plan
stage: plan
date: 2025-12-17
surface: slash-command
model: claude-sonnet-4-5
feature: 002-api-endpoints
branch: 002-api-endpoints
user: User
command: /sp.plan
labels: [planning, design, backend, api, fastapi]
links:
  spec: ../../../specs/002-api-endpoints/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - path: specs/002-api-endpoints/plan.md
    action: created
    description: Implementation plan with technical context and constitution check
  - path: specs/002-api-endpoints/research.md
    action: created
    description: Phase 0 research findings resolving all technical questions
  - path: specs/002-api-endpoints/data-model.md
    action: created
    description: Phase 1 Pydantic request/response model definitions
  - path: specs/002-api-endpoints/contracts/openapi.yaml
    action: created
    description: Phase 1 OpenAPI 3.0 specification for both endpoints
  - path: specs/002-api-endpoints/quickstart.md
    action: created
    description: Phase 1 manual testing guide with curl examples
  - path: CLAUDE.md
    action: modified
    description: Updated agent context with Python 3.12+, FastAPI, OpenAI SDK tech stack
tests: []
---

## Prompt

(Invoked via `/sp.plan` slash command with no additional arguments)

Execute the implementation planning workflow for the 002-api-endpoints feature following the plan template structure:
1. Setup: Run setup-plan.ps1 and parse configuration
2. Load context: Read spec.md and constitution.md
3. Fill technical context and evaluate constitution gates
4. Phase 0: Research unknowns and best practices
5. Phase 1: Create data models, API contracts, and quickstart guide
6. Phase 1: Update agent context files
7. Stop and report

## Response snapshot

Successfully executed complete planning workflow:

### Setup & Context (Steps 1-2)
- Ran setup-plan.ps1, received config: BRANCH=002-api-endpoints, IMPL_PLAN=specs/002-api-endpoints/plan.md
- Loaded spec.md (11 functional requirements, 2 user stories) and constitution.md (5 core principles)
- Examined existing codebase: backend/server.py (44 lines), agent.py (111 lines), pyproject.toml

### Technical Context (Step 3)
Filled plan.md with concrete technical details:
- **Language**: Python 3.12+ (per pyproject.toml requires-python >=3.12)
- **Dependencies**: FastAPI 0.124.0+, OpenAI 2.9.0+, Pydantic, uvicorn 0.38.0+ (all already installed)
- **Storage**: N/A (uses existing agent.py → retriveing.py → Qdrant flow)
- **Testing**: pytest (standard, not yet configured)
- **Platform**: Linux server (Render deployment at https://physical-ai-backend-j8q9.onrender.com)
- **Performance**: Match existing /chat endpoint (~2-5s response time)
- **Scale**: 2 endpoints, 2 Pydantic models, ~40-60 lines of code

### Constitution Check (Step 3)
Evaluated all 5 applicable principles:
- ✅ Principle VI (Separation of Concerns): Backend-only changes, no frontend code
- ✅ Principle VII (Technology Stack): Python 3.12+, FastAPI, OpenAI SDK
- ✅ Principle VIII (SpecKit Protocol): Following /sp.specify → /sp.plan → /sp.tasks workflow
- ✅ Principle I (User-Centric Design): Translation (P1) for Urdu accessibility, Personalization (P2) for adaptive learning
- ✅ Principle V (Continuous Delivery): Render auto-deploy on main branch merge
- **GATE RESULT: ALL CHECKS PASS**

### Phase 0: Research (Step 4)
Created research.md documenting 5 key decisions:
1. **run_agent Integration**: Pass custom prompt strings without modifying agent.py signature
2. **Pydantic Pattern**: Follow existing ChatRequest model pattern with simple BaseModel classes
3. **Error Handling**: Copy existing try-except HTTPException pattern from /chat endpoint
4. **Translation Prompt**: Leverage existing SYSTEM_PROMPT language rules (lines 30-50 in agent.py)
5. **Response Time**: Expect 2-5s (matching /chat); no special monitoring for MVP

All technical clarifications resolved through codebase examination. No unresolved questions.

### Phase 1: Design Artifacts (Step 5)
Created 4 design documents:

**data-model.md** (defined 4 entities):
- TranslateRequest: `{text: str}` with Pydantic validation
- TranslateResponse: `{translation: str}` as dict literal
- PersonalizeRequest: `{topic: str, role: str}` with Pydantic validation
- PersonalizeResponse: `{explanation: str}` as dict literal
- Documented validation rules, example requests/responses, error scenarios

**contracts/openapi.yaml** (162 lines):
- Full OpenAPI 3.0 specification for both endpoints
- Request/response schemas with examples
- Error response schemas (422 validation, 500 internal)
- Production + local server URLs
- Tags: Translation, Personalization

**quickstart.md** (comprehensive testing guide):
- Prerequisites for local + production testing
- curl examples for /translate and /personalize
- Python requests examples
- Postman instructions
- 9 edge case tests (missing fields, empty values, unconventional inputs)
- Troubleshooting guide
- Success criteria validation checklist

### Phase 1: Agent Context Update (Step 6)
Ran update-agent-context.ps1:
- Updated CLAUDE.md with Python 3.12+, FastAPI 0.124.0+, OpenAI 2.9.0+
- Preserved existing Skills 1-11 and project structure documentation
- Added technology context for future feature development

## Outcome

- ✅ Impact: Implementation plan complete with zero unresolved clarifications. All design artifacts ready for /sp.tasks command to generate actionable task breakdown.
- 🧪 Tests: No code tests (planning phase). Manual test procedures documented in quickstart.md (9 edge cases, 7 success criteria).
- 📁 Files: Created 5 files (plan.md, research.md, data-model.md, openapi.yaml, quickstart.md), modified 1 file (CLAUDE.md)
- 🔁 Next prompts: Ready for `/sp.tasks` to break down implementation into atomic tasks, then `/sp.implement` for execution
- 🧠 Reflection: Research phase identified that all required patterns already exist in codebase (Pydantic models, async endpoints, error handling). Implementation is straightforward replication of /chat endpoint pattern with custom prompts. No new dependencies or architectural changes needed.

## Evaluation notes (flywheel)

- Failure modes observed: None. All scripts executed successfully. Agent context update script correctly parsed plan.md and updated CLAUDE.md without breaking existing content.
- Graders run and results (PASS/FAIL): Constitution Check - PASS (5/5 principles). Research completeness - PASS (5/5 questions resolved). Design artifacts - PASS (all mandatory templates filled).
- Prompt variant (if applicable): Standard /sp.plan workflow per .specify/templates/commands/plan.md
- Next experiment (smallest change to try): Consider adding rate limiting or request queuing if OpenAI API 429 errors become frequent in production. Current spec assumes rate limits are handled at infrastructure level (out of scope per spec.md line 123).
