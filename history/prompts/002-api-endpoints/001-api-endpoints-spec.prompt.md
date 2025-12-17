---
id: 001
title: api-endpoints-spec
stage: spec
date: 2025-12-17
surface: slash-command
model: claude-sonnet-4-5
feature: 002-api-endpoints
branch: 002-api-endpoints
user: User
command: /sp.specify
labels: [specification, backend, api, endpoints]
links:
  spec: ../../../specs/002-api-endpoints/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - path: specs/002-api-endpoints/spec.md
    action: created
    description: Feature specification for translation and personalization API endpoints
  - path: specs/002-api-endpoints/checklists/requirements.md
    action: created
    description: Quality validation checklist
tests: []
---

## Prompt

Update backend/server.py to fix the 404 errors for the /translate and /personalize endpoints.

Please modify server.py to include these two missing POST routes:

1. POST /translate:
   - Accept JSON body with "text".
   - Use the 'run_agent' function from agent.py.
   - Prompt the agent to: "Translate this technical text to formal Urdu, keeping English keywords."
   - Return JSON: {"translation": response}

2. POST /personalize:
   - Accept JSON body with "topic" and "role".
   - Use the 'run_agent' function from agent.py.
   - Prompt the agent to explain the topic specifically for that user role.
   - Return JSON: {"explanation": response}

Ensure imports are correct (from agent import run_agent) and keep the existing /chat route and CORS middleware intact.

## Response snapshot

Created comprehensive feature specification document following the spec-template.md structure:

1. **Branch Setup**: Created `002-api-endpoints` branch with spec directory structure
2. **User Stories**: Defined 2 prioritized user stories:
   - P1: Technical Text Translation - Core bilingual accessibility feature
   - P2: Personalized Learning Explanations - Adaptive learning enhancement
3. **Requirements**: Specified 11 functional requirements covering:
   - Two POST endpoints (/translate, /personalize)
   - Request/response data structures
   - Integration with existing run_agent function
   - Error handling and CORS maintenance
4. **Success Criteria**: Established 7 measurable outcomes focused on:
   - HTTP 200 responses with proper JSON formatting
   - Technical term preservation in translations
   - Response time parity with existing /chat endpoint
5. **Edge Cases**: Identified 6 edge cases including Urdu input, rate limits, and timeouts
6. **Quality Validation**: Created requirements.md checklist with all items passing validation

## Outcome

- ✅ Impact: Specification complete and ready for planning phase. Provides clear, testable requirements for implementing two missing API endpoints without any implementation details.
- 🧪 Tests: No code tests (specification phase). Quality validated via 14-item checklist covering content quality, requirement completeness, and feature readiness.
- 📁 Files: Created 2 files - spec.md (144 lines) and checklists/requirements.md (validation document)
- 🔁 Next prompts: Ready for `/sp.plan` to design implementation approach, then `/sp.tasks` to generate actionable task list
- 🧠 Reflection: Specification successfully focused on "what" not "how" - described endpoint behavior, data structures, and user value without prescribing FastAPI implementation details. All 11 functional requirements are testable and unambiguous.

## Evaluation notes (flywheel)

- Failure modes observed: Initial PowerShell script invocation failed due to argument parsing (positional parameters). Recovered by using existing branch created in first attempt. PHR creation script not available, successfully fell back to manual template-based creation.
- Graders run and results (PASS/FAIL): Manual validation via requirements.md checklist - PASS (14/14 items)
- Prompt variant (if applicable): Standard /sp.specify workflow
- Next experiment (smallest change to try): Consider adding request validation requirements (min/max text length) to prevent API abuse. Current spec assumes client-side or OpenAI limits will suffice.
