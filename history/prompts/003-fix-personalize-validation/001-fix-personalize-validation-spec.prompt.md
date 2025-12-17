---
id: 001
title: fix-personalize-validation-spec
stage: spec
date: 2025-12-17
surface: slash-command
model: claude-sonnet-4-5
feature: 003-fix-personalize-validation
branch: 003-fix-personalize-validation
user: User
command: /sp.specify
labels: [specification, backend, api, validation, bug-fix]
links:
  spec: ../../../specs/003-fix-personalize-validation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - path: specs/003-fix-personalize-validation/spec.md
    action: created
    description: Feature specification for fixing personalize endpoint validation error
  - path: specs/003-fix-personalize-validation/checklists/requirements.md
    action: created
    description: Quality validation checklist
tests: []
---

## Prompt

Fix the "422 Unprocessable Entity" error on the /personalize endpoint in backend/server.py.

The issue is likely a mismatch between the Frontend's JSON keys and the Backend's Pydantic model validation. The frontend might be sending keys like "background" or "userRole", while the backend expects "role".

Please update backend/server.py to:
1. Relax the validation for the /personalize endpoint.
2. Instead of a strict Pydantic model, use `request: Request` to accept raw JSON.
3. Manually extract the role using: `role = data.get("role") or data.get("background") or data.get("userRole") or "student"`.
4. Ensure the endpoint prints the received data to the console (for debugging) before processing.
5. Keep the existing logic for calling 'run_agent'.

This will ensure the endpoint works regardless of the variable name sent by the frontend.

## Response snapshot

Created comprehensive feature specification document following the spec-template.md structure to fix critical validation bug blocking frontend integration:

1. **Branch Setup**: Created `003-fix-personalize-validation` branch with spec directory structure

2. **User Story**: Defined single P1 critical user story:
   - Frontend Compatibility with Flexible JSON Keys - Endpoint currently broken due to strict Pydantic validation
   - 4 acceptance scenarios covering all key variants (role, background, userRole, missing key)

3. **Requirements**: Specified 8 functional requirements covering:
   - Accept flexible JSON key names (role, background, userRole)
   - Priority-based extraction with "student" default fallback
   - Console logging for debugging
   - Preserve existing run_agent integration
   - Maintain response format and error handling patterns
   - No regression on /translate and /chat endpoints

4. **Success Criteria**: Established 7 measurable outcomes focused on:
   - HTTP 200 responses (not 422) for all key variants
   - Backward compatibility with existing "role" key
   - Console logging visibility
   - Response time parity (~2-5 seconds)
   - No endpoint regressions

5. **Edge Cases**: Identified 5 edge cases including:
   - Empty role strings
   - Missing topic field
   - Multiple role keys present
   - Malformed JSON
   - Special characters in role values

6. **Quality Validation**: Created requirements.md checklist with all 14 items passing validation

## Outcome

- ✅ Impact: Specification complete and ready for planning phase. Addresses critical bug blocking personalization feature from working with frontend clients that don't match exact Pydantic schema. Solution enables flexible key name acceptance while defaulting gracefully.
- 🧪 Tests: No code tests (specification phase). Quality validated via 14-item checklist covering content quality, requirement completeness, and feature readiness.
- 📁 Files: Created 2 files - spec.md (110 lines) and checklists/requirements.md (57 lines with validation notes)
- 🔁 Next prompts: Ready for `/sp.plan` to design implementation approach (Request object vs Pydantic model relaxation), then `/sp.tasks` to generate actionable task list
- 🧠 Reflection: Specification successfully avoided implementation details by describing WHAT needs to happen (accept flexible keys, extract with priority, default to "student") without prescribing HOW (no mention of Request object mechanics or JSON parsing specifics). All 8 functional requirements are testable via HTTP requests with different key names.

## Evaluation notes (flywheel)

- Failure modes observed: PowerShell script for creating feature branch succeeded for branch creation but failed on description parameter binding. Recovered by using successfully created branch and manually creating spec directory structure. No PHR creation script available - created PHR manually following existing 002-api-endpoints pattern.
- Graders run and results (PASS/FAIL): Manual validation via requirements.md checklist - PASS (14/14 items). All sections complete, no [NEEDS CLARIFICATION] markers, requirements testable, success criteria measurable and technology-agnostic.
- Prompt variant (if applicable): Standard /sp.specify workflow with explicit implementation requirements in user prompt (Request object, manual extraction logic)
- Next experiment (smallest change to try): Consider adding validation requirement for handling conflicting role keys (e.g., both "role" and "background" present with different values). Current spec assumes priority order handles this implicitly, but explicit handling might improve debugging experience.
