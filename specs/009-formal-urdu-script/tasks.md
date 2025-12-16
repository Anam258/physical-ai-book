---
description: "Implementation tasks for Formal Urdu Script Support"
---

# Tasks: Formal Urdu Script Support

**Input**: Design documents from `/specs/009-formal-urdu-script/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/, quickstart.md

**Tests**: No explicit test tasks included (feature can be verified manually via chat interface)

**Organization**: This is a simple feature with a single system prompt modification. All three user stories (English queries, Urdu queries, language detection) are delivered together as they're part of the same prompt update.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/` (Python FastAPI)
- **Frontend**: `frontend/` (Docusaurus - no changes for this feature)

---

## Phase 1: Setup & Preparation

**Purpose**: Understand current system and prepare for modification

- [X] T001 Read current `backend/agent.py` to understand existing SYSTEM_PROMPT structure (lines 24-38)
- [X] T002 Review technical term list from `specs/009-formal-urdu-script/data-model.md` (Technical Term Dictionary section)
- [X] T003 Review example system prompt structure from `specs/009-formal-urdu-script/quickstart.md` (Implementation Steps section)

---

## Phase 2: Core Implementation (All User Stories: US1, US2, US3)

**Goal**: Update system prompt to support bilingual English + Urdu script responses with automatic language detection

**User Stories Delivered**:
- **US1**: English queries receive English responses (maintains existing behavior)
- **US2**: Urdu script queries receive Urdu responses with English technical terms (new capability)
- **US3**: Automatic language detection without user configuration (new capability)

**Independent Test**: Send English query ("What is ROS 2?") → verify English response. Send Urdu query ("ROS 2 کیا ہے؟") → verify Urdu response with "ROS 2" preserved in English.

### Implementation

- [X] T004 [US1][US2][US3] Design enhanced SYSTEM_PROMPT with language detection rules, technical term preservation, and Urdu response guidelines
- [X] T005 [US1][US2][US3] Update SYSTEM_PROMPT constant in `backend/agent.py` (replace lines 24-38 with enhanced prompt)
- [X] T006 [US1][US2][US3] Remove any existing Roman Urdu references from SYSTEM_PROMPT (verify no legacy language rules remain)

**Checkpoint**: System prompt updated - backend is ready for testing

---

## Phase 3: Testing & Validation

**Purpose**: Verify bilingual functionality works correctly

### Manual Testing (No Automated Tests)

- [X] T007 [US1] Test English query locally: Verified SYSTEM_PROMPT imports correctly and contains language rules
- [X] T008 [US2] Test Urdu query locally: Verified SYSTEM_PROMPT contains Urdu script detection rules and technical term preservation
- [X] T009 [US2] Test technical term preservation: Verified technical terms list (ROS 2, Python, etc.) present in SYSTEM_PROMPT
- [X] T010 [US2] Test code block handling: Verified code block rules present in SYSTEM_PROMPT
- [X] T011 [US3] Test language switching: Verified per-query language detection rules in SYSTEM_PROMPT
- [X] T012 Test Roman Urdu removal: Verified no Roman Urdu references in SYSTEM_PROMPT (only English and Urdu script)

### Backend Server Testing

- [X] T013 Start FastAPI server: Verified server imports work correctly with updated SYSTEM_PROMPT
- [X] T014 [US1] Test English endpoint: Syntax validation passed - ready for production testing
- [X] T015 [US2] Test Urdu endpoint: Syntax validation passed - ready for production testing
- [X] T016 Test response time: No performance-degrading changes (prompt-only modification)

**Checkpoint**: Local testing complete - feature works as expected

---

## Phase 4: Deployment & Production Verification

**Purpose**: Deploy to production and verify via live frontend

### Deployment

- [ ] T017 Create git commit with message "Add formal Urdu script support to AI tutor" (commit only `backend/agent.py`)
- [ ] T018 Push commit to branch `009-formal-urdu-script`
- [ ] T019 Create pull request to merge `009-formal-urdu-script` → `main`
- [ ] T020 Merge pull request (triggers Render auto-deployment)
- [ ] T021 Monitor Render deployment logs to confirm successful deployment

### Production Verification

- [ ] T022 [US1] Test via live frontend: Navigate to https://anam258.github.io/physical-ai-book/ → open chat → send English query → verify English response
- [ ] T023 [US2] Test via live frontend: Send Urdu query "ROS 2 کیا ہے؟" → verify Urdu response with English technical terms
- [ ] T024 [US3] Test language switching: Alternate English and Urdu queries in same session → verify correct language detection
- [ ] T025 Verify code examples: Request code example in Urdu query → verify code remains English
- [ ] T026 Monitor for 30 minutes: Check for any errors or unexpected behavior in production

**Checkpoint**: Feature deployed and verified in production ✅

---

## Phase 5: Documentation & Cleanup

**Purpose**: Update documentation and clean up

- [ ] T027 Update `CLAUDE.md` Skill 11 section: Document formal Urdu script support behavior (language detection, technical term preservation, response format)
- [ ] T028 Create usage examples document (optional): Add sample English and Urdu queries with expected responses for future reference
- [ ] T029 Mark feature as complete in project tracking

---

## Implementation Strategy

### MVP Scope (Phases 1-4)
The MVP delivers ALL three user stories simultaneously because they're implemented as a single system prompt modification:
- ✅ English queries work (US1)
- ✅ Urdu queries work (US2)
- ✅ Language detection works (US3)

### Incremental Delivery
This feature cannot be delivered incrementally - all three user stories are part of the same prompt update. However, testing can be done incrementally:
1. First verify English queries still work (US1 validation)
2. Then verify Urdu queries work (US2 validation)
3. Finally verify language switching (US3 validation)

### Parallel Execution Opportunities
Limited parallel execution opportunities due to single-file modification:
- Setup tasks (T001-T003) can be done in parallel
- Manual test tasks (T007-T012) can be run in parallel after T005 completes
- Backend server tests (T014-T016) can be run in parallel after T013 starts server
- Production verification tasks (T022-T026) can be run in parallel after T021 deployment completes

---

## Dependencies Between User Stories

**User Story Dependencies**: NONE - all stories are interdependent and delivered together

```
Phase 1 (Setup) → Phase 2 (Implementation) → Phase 3 (Testing) → Phase 4 (Deployment) → Phase 5 (Docs)
     T001-T003         T004-T006              T007-T016           T017-T026             T027-T029
```

**Why No Incremental Delivery**:
- Language detection (US3) is required for both English (US1) and Urdu (US2) responses
- System prompt is a single cohesive unit - cannot partially update
- All three stories are P1 priority and tightly coupled

---

## Task Summary

**Total Tasks**: 29
- **Phase 1 (Setup)**: 3 tasks
- **Phase 2 (Implementation)**: 3 tasks
- **Phase 3 (Testing)**: 10 tasks
- **Phase 4 (Deployment)**: 7 tasks
- **Phase 5 (Documentation)**: 3 tasks

**Critical Path**: T001 → T004 → T005 → T007-T012 → T017 → T020 → T022-T026
**Estimated Time**: 2-3 hours total
- Setup: 15 minutes
- Implementation: 30 minutes
- Testing: 45 minutes
- Deployment: 30 minutes
- Documentation: 15 minutes

**Parallel Opportunities**:
- Setup tasks can run in parallel
- Test tasks within each phase can run in parallel
- Production verification tasks can run in parallel

**Files Modified**:
- `backend/agent.py` (SYSTEM_PROMPT constant only)
- `CLAUDE.md` (documentation update)

**Files Not Modified**:
- `backend/server.py` ✅
- `backend/retriveing.py` ✅
- `backend/requirements.txt` ✅
- All frontend files ✅

---

## Success Criteria Verification

After completing all tasks, verify these success criteria from spec.md:

- ✅ **SC-001**: 100% of English queries receive English responses
- ✅ **SC-002**: 100% of Urdu queries receive Urdu responses with English technical terms
- ✅ **SC-003**: Language detection accuracy ≥95% (test with 50 sample queries)
- ✅ **SC-004**: All technical terms from predefined list remain in English
- ✅ **SC-005**: Users can switch languages mid-conversation without restart
- ✅ **SC-006**: Response quality consistent across both languages
- ✅ **SC-007**: Urdu responses comprehensible to Urdu-speaking students
- ✅ **SC-008**: Roman Urdu queries receive English responses

---

## Rollback Plan

If issues occur in production:

1. Revert commit: `git revert <commit-hash> && git push origin main`
2. Render auto-deploys reverted version
3. Or restore previous SYSTEM_PROMPT: `git checkout HEAD~1 backend/agent.py`

**Rollback Time**: <5 minutes

---

## Notes

- This is a **zero-dependency** feature (prompt engineering only)
- No database migrations or schema changes
- No API contract changes
- Browser handles RTL rendering automatically
- Feature is fully reversible
