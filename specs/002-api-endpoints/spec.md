# Feature Specification: Backend API Endpoints - Translation and Personalization

**Feature Branch**: `002-api-endpoints`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "Update backend/server.py to fix the 404 errors for the /translate and /personalize endpoints. Please modify server.py to include these two missing POST routes."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Technical Text Translation (Priority: P1)

A student reading technical documentation in the Physical AI handbook encounters complex terms and wants to see them translated to formal Urdu while preserving English technical keywords for clarity.

**Why this priority**: This is the most critical feature as it directly addresses the bilingual learning need identified in the system (per Skill 11 in CLAUDE.md). Students who prefer Urdu can understand concepts better while maintaining familiarity with English technical terminology.

**Independent Test**: Can be fully tested by sending a POST request to `/translate` with technical English text and verifying the response contains formal Urdu translation with preserved English technical terms. Delivers immediate value for Urdu-speaking students.

**Acceptance Scenarios**:

1. **Given** a student has selected technical text containing terms like "ROS 2", "Node", and "Python", **When** they request translation via the translate endpoint, **Then** the system returns formal Urdu text with "ROS 2", "Node", and "Python" remaining in English.

2. **Given** a student submits text with code blocks and explanatory content, **When** the translation is processed, **Then** code blocks remain entirely in English while explanatory text is translated to formal Urdu.

3. **Given** a student submits empty text or malformed request, **When** the endpoint processes it, **Then** the system returns an appropriate error response without crashing.

---

### User Story 2 - Personalized Learning Explanations (Priority: P2)

A student with a specific background (e.g., "Arduino hobbyist" or "Python developer") encounters a new robotics topic and wants an explanation tailored to their existing knowledge and experience.

**Why this priority**: This feature enables adaptive learning by contextualizing new information based on the learner's background. While important, it's secondary to basic translation needs since it's an enhancement to learning rather than core accessibility.

**Independent Test**: Can be fully tested by sending a POST request to `/personalize` with a topic and user role/background, then verifying the response provides a contextualized explanation. Delivers value by improving learning efficiency.

**Acceptance Scenarios**:

1. **Given** a student with "Arduino hobbyist" background encounters the topic "ROS 2 Publishers", **When** they request personalization, **Then** the system provides a 2-sentence analogy comparing ROS 2 publishers to Arduino serial output patterns.

2. **Given** a student with "Python developer" background wants to understand "URDF files", **When** they request personalization, **Then** the system explains URDF in terms familiar to Python developers (e.g., comparing XML structure to Python dictionaries or config files).

3. **Given** a student submits a topic without specifying their role/background, **When** the endpoint processes it, **Then** the system returns an error or requests the missing information.

---

### Edge Cases

- What happens when the translation endpoint receives text that is already in Urdu?
- How does the system handle mixed language input (partial English, partial Urdu)?
- What happens when the personalize endpoint receives an unknown or unconventional user role (e.g., "quantum physicist")?
- How does the system respond when API rate limits are hit on the OpenAI backend?
- What happens if the agent.run_agent() function times out or fails?
- How are very long text inputs (>2000 words) handled in translation requests?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a POST endpoint at `/translate` that accepts JSON with a "text" field containing the content to be translated.

- **FR-002**: System MUST use the existing `run_agent` function from `agent.py` to process translation requests, instructing the agent to "Translate this technical text to formal Urdu, keeping English keywords."

- **FR-003**: System MUST return translation results as JSON in the format `{"translation": response}` where response contains the formal Urdu text with preserved English technical terms.

- **FR-004**: System MUST provide a POST endpoint at `/personalize` that accepts JSON with "topic" and "role" fields.

- **FR-005**: System MUST use the existing `run_agent` function from `agent.py` to process personalization requests, instructing the agent to explain the topic specifically for the provided user role.

- **FR-006**: System MUST return personalization results as JSON in the format `{"explanation": response}` where response contains a contextualized 2-sentence explanation.

- **FR-007**: System MUST preserve existing `/chat` endpoint functionality without modification or interference.

- **FR-008**: System MUST maintain existing CORS middleware configuration (allow_origins=["*"]) for all endpoints.

- **FR-009**: System MUST import `run_agent` from `agent.py` at the module level (this import already exists per current codebase review).

- **FR-010**: System MUST handle errors gracefully by catching exceptions from `run_agent` and returning appropriate HTTP error responses.

- **FR-011**: System MUST await the `run_agent` coroutine since it is an async function (as confirmed by the `async def` in agent.py line 65).

### Key Entities

- **TranslateRequest**: Represents incoming translation requests
  - Attributes: text (string, required) - the content to be translated
  - Validation: text must be non-empty string

- **PersonalizeRequest**: Represents incoming personalization requests
  - Attributes:
    - topic (string, required) - the subject matter to be explained
    - role (string, required) - the user's background/expertise level
  - Validation: both fields must be non-empty strings

- **TranslateResponse**: Represents translation results
  - Attributes: translation (string) - the Urdu translation with preserved English terms

- **PersonalizeResponse**: Represents personalization results
  - Attributes: explanation (string) - the contextualized explanation

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Frontend requests to `/translate` endpoint receive successful responses (HTTP 200) with properly formatted JSON containing Urdu translations.

- **SC-002**: Frontend requests to `/personalize` endpoint receive successful responses (HTTP 200) with properly formatted JSON containing personalized explanations.

- **SC-003**: Translation results preserve all technical English keywords (ROS 2, Python, Node, API, etc.) while surrounding text is in formal Urdu.

- **SC-004**: Personalization explanations are limited to 2-3 sentences as specified in the system requirements.

- **SC-005**: Existing `/chat` endpoint continues to function without any degradation or behavioral changes.

- **SC-006**: Server handles invalid requests (missing fields, empty values) by returning appropriate error responses without crashing.

- **SC-007**: Response time for both endpoints matches the existing `/chat` endpoint performance characteristics (no significant degradation).

## Assumptions

- The OpenAI GPT-4o model has sufficient context about technical terms to preserve them correctly during translation.
- The existing `run_agent` function can handle different system prompts flexibly without modification.
- The user roles provided to the personalize endpoint will be common technology backgrounds (e.g., "Arduino hobbyist", "Python developer", "ROS beginner") rather than obscure specializations.
- The frontend components already have UI for triggering these endpoints and are only waiting for the backend implementation.
- Rate limiting and quota management for OpenAI API calls are handled at the infrastructure level, not within these endpoints.
- Input validation for reasonable text length limits will be handled by the client-side or by OpenAI's own token limits.

## Dependencies

- Existing `agent.py` module with `run_agent` async function
- OpenAI API access configured via environment variables
- FastAPI framework and Pydantic for request models
- Existing CORS middleware configuration

## Out of Scope

- Modifying the `run_agent` function's internal logic or system prompt
- Implementing caching for repeated translations or personalizations
- Adding authentication or authorization to the endpoints
- Creating new Pydantic base models beyond the simple request/response structures needed
- Implementing streaming responses (endpoints will return complete responses)
- Adding logging infrastructure beyond the existing print statements pattern
- Creating comprehensive input sanitization beyond basic Pydantic validation
- Building a translation memory or glossary system
- Supporting languages other than English and Urdu
