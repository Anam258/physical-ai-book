# Feature Specification: Fix Personalize Endpoint Validation Error

**Feature Branch**: `003-fix-personalize-validation`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "Fix the 422 Unprocessable Entity error on the /personalize endpoint in backend/server.py by relaxing validation to accept multiple JSON key variants (role, background, userRole)"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Frontend Compatibility with Flexible JSON Keys (Priority: P1)

A frontend developer integrates with the /personalize endpoint but uses "background" or "userRole" as the JSON key instead of "role". Currently, the backend returns a 422 Unprocessable Entity error due to strict Pydantic validation, blocking the personalization feature from working.

**Why this priority**: This is critical because the endpoint is currently broken for frontend clients that don't match the exact Pydantic model schema. This blocks the personalization feature entirely, preventing students from receiving tailored explanations based on their background.

**Independent Test**: Can be fully tested by sending POST requests to /personalize with different JSON key names ("role", "background", "userRole") and verifying all variants return HTTP 200 with personalized explanations instead of 422 errors.

**Acceptance Scenarios**:

1. **Given** a frontend sends `{"topic": "ROS 2", "background": "Python developer"}`, **When** the /personalize endpoint processes the request, **Then** the system extracts "Python developer" from the "background" key and returns a personalized explanation without a 422 error.

2. **Given** a frontend sends `{"topic": "URDF files", "userRole": "Arduino hobbyist"}`, **When** the /personalize endpoint processes the request, **Then** the system extracts "Arduino hobbyist" from the "userRole" key and returns a personalized explanation.

3. **Given** a frontend sends `{"topic": "LIDAR sensors", "role": "JavaScript developer"}`, **When** the /personalize endpoint processes the request, **Then** the system extracts "JavaScript developer" from the "role" key (original key name) and returns a personalized explanation.

4. **Given** a frontend sends `{"topic": "Servos"}` without any role/background key, **When** the /personalize endpoint processes the request, **Then** the system defaults to "student" as the role and returns a general explanation without errors.

---

### Edge Cases

- What happens when the frontend sends an empty string for role/background/userRole?
- How does the system handle requests where the topic is missing but role is present?
- What happens if multiple role keys are present (e.g., both "role" and "background" with different values)?
- How does the system respond if the JSON body is malformed or empty?
- What happens when the role value contains special characters or very long text?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST accept POST requests to /personalize endpoint with flexible JSON key names for the user's background/role field, including "role", "background", and "userRole".

- **FR-002**: System MUST attempt to extract the role value by checking JSON keys in the following priority order: "role", "background", "userRole", defaulting to "student" if none are present.

- **FR-003**: System MUST log the received JSON data to the console for debugging purposes before processing the request.

- **FR-004**: System MUST continue using the existing run_agent function to generate personalized explanations based on the extracted topic and role values.

- **FR-005**: System MUST return personalized explanations in the format `{"explanation": response}` matching the existing endpoint behavior.

- **FR-006**: System MUST handle missing or empty role values gracefully by defaulting to "student" instead of returning a 422 validation error.

- **FR-007**: System MUST preserve the existing error handling pattern (try-except with HTTPException for 500 errors) for agent processing failures.

- **FR-008**: System MUST maintain the existing /translate and /chat endpoints without modification or regression.

### Key Entities

- **PersonalizeRequest (flexible)**: Represents incoming personalization requests with flexible field names
  - topic (required): The subject to be explained
  - role/background/userRole (optional): The user's expertise level or background
  - Fallback role: "student" if no role field provided

- **PersonalizeResponse**: Represents personalization results
  - explanation: The contextualized 2-3 sentence explanation

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Frontend requests to /personalize with "background" key return HTTP 200 instead of 422.

- **SC-002**: Frontend requests to /personalize with "userRole" key return HTTP 200 instead of 422.

- **SC-003**: Frontend requests to /personalize with "role" key continue to return HTTP 200 (no regression).

- **SC-004**: Frontend requests to /personalize without any role key default to "student" and return HTTP 200 with a general explanation.

- **SC-005**: All personalize requests log the received JSON data to console for debugging visibility.

- **SC-006**: Existing /translate and /chat endpoints continue to function without any changes or regressions.

- **SC-007**: Response time for /personalize endpoint matches previous performance (~2-5 seconds for OpenAI completion).

## Assumptions

- The frontend may use different JSON key names due to naming conventions in different frameworks or legacy code.
- The priority order (role → background → userRole → "student") covers the most common key name variations.
- Defaulting to "student" when no role is specified provides a reasonable general explanation suitable for beginners.
- The topic field is always present (if missing, the endpoint should return an appropriate error).
- Manual extraction of JSON fields is acceptable for this simple use case without introducing additional validation libraries.
- The existing run_agent function can handle "student" as a role value effectively.

## Dependencies

- Existing run_agent function in backend/agent.py
- FastAPI Request object for accessing raw JSON body
- Existing error handling and logging patterns

## Out of Scope

- Adding frontend-side validation to enforce specific JSON key names
- Creating a comprehensive schema validation system for all possible key variants
- Implementing automatic key name normalization across all endpoints
- Adding request/response logging to a persistent logging system
- Creating automated tests for all key name combinations
- Modifying the /translate endpoint (not affected by this issue)
- Changing the agent.py logic or system prompt
