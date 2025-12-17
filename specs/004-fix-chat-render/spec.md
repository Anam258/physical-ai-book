# Feature Specification: Fix RAG Chatbot Deployment on Render

**Feature Branch**: `004-fix-chat-render`
**Created**: 2025-12-18
**Status**: Draft
**Input**: User description: "Issue: The RAG Chatbot (/chat endpoint) works on Localhost but fails on Render (API Error/No Response). Context: Translation and Personalization work fine on Render. Based on CLAUDE.md, please: 1. Fix `backend/server.py` to use dynamic PORT (os.environ.get("PORT")). 2. Ensure the `/chat` response key matches the frontend (data.answer). 3. Check `backend/agent.py` for any absolute paths that fail on Linux/Render. 4. Verify CORS settings are fully open for production."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Chat Functionality Works on Production (Priority: P1)

Students and learners access the deployed handbook on GitHub Pages and attempt to use the AI tutor chatbot to ask questions about Physical AI and Humanoid Robotics content. The chatbot should successfully retrieve context from the Qdrant vector database and provide accurate, contextual responses powered by the RAG system.

**Why this priority**: This is the core value proposition of the handbook - an interactive AI tutor. Without this working on production, users cannot receive personalized assistance, making the deployed site significantly less valuable than the local version.

**Independent Test**: Can be fully tested by opening the production site at https://Anam258.github.io/physical-ai-book/, clicking the chat button, typing "What is ROS 2?", and verifying that a relevant response is returned within 5 seconds. Delivers immediate value by restoring the primary interactive feature.

**Acceptance Scenarios**:

1. **Given** a user visits the production handbook site, **When** they open the chatbot and send a question about ROS 2 fundamentals, **Then** the system retrieves relevant context from Qdrant and returns an accurate answer within 5 seconds
2. **Given** the chatbot is active on production, **When** a user asks multiple questions in sequence, **Then** each question is processed successfully without errors or timeouts
3. **Given** concurrent users are accessing the chatbot, **When** each sends a question simultaneously, **Then** all users receive responses without system failures or degraded performance

---

### User Story 2 - Error Handling Provides Clear Feedback (Priority: P2)

When the backend encounters issues (network problems, API rate limits, or internal errors), users should see helpful error messages that explain what went wrong and suggest next steps, rather than generic "API Error" messages or silent failures.

**Why this priority**: Good error handling improves user trust and reduces support burden. While core functionality (P1) must work first, clear error messages significantly enhance the user experience when problems occur.

**Independent Test**: Can be tested independently by simulating various failure scenarios (disconnecting from Qdrant, using invalid API keys, timing out requests) and verifying that users see specific, actionable error messages. Delivers value by improving troubleshooting and user communication.

**Acceptance Scenarios**:

1. **Given** the Qdrant vector database is temporarily unavailable, **When** a user sends a chat query, **Then** the system returns a user-friendly message explaining the temporary unavailability and suggesting to try again shortly
2. **Given** the OpenAI API rate limit is exceeded, **When** a user sends a query, **Then** the system provides a clear message about the rate limit and estimated wait time
3. **Given** a network timeout occurs during processing, **When** the request exceeds 30 seconds, **Then** the user receives a timeout message with option to retry

---

### User Story 3 - System Monitoring and Logging (Priority: P3)

Administrators and developers can monitor the health of the production deployment through logs and metrics, allowing them to proactively identify and resolve issues before they impact users.

**Why this priority**: While critical for long-term maintainability, monitoring doesn't directly impact immediate user experience. It becomes important after core functionality (P1) and error handling (P2) are working.

**Independent Test**: Can be tested by checking Render logs after sending test queries, verifying that request/response cycles, errors, and performance metrics are properly logged. Delivers value by enabling proactive maintenance and troubleshooting.

**Acceptance Scenarios**:

1. **Given** the chatbot is deployed on Render, **When** users send queries, **Then** each request is logged with timestamp, user query (sanitized), response time, and success/failure status
2. **Given** an error occurs in the backend, **When** the system catches the exception, **Then** detailed error information (type, stack trace, context) is logged for developer review
3. **Given** the system has been running for 24 hours, **When** administrators review logs, **Then** they can identify performance patterns, error rates, and usage metrics

---

### Edge Cases

- What happens when Render restarts the server and port assignment changes dynamically?
- How does the system handle requests when environment variables (PORT, OPENAI_API_KEY, etc.) are missing or invalid?
- What occurs if the frontend expects `data.answer` but the backend returns `data.response` due to inconsistent naming?
- How does the system behave when file paths use Windows-style backslashes on a Linux production environment?
- What happens if CORS headers are too restrictive and block legitimate frontend requests?
- How does the system handle streaming responses that are interrupted mid-stream?
- What occurs when the Qdrant vector database returns no relevant results for a query?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Backend server MUST bind to the PORT specified in the environment variable `os.environ.get("PORT")` when deployed on Render, with fallback to port 8000 for local development
- **FR-002**: The `/chat` endpoint response MUST return JSON with the key `answer` (not `response` or other variants) to match frontend expectations in `ChatBot/index.js`
- **FR-003**: All file path operations in `backend/agent.py` and `backend/server.py` MUST use `os.path.join()` or pathlib to ensure Linux/Windows compatibility
- **FR-004**: CORS middleware MUST allow all origins (`allow_origins=["*"]`) and include credentials support for production deployment
- **FR-005**: The `/chat` endpoint MUST handle both streaming and non-streaming response modes without breaking when deployed to Render's Linux environment
- **FR-006**: Backend MUST validate that all required environment variables (OPENAI_API_KEY, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY) are present on startup and log clear error messages if missing
- **FR-007**: All async operations in `agent.py` MUST be properly awaited to prevent "coroutine was never awaited" warnings or silent failures
- **FR-008**: The system MUST return appropriate HTTP status codes (200 for success, 500 for server errors, 400 for bad requests) with consistent error response format
- **FR-009**: Backend MUST handle streaming responses from OpenAI API correctly, yielding chunks in the format expected by the frontend
- **FR-010**: System MUST log all incoming chat requests, responses, and errors to Render's stdout for debugging and monitoring

### Key Entities

- **Chat Request**: User query sent from frontend, contains message text and optional context from text selection
- **RAG Response**: Generated answer that includes retrieved context from Qdrant vector database and LLM-generated explanation
- **Environment Configuration**: Collection of environment variables (PORT, API keys, database URLs) required for production deployment
- **Error Response**: Structured error message returned to frontend containing error type, user-friendly message, and optional debug information

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can successfully send chat queries on the production site (https://Anam258.github.io/physical-ai-book/) and receive accurate responses within 5 seconds, with 95% success rate
- **SC-002**: The backend server starts successfully on Render using dynamically assigned PORT without manual intervention or hardcoded port values
- **SC-003**: Chat functionality works identically on localhost:8000 and production deployment, with zero environment-specific bugs
- **SC-004**: All chat requests return responses with the correct JSON key structure that matches frontend expectations (data.answer), achieving 100% consistency
- **SC-005**: System handles at least 50 concurrent users sending chat queries without failures, timeouts, or degraded response times
- **SC-006**: Error scenarios (API failures, timeouts, missing keys) provide clear user-facing messages, reducing "unknown error" occurrences to zero
- **SC-007**: Deployment process completes successfully on Render without requiring code changes between local development and production environments

## Assumptions

- Render deployment uses a Linux-based environment (Ubuntu or similar)
- Environment variables are correctly configured in Render dashboard
- The frontend code at `frontend/src/components/ChatBot/index.js` expects response key to be `answer`
- Translation (`/translate`) and Personalization (`/personalize`) endpoints are already working correctly on Render, indicating basic server and environment setup is functional
- Qdrant Cloud and OpenAI API are accessible from Render's network
- The issue is specifically with the `/chat` endpoint, not with authentication, CORS preflight, or network connectivity in general

## Dependencies

- **External Services**: OpenAI API (GPT-4o), Cohere API (embeddings), Qdrant Cloud (vector database)
- **Deployment Platform**: Render.com with automatic deploys from main branch
- **Frontend Deployment**: GitHub Pages must remain accessible to send requests to Render backend
- **Environment Variables**: Must be configured in Render dashboard before deployment succeeds

## Out of Scope

- Redesigning the RAG system architecture or changing retrieval algorithms
- Migrating to a different deployment platform (e.g., AWS, Vercel, Railway)
- Implementing authentication or rate limiting for the chat endpoint
- Adding caching layers (Redis) or performance optimizations beyond fixing the deployment issue
- Modifying the frontend ChatBot component UI or user experience
- Changing the vector database (Qdrant) or embedding model (Cohere)
- Implementing WebSocket-based streaming instead of HTTP streaming
