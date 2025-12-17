# Implementation Plan: Fix RAG Chatbot Deployment on Render

**Branch**: `004-fix-chat-render` | **Date**: 2025-12-18 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-fix-chat-render/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

**Primary Requirement**: Fix the RAG chatbot `/chat` endpoint to work on Render production deployment while maintaining localhost functionality. The chatbot works locally but fails on Render with "API Error/No Response", while translation and personalization endpoints work correctly.

**Technical Approach**:
1. Modify `backend/server.py` to use dynamic PORT from environment variable with fallback
2. Ensure all async operations in `agent.py` are properly awaited
3. Verify response JSON key consistency (`answer`) between backend and frontend
4. Ensure cross-platform path compatibility using `os.path.join()` or pathlib
5. Validate CORS configuration for production deployment
6. Add comprehensive logging for debugging on Render platform

## Technical Context

**Language/Version**: Python 3.10+ (backend), JavaScript/React 19 (frontend)
**Primary Dependencies**: FastAPI, OpenAI SDK, Qdrant Client, Cohere, uvicorn, python-dotenv (backend); Docusaurus 3.9.2, Framer Motion (frontend)
**Storage**: Qdrant Cloud (vector database for RAG context retrieval), localStorage (frontend auth)
**Testing**: Manual testing on localhost and Render production deployment
**Target Platform**: Render.com (Linux Ubuntu server) for backend, GitHub Pages for frontend
**Project Type**: Web application (separate backend/frontend)
**Performance Goals**: 5-second maximum response time, 50 concurrent users support, 95% success rate
**Constraints**: Must work identically on localhost (Windows) and Render (Linux), must use dynamic PORT assignment
**Scale/Scope**: Educational handbook with RAG chatbot serving ~100 active users

**Current Issue Analysis**:
- **Root Cause**: Port binding issue - `server.py` uses PORT from environment at lines 102-103, but the uvicorn.run() call at line 104 may not be executing in production
- **Response Key**: Already correct - backend returns `{"answer": response}` at line 45, frontend expects `data.answer` at line 123
- **Async Operations**: Already correct - `run_agent()` is properly awaited at line 44
- **CORS**: Already correct - `allow_origins=["*"]` at line 15, `allow_credentials=True` at line 16
- **File Paths**: No file operations in `agent.py` or `server.py` that would cause Linux compatibility issues

**Working Endpoints for Comparison**:
- `/translate` (lines 50-62): Works on Render - uses same `await run_agent()` pattern
- `/personalize` (lines 64-99): Works on Render - uses same `await run_agent()` pattern
- Both have identical error handling and return structure

**Hypothesis**: The `/chat` endpoint code is actually correct. The issue may be related to how Render starts the application (using a different command than `python server.py`), or there may be environment-specific differences in error handling that cause silent failures.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Principle VI - Strict Separation of Concerns**: ✅ PASS
- Changes confined to `/backend` directory only (`server.py`, `agent.py`)
- No modifications to `/frontend` directory required
- No mixing of Python code in frontend or UI code in backend

**Principle VII - Technology Stack**: ✅ PASS
- Backend: Python 3.10+, FastAPI (existing stack)
- Frontend: No changes required (Docusaurus v3, React)
- Deployment: Render.com for backend (existing), GitHub Pages for frontend (existing)

**Principle VIII - SpecKit Protocol**: ✅ PASS
- Feature started with `/sp.specify` command
- Currently executing `/sp.plan` command
- Will complete with `/sp.tasks` before implementation
- Full documentation trail in `specs/004-fix-chat-render/`

**Principle V - Continuous Delivery**: ✅ PASS
- Fix targets immediate production deployment on Render
- No feature is "done" until verified on live deployment
- Testing plan includes verification on https://physical-ai-backend-j8q9.onrender.com

**Result**: All constitutional gates PASSED - proceeding to Phase 0 research.

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
├── server.py          # FastAPI application with /chat, /translate, /personalize endpoints
├── agent.py           # OpenAI GPT-4o agent with RAG system prompt
├── retriveing.py      # Qdrant vector search retrieval logic
├── main.py            # Data ingestion pipeline (sitemap scraping)
├── requirements.txt   # Python dependencies
└── .env               # Environment variables (not in git)

frontend/
├── src/
│   └── components/
│       └── ChatBot/
│           ├── index.js        # Main ChatBot component (calls /chat endpoint)
│           └── ChatWindow.js   # Chat UI subcomponent
├── docs/              # Handbook markdown content
├── docusaurus.config.js
└── package.json

.specify/
└── [project management files]

specs/004-fix-chat-render/
├── spec.md           # Feature specification
├── plan.md           # This file
├── research.md       # Phase 0 output (to be created)
├── data-model.md     # Phase 1 output (to be created)
├── quickstart.md     # Phase 1 output (to be created)
└── contracts/        # Phase 1 output (to be created)
```

**Structure Decision**: Web application (Option 2) - This is a deployment fix for the existing backend API. Changes are isolated to `backend/server.py` and `backend/agent.py` only. No frontend modifications required since the ChatBot component already correctly calls the `/chat` endpoint and expects `data.answer` in the response.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations detected - all constitutional principles are satisfied.
