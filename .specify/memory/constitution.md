# Physical AI & Humanoid Robotics Handbook Constitution

## Core Principles

### I. User-Centric Design
Prioritize intuitive user experience and mobile responsiveness for the Docusaurus site. Ensure content is easily accessible and code examples are runnable.

### II. Modularity & Scalability
Design Part 2 features (RAG chatbot, login, localization) with modularity to allow for future extensions and scalability.

### III. Comprehensive Content
The handbook must cover exactly 14 chapters, aligning with the 13-week course and capstone project outline.

### IV. Global Accessibility
Implement features like Urdu language support (Part 2) to ensure the handbook is accessible to a diverse global audience.

### V. Continuous Delivery
Aim for a live GitHub Pages deployment today. No feature is "done" until it is deployed and verified.

## Architecture & Integrity

### VI. Strict Separation of Concerns
* **`/frontend`**: Reserved strictly for the Docusaurus v3 application (React/TypeScript/Markdown). No Python code allowed here.
* **`/backend`**: Reserved for the Python RAG Chatbot (FastAPI). No UI code allowed here.

### VII. Technology Stack
* **Frontend**: Docusaurus v3, React, JavaScript (ES6+).
* **Backend**: Python 3.10+, FastAPI, OpenAI Agent SDK.
* **Deployment**: GitHub Pages (Frontend).

## Development Workflow

### VIII. SpecKit Protocol (History Mandate)
* **Plan Before Code**: Every modification must start with a `/plan` command to generate a documented strategy in the history.
* **Verification**: No task is marked "Complete" until `npm run build` (for frontend) passes successfully.

## Governance

This Constitution guides all project development. Amendments require team consensus and version increment. Compliance is verified through regular code reviews.

**Version**: 1.1.0 | **Ratified**: 2025-12-08 | **Status**: Active