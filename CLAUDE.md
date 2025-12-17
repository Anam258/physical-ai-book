# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **Physical AI & Humanoid Robotics Handbook** - an educational platform built with Docusaurus v3 that provides interactive learning content with an AI-powered tutoring chatbot. The project consists of:

-   **Frontend**: Docusaurus-based static site with React components
-   **Backend**: FastAPI server with RAG (Retrieval-Augmented Generation) capabilities

## Architecture

### Frontend (Docusaurus v3)
-   **Framework**: Docusaurus 3.9.2 with React 19
-   **Key Components**:
    -   `frontend/src/components/ChatBot/` - Main AI tutor chat interface with selection tooltip feature
    -   `frontend/src/components/Auth/` - Custom Authentication System (Mock/LocalStorage)
    -   `frontend/src/components/HomepageFeatures/` - Landing page components
    -   `frontend/docs/` - Handbook content organized by modules (ROS2, Simulation, Isaac Sim, VLA/Voice, Advanced Topics)

-   **Configuration**:
    -   `docusaurus.config.js` - Site config, deployment to GitHub Pages (Anam258/physical-ai-book)
    -   `sidebars.js` - Documentation navigation structure
    -   Default dark mode enabled

### Backend (FastAPI + RAG)
-   **API Server**: `backend/server.py` - FastAPI with CORS enabled, single `/chat` endpoint
-   **AI Agent**: `backend/agent.py` - OpenAI GPT-4o integration with RAG system prompt
-   **Data Ingestion**: `backend/main.py` - Scrapes sitemap, chunks content, creates Qdrant embeddings
-   **Retrieval**: `backend/retriveing.py` - Qdrant vector search using Cohere embeddings
-   **Stack**:
    -   Vector DB: Qdrant Cloud
    -   Embeddings: Cohere `embed-english-v3.0` (1024 dimensions)
    -   LLM: OpenAI GPT-4o
    -   Sitemap source: `https://anam258.github.io/physical-ai-book/sitemap.xml`

### Data Flow
1.  User asks question via ChatBot component
2.  Request sent to Render deployment: `https://physical-ai-backend-j8q9.onrender.com/chat`
3.  Backend retrieves relevant chunks from Qdrant (top 5 results)
4.  GPT-4o generates answer using retrieved context
5.  Response streamed back to frontend

## Commands

### Frontend Development
```bash
cd frontend
npm install              # Install dependencies
npm start               # Start dev server (default: localhost:3000)
npm run build           # Build for production
npm run serve           # Serve production build
npm run clear           # Clear Docusaurus cache
npm run swizzle         # For component overrides

# Deployment to GitHub Pages
$env:GIT_USER="Anam258"; npm run deploy    # Windows PowerShell
GIT_USER="Anam258" npm run deploy          # Linux/Mac


### Backend Development
```bash
cd backend
pip install -r requirements.txt   # Install Python dependencies

# Development
python server.py                  # Run FastAPI server (port 8000)
python main.py                    # Run data ingestion pipeline
python retriveing.py              # Test retrieval system
python agent.py                   # Test agent directly

# Production
python -m uvicorn server:app --host 0.0.0.0 --port 8000

## Environment Variables

**Backend** requires a `.env` file with the following keys:
```env
OPENAI_API_KEY=...
COHERE_API_KEY=...
QDRANT_URL=...
QDRANT_API_KEY=...
Bilkul Anum, yeh raha baqi ka hissa. Aap isay Environment Variables wale section ke foran baad paste kar dein.

(Maine note kiya ke aapne jahan roka wahan code block band nahi hua tha, isliye maine shuru mein ``` lagaya hai taake format na toote).

Yahan se copy karein aur end mein paste karein:

Frontend:

⚠️ Do NOT use process.env. Hardcode the API URL in index.js:

Local: http://localhost:8000/chat

Production: https://physical-ai-backend-j8q9.onrender.com/chat

Key Patterns & Agent Skills
ChatBot Component Architecture
Text Selection Feature: When text is selected on the page, a tooltip appears allowing users to ask questions about it.

Animations: Uses Framer Motion for smooth UI interactions.

State Management: Stores messages in React state (no persistence by default).

⚡ Important: When text is selected, immediately open the ChatBot window and automatically trigger handleSend with the selected context. Clear context states immediately after API initiation to prevent hangs.

Backend RAG System
Collection Name: humanoid_ai_book (must match between main.py and retriveing.py).

Chunk Size: 1000 characters with sentence boundary splitting.

Retrieval Limit: 5 most relevant chunks.

Temperature: 0.2 for consistent answers.

Skill 4: Authentication & User Background Collection
Authentication System: Uses Custom React Components (Mock System) to mimic Better-Auth UX. Does NOT use @better-auth library to avoid backend conflicts.

User Registration: Collects user's software background (Python, JS, etc.) and hardware background (Arduino, ROS, etc.) during sign-up.

User Session Management: Stores authentication state and user background in localStorage with AuthProvider context.

Glassmorphism UI: Auth modal features premium glassmorphism design with blur effects and gradient backgrounds.

Navbar Integration: Shows "Login / Sign Up" button when logged out, displays user's initial/avatar when logged in.

Portal Rendering: Auth Modal must render outside the root DOM using React Portals to avoid Z-index issues.

Personalization: User background information is stored to tailor AI tutor responses.

Skill 5: Logout Experience Patterns
Logout Logic: Use localStorage.clear() (not removeItem) for complete session cleanup. Always follow sequence: clear() → console.log('User logged out') → window.location.href = '/'.

Confirmation Modals: Use React Portal (ReactDOM.createPortal) to render modals directly under document.body, preventing z-index conflicts.

Event Handling: Implement Escape key (window.addEventListener('keydown')) and outside click (onClick on overlay with stopPropagation on content) for modal dismissal. Always cleanup event listeners in useEffect return function.

Body Scroll: Prevent background scrolling when modal is open using document.body.style.overflow = 'hidden', restore with 'unset' on close.

Glassmorphism Modal Pattern: Use backdrop-filter: blur(12px), navy background (#1e293b), neon border (gradient cyan #06b6d4 to pink #ec4899), grey Cancel button (#475569), red/pink gradient Logout button.

Single Instance Guarantee: Use single boolean state (isModalOpen) and guard clause in click handler to prevent modal stacking from rapid clicks.

Adding New Handbook Content
Create markdown files in frontend/docs/[Module-Name]/.

Follow existing module structure (01-Introduction, Module-01-ROS2, etc.).

Run npm start to preview changes.

Rebuild and redeploy to trigger sitemap update.

Run backend/main.py to re-ingest content into Qdrant.

Code Style Guidelines
React: Use functional components and hooks.

API: Use async/await for all fetch calls.

Async Operations: Ensure await is used with run_agent() in backend.

Skill 6: Form Validation Patterns
Real-time Validation: Implement onChange validation for password length and onBlur validation for email format to optimize user experience.

Validation State Management: Use React useState to manage validation errors with structure:
{
  email: string,
  password: string,
  name: string,
  softwareBackground: string,
  hardwareBackground: string
}

Error Display: Show error messages below fields with .error-text CSS class and apply .input-error class to invalid fields for red border feedback.

Visual Feedback: Use red border color (#ef4444) for invalid fields and implement shake animation for submit button when form has validation errors.

Validation Rules: Email must contain "@" and "." (regex: /^[\w.-]+@[\w.-]+\.\w+$/), password must be at least 6 characters, required fields must not be empty.

Submission Prevention: Block form submission if validation errors exist and provide clear user feedback.

Skill 7: Dark Glassmorphism Homepage Features
Feature Cards: Homepage now displays 3 key features with dark glassmorphism styling: '🤖 AI RAG Tutor', '📝 Smart Quizzes', and '🗣️ Urdu & English'.

Design Specifications: Uses background: rgba(30, 41, 59, 0.7), backdrop-filter: blur(10px), border: 1px solid rgba(0, 247, 255, 0.2).

Typography: White bold titles (color: #ffffff, font-weight: 700) with light grey descriptions (color: #cbd5e1).

Hover Effects: Cards scale with translateY(-5px) and display glow border (#00f7ff) on hover for enhanced interactivity.

Skill 8: Premium Homepage Features Design Polish
Icon Implementation: Uses inline SVGs for feature icons instead of external files, wrapped in .iconWrapper divs for enhanced styling.

Icon Styling: .iconWrapper is styled as a circle (60px) with neon gradient background: linear-gradient(135deg, #00f7ff 0%, #0066cc 100%) and centered SVG content.

Card Enhancement: .glassCard now uses vertical gradient background: linear-gradient(180deg, rgba(30, 41, 59, 0.8) 0%, rgba(15, 23, 42, 0.9) 100%).

Visual Depth: Enhanced shadows with box-shadow: 0 8px 32px 0 rgba(0, 0, 0, 0.37) for premium appearance.

Result: Icons appear as glowing buttons floating above the glass cards for a premium visual effect.

Skill 9: Hero Section Layout Balance Fix
Layout Structure: .heroContainer uses gap: 2rem to create proper space between text and robot elements.

Text Proportions: .heroText uses flex: 1.2 (60%) with padding-right: 2rem to give text adequate breathing room.

Robot Proportions: .heroRobotContainer uses flex: 0.8 (40%) with centering properties to make robot prominent but not overwhelming.

Responsive Design: On mobile (max-width: 996px), flex-direction is column-reverse (text on top), padding-right removed from text, and robot max-width reduced to 350px.

Image Sizing: .hero-robot-static max-width reduced from 600px to 450px (desktop) and 350px (mobile).

Result: Balanced layout where text has breathing room and robot is prominent but not overwhelming.

Skill 10: Personalization Feature
Backend Endpoint: Added POST /personalize endpoint accepting {topic, user_background} parameters.

AI Generation: Uses OpenAI to generate 2-sentence analogies explaining topics based on user's background.

Frontend Component: Created <PersonalizationCard topic="" /> component with localStorage authentication check.

UI Logic: Shows "🔒 Login to Personalize" when logged out (opens AuthModal), "✨ Explain for My Background" when logged in.

Display: Results shown in dark glassmorphism card with navy background and cyan border.

Goal: Enables users to get context-aware summaries inside chapters based on their software/hardware background.

Skill 11: Formal Urdu Script Language Support
Language Detection: AI tutor automatically detects query language (English vs Urdu script) based on character analysis (Arabic script = Urdu, Latin script = English).

Bilingual Responses: System responds in the same language as the query - English queries receive English responses, Urdu script queries receive Urdu script responses.

Technical Term Preservation: ALL technical terms remain in English regardless of response language (ROS 2, Python, Node, C++, JavaScript, DOF, LIDAR, IMU, API, actuator, sensor, servo, Gazebo, Isaac Sim, URDF, TF2, publisher, subscriber, service, action, topic, namespace, parameter, launch file).

Code Block Handling: Code blocks and code comments ALWAYS remain in English, even when surrounding explanatory text is in Urdu script.

Urdu Response Quality: Uses formal, professional Pakistani Urdu conventions suitable for educational content.

Roman Urdu Removed: Latin-script Urdu queries (e.g., "Robot kaise kaam karta hai?") are treated as English (Roman Urdu support explicitly removed).

Implementation: System prompt enhancement in backend/agent.py - zero new dependencies, GPT-4o native multilingual capabilities.

Deployment
Frontend
Host: GitHub Pages at https://Anam258.github.io/physical-ai-book/

Deploy Command: npm run deploy with GIT_USER="Anam258" env var.

Branch: Uses gh-pages branch.

Base URL: /physical-ai-book/

Backend
Host: Render at https://physical-ai-backend-j8q9.onrender.com

Strategy: Auto-deploys from main branch.

Node Version
Requires Node.js >= 20.0 (specified in frontend/package.json).

Important Notes
⚠️ Git History: Frontend references removed files in git history - check git status before committing.

🔓 CORS: Wide open (allow_origins=["*"]) - acceptable for hackathon/educational use.

🗑️ Qdrant: Collection is recreated on each ingestion run (destructive operation).

📁 Specs: The project uses a .specify/ directory for project planning and feature specifications. Do not create specs in root.
