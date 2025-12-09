# Implementation Plan: Physical AI Tutor Chat Widget with Text Selection Feature

**Branch**: `001-chat-widget` | **Date**: 2025-12-09 | **Spec**: [specs/001-chat-widget/spec.md]
**Input**: Feature specification from `/specs/001-chat-widget/spec.md` with additional "Ask about selection" requirement

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a Chatbot Widget component that integrates with the Docusaurus v3 frontend to provide AI-powered assistance to users reading the Physical AI & Humanoid Robotics handbook. The widget will connect to a backend chat API at `http://localhost:8000/chat` and feature a glassmorphism UI with electric blue theming. The component will include an advanced "Ask about selection" feature that detects when users highlight text on the page and provides a contextual "Ask AI" tooltip or auto-quotes the selected text in the chat input. The component will be implemented as a React component with proper markdown rendering for code blocks and integrated into the Docusaurus layout to persist across all handbook chapters.

## Technical Context

**Language/Version**: JavaScript (ES6+), React 18+ for Docusaurus v3 compatibility
**Primary Dependencies**: Docusaurus v3, React, react-markdown, remark-gfm, react-icons, CSS Modules, DOM selection APIs
**Storage**: Browser localStorage for session persistence (optional), N/A for core functionality
**Testing**: Jest for unit tests, React Testing Library for component tests
**Target Platform**: Web browser (Chrome, Firefox, Safari, Edge) - compatible with Docusaurus deployment
**Project Type**: Web application frontend component integrated with Docusaurus
**Performance Goals**: <200ms response time for UI interactions, <500ms for API calls to backend, minimal impact on page selection performance
**Constraints**: Must follow Docusaurus v3 architecture, maintain mobile responsiveness, comply with existing CSS styling, not interfere with normal text selection
**Scale/Scope**: Single component serving all handbook users, supporting concurrent chat sessions per user, working across all handbook chapters

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Pre-Design Compliance Verification:
- **VI. Strict Separation of Concerns**: ✅ Component will be in `/frontend` directory only, no Python code included
- **VII. Technology Stack**: ✅ Using React/JavaScript as specified for frontend
- **I. User-Centric Design**: ✅ Focuses on intuitive user experience with mobile responsiveness and enhanced selection feature
- **II. Modularity & Scalability**: ✅ Component designed to be modular and scalable for future enhancements
- **V. Continuous Delivery**: ✅ Component will be built and tested with `npm run build` process

### Post-Design Compliance Verification:
- **VI. Strict Separation of Concerns**: ✅ All code remains in frontend directory with no backend dependencies in the component
- **VII. Technology Stack**: ✅ All dependencies (react-markdown, remark-gfm, react-icons) are frontend libraries compatible with Docusaurus
- **I. User-Centric Design**: ✅ Design includes responsive layout, accessibility considerations, and improved text selection feature
- **II. Modularity & Scalability**: ✅ Component architecture supports future enhancements and theming options
- **V. Continuous Delivery**: ✅ Component integrates with existing build process without modifications

### Gates Status: ALL PASSED - No violations identified

## Project Structure

### Documentation (this feature)

```text
specs/001-chat-widget/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

frontend/
├── src/
│   └── components/
│       └── ChatBot/
│           ├── index.js              # Main ChatBot component with selection detection
│           ├── SelectionTooltip.js   # Tooltip component for selected text
│           ├── ChatWindow.js         # Chat window subcomponent
│           └── styles.module.css     # CSS modules for styling (glassmorphism + electric blue)
└── docusaurus.config.js              # Configuration for layout integration

**Structure Decision**: Component-based structure following Docusaurus v3 conventions with the ChatBot component placed in the standard components directory. The component includes specialized sub-components for text selection handling. Integration with Docusaurus layout will be achieved through theme customization or layout wrapper.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A - All constitution checks passed |

## Additional Considerations for Text Selection Feature

| Complexity | Impact | Mitigation Strategy |
|------------|--------|-------------------|
| DOM Selection API Integration | Medium | Use standard window.getSelection() API with proper event listeners |
| Tooltip Positioning | Medium | Calculate position based on selection bounds with viewport constraints |
| Performance Impact | Low | Debounce selection events and use efficient rendering |
| Mobile Compatibility | High | Implement touch-based selection handling for mobile devices |
