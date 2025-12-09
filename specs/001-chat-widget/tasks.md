# Tasks: Physical AI Tutor Chat Widget with Text Selection Feature

**Feature**: Physical AI Tutor Chat Widget with Text Selection Feature
**Branch**: `001-chat-widget`
**Created**: 2025-12-09
**Status**: Active

## Implementation Strategy

**MVP Approach**: Start with User Story 1 (core chat functionality) as the minimum viable product, then incrementally add text selection features and cross-chapter persistence.

**Delivery Order**:
1. Phase 1: Setup foundational components
2. Phase 2: Core chat functionality
3. Phase 3: Text selection feature
4. Phase 4: Layout integration and persistence
5. Phase 5: Polish and optimization

**Parallel Execution Opportunities**: UI components (ChatWindow, SelectionTooltip) can be developed in parallel with backend integration components.

## Dependencies

- **User Story 2** depends on successful completion of **User Story 1** (core functionality)
- **User Story 3** depends on successful completion of **User Story 1** (markdown rendering)
- **User Story 3** depends on **User Story 2** (persistence across chapters)

## Parallel Execution Examples

**Per User Story**:
- **User Story 1**: UI components (ChatWindow.js, styles.module.css) can be developed in parallel with API integration (index.js)
- **User Story 2**: Layout integration can be developed in parallel with session persistence logic
- **User Story 3**: Markdown rendering can be developed in parallel with rich content handling

## Phase 1: Setup

### Goal
Initialize the project structure and install necessary dependencies for the ChatBot component.

### Independent Test Criteria
No independent test - this phase sets up prerequisites for all other phases.

### Tasks

- [X] T001 Create project structure per implementation plan in frontend/src/components/ChatBot/
- [X] T002 [P] Install react-markdown dependency for markdown rendering in package.json
- [X] T003 [P] Install remark-gfm dependency for GitHub Flavored Markdown in package.json
- [X] T004 [P] Install react-icons dependency for icon components in package.json
- [X] T005 Create ChatBot component directory structure in frontend/src/components/ChatBot/

## Phase 2: Foundational Components

### Goal
Implement core chat functionality with basic UI, API integration, and message handling.

### Independent Test Criteria
Student can click the floating AI tutor button, type a question about the handbook content, receive a properly formatted response from the AI, and return to reading without losing their place.

### Tasks

- [X] T006 [P] [US1] Create main ChatBot component in frontend/src/components/ChatBot/index.js
- [X] T007 [P] [US1] Create ChatWindow subcomponent in frontend/src/components/ChatBot/ChatWindow.js
- [X] T008 [P] [US1] Create CSS modules for styling in frontend/src/components/ChatBot/styles.module.css
- [X] T009 [US1] Implement floating button UI in index.js with robot icon
- [X] T010 [US1] Implement chat window UI with glassmorphism effect in ChatWindow.js
- [X] T011 [US1] Implement state management for chat messages in index.js
- [X] T012 [US1] Implement API integration to send questions to http://localhost:8000/chat in index.js
- [X] T013 [US1] Implement "Typing..." indicator display in ChatWindow.js
- [X] T014 [US1] Implement message display functionality in ChatWindow.js
- [X] T015 [US1] Implement basic send message functionality in index.js
- [X] T016 [US1] Implement error handling for API calls in index.js
- [ ] T017 [US1] Test basic chat functionality with mock API responses

## Phase 3: Text Selection Feature

### Goal
Add the "Ask about selection" functionality that detects when users highlight text and provides a contextual tooltip.

### Independent Test Criteria
When users select text on any handbook page, a small "Ask AI" tooltip appears near the selected text, and clicking it opens the chat window with the selected text as context.

### Tasks

- [X] T018 [P] [US1] Create SelectionTooltip component in frontend/src/components/ChatBot/SelectionTooltip.js
- [X] T019 [US1] Implement text selection detection using window.getSelection() in index.js
- [X] T020 [US1] Implement selectionchange event listener in index.js
- [X] T021 [US1] Implement tooltip positioning using getBoundingClientRect() in SelectionTooltip.js
- [X] T022 [US1] Implement "Ask AI" button in SelectionTooltip.js
- [X] T023 [US1] Implement tooltip visibility logic in index.js
- [X] T024 [US1] Implement contextual query formatting in index.js
- [X] T025 [US1] Implement tooltip delay configuration in index.js
- [X] T026 [US1] Test text selection detection and tooltip display
- [X] T027 [US1] Test contextual query formatting with selected text

## Phase 4: Layout Integration & Persistence

### Goal
Integrate the ChatBot component into the Docusaurus layout so it persists across all handbook chapters.

### Independent Test Criteria
Student can navigate from one chapter to another and find the AI tutor widget consistently available in the same position.

### Tasks

- [X] T028 [US2] Integrate ChatBot component with Docusaurus layout in docusaurus.config.js
- [X] T029 [US2] Implement session persistence across page navigation using localStorage
- [X] T030 [US2] Implement chat window state persistence across page navigation
- [X] T031 [US2] Test component persistence across different handbook chapters
- [X] T032 [US2] Test session continuity when navigating between chapters
- [X] T033 [US2] Implement proper cleanup of event listeners on component unmount

## Phase 5: Rich Content Handling

### Goal
Ensure AI responses with code blocks and markdown are properly formatted and displayed.

### Independent Test Criteria
Student can ask questions that result in complex responses containing code, and all content is properly formatted and readable.

### Tasks

- [X] T034 [P] [US3] Implement react-markdown integration in ChatWindow.js
- [X] T035 [US3] Implement remark-gfm plugin for GitHub Flavored Markdown in ChatWindow.js
- [X] T036 [US3] Implement code block syntax highlighting in ChatWindow.js
- [X] T037 [US3] Test markdown rendering with various content types
- [X] T038 [US3] Test code block rendering with Python/ROS 2 examples
- [X] T039 [US3] Implement scrolling for long AI responses in ChatWindow.js
- [X] T040 [US3] Test rich content rendering with complex examples

## Phase 6: Polish & Cross-Cutting Concerns

### Goal
Finalize the component with error handling, performance optimization, and mobile responsiveness.

### Independent Test Criteria
Component works reliably across all scenarios with proper error handling and responsive design.

### Tasks

- [X] T041 Implement error handling for network issues in index.js
- [X] T042 Implement graceful degradation when backend is unavailable
- [X] T043 Implement mobile-responsive design in styles.module.css
- [X] T044 Optimize performance for text selection detection
- [X] T045 Implement accessibility features for screen readers
- [X] T046 Test component across different browsers (Chrome, Firefox, Safari, Edge)
- [X] T047 Test component on different screen sizes and mobile devices
- [X] T048 Document component usage and configuration options
- [X] T049 Run full build process with `npm run build` to verify integration
- [X] T050 Final testing of all user stories and acceptance criteria