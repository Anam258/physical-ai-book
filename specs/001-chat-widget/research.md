# Research Summary: Physical AI Tutor Chat Widget with Text Selection Feature

## Decision: Component Architecture
**Rationale**: Using a React component approach with sub-components for maintainability and following Docusaurus v3 conventions. The main ChatBot component will manage state and API communication, while ChatWindow will handle the UI rendering. The new SelectionTooltip component will handle text selection detection and UI.

**Alternatives considered**:
- Full custom element approach - rejected due to complexity and lack of React ecosystem benefits
- Third-party chat widget - rejected due to customization requirements for glassmorphism and electric blue theme

## Decision: Text Selection Detection
**Rationale**: Using the standard DOM Selection API with window.getSelection() and selectionchange event to detect when users highlight text on the page. This provides reliable cross-browser support and doesn't interfere with normal page functionality.

**Alternatives considered**:
- Mouse event tracking - rejected as it's more complex and less reliable than native selection API
- MutationObserver approach - rejected as it would be overkill for simple text selection detection

## Decision: Tooltip Implementation
**Rationale**: Creating a floating tooltip component that appears near selected text with a "Ask AI" button. The tooltip will be positioned using getBoundingClientRect() of the selection to ensure proper placement relative to the selected content.

**Alternatives considered**:
- Modifying the main chat window - rejected as it would interrupt user flow
- Auto-populating the chat input without user action - rejected as it would be intrusive

## Decision: API Integration
**Rationale**: Direct fetch API calls to `http://localhost:8000/chat` endpoint following the existing backend API design. Enhanced to include context from selected text when available. This maintains consistency with the established architecture.

**Alternatives considered**:
- WebSocket connection - rejected as overkill for simple Q&A functionality
- GraphQL - rejected due to backend simplicity requirements

## Decision: Contextual Query Enhancement
**Rationale**: When text is selected, the query sent to the backend will include both the selected text as context and the user's question, formatted as "Context: [selected text] Question: [user question]". This provides the AI with proper context for more accurate responses.

**Alternatives considered**:
- Separate context field in API - rejected as it would require backend changes
- Preprocessing on backend - rejected as client-side context inclusion is simpler

## Decision: Markdown Rendering
**Rationale**: Using react-markdown with remark-gfm plugin for proper rendering of code blocks and other markdown elements from AI responses. This handles the technical content requirements specified in the feature.

**Alternatives considered**:
- Custom markdown parser - rejected due to security concerns and complexity
- Backend-side rendering - rejected as it would increase API response time

## Decision: Layout Integration
**Rationale**: Using Docusaurus theme customization to wrap the layout with the ChatBot component, ensuring it persists across all handbook chapters while following Docusaurus best practices. The selection detection will be global across the entire page.

**Alternatives considered**:
- Page-level inclusion - rejected as it wouldn't persist across navigation
- Swizzling the Layout component - rejected as it's harder to maintain across Docusaurus updates

## Decision: Styling Approach
**Rationale**: CSS Modules for component-specific styling with glassmorphism effect and electric blue theme. Additional styling for the selection tooltip will follow the same design language for consistency.

**Alternatives considered**:
- Global CSS - rejected due to potential conflicts with existing styles
- Styled-components - rejected for simplicity and to avoid additional dependencies