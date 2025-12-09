# Quickstart Guide: Physical AI Tutor Chat Widget with Text Selection Feature

## Prerequisites
- Node.js 18+ installed
- Docusaurus v3 project set up
- Backend API running at `http://localhost:8000/chat`

## Installation Steps

1. **Install Dependencies**
   ```bash
   npm install react-markdown remark-gfm react-icons
   ```

2. **Create Component Directory**
   ```bash
   mkdir -p src/components/ChatBot
   ```

3. **Add Component Files**
   - Create `src/components/ChatBot/index.js`
   - Create `src/components/ChatBot/styles.module.css`
   - Create `src/components/ChatBot/ChatWindow.js`
   - Create `src/components/ChatBot/SelectionTooltip.js`

4. **Integrate with Docusaurus Layout**
   - Add the ChatBot component to your layout wrapper or theme configuration
   - Ensure it's included in all handbook pages

## Basic Usage

The ChatBot component will automatically:
- Display a floating button at the bottom-right of the screen
- Open a glassmorphism chat window when clicked
- Detect when users select/highlight text on the page
- Show a contextual "Ask AI" tooltip near selected text
- Allow users to send messages to the backend API with or without text context
- Render AI responses with markdown support
- Persist across page navigation

## Text Selection Feature

When users select text on any handbook page:
1. A small "Ask AI" tooltip will appear near the selected text
2. Clicking the tooltip opens the chat window and pre-fills the query with the selected text as context
3. Users can also type additional questions related to the selected text
4. The query sent to the backend will include both the selected text and user's question

## Configuration

The component can be configured with props:
- `apiEndpoint`: API endpoint URL (default: `http://localhost:8000/chat`)
- `themeColor`: Primary theme color (default: electric blue)
- `initiallyOpen`: Whether the chat starts open (default: false)
- `enableTextSelection`: Whether to enable the text selection feature (default: true)
- `tooltipDelay`: Delay in milliseconds before showing the tooltip (default: 500ms)

## Testing

1. **Unit Tests**: Test individual component functions
2. **Integration Tests**: Test API communication
3. **UI Tests**: Test user interactions and state changes
4. **Selection Tests**: Test text selection detection and tooltip functionality

Run tests with:
```bash
npm test
```

## Building

To build the Docusaurus site with the new component:
```bash
npm run build
```

## Troubleshooting

- If the chat window doesn't appear, verify the component is properly integrated into the layout
- If API calls fail, check that the backend is running at the expected endpoint
- If markdown doesn't render properly, verify react-markdown dependencies are installed
- If text selection doesn't work, ensure the component is properly mounted and event listeners are active
- If tooltips don't appear, check that selection detection is enabled and properly configured