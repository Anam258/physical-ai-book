# Physical AI Tutor Chat Widget

The Physical AI Tutor Chat Widget is a React component that provides AI-powered assistance to users reading the Physical AI & Humanoid Robotics handbook. The widget features a floating button that opens a chat interface, with advanced "Ask about selection" functionality that detects when users highlight text and provides contextual assistance.

## Features

- Floating AI tutor button with robot icon
- Glassmorphism chat window with electric blue theming
- Text selection detection with contextual "Ask AI" tooltip
- Markdown rendering for AI responses (including code blocks)
- Session persistence across page navigation
- Mobile-responsive design
- Accessibility features for screen readers

## Installation

1. Install the required dependencies:
```bash
npm install react-markdown remark-gfm react-icons
```

2. The component is located at `src/components/ChatBot/` and can be imported directly.

## Usage

```jsx
import ChatBot from './components/ChatBot';

function App() {
  return (
    <div>
      {/* Your app content */}
      <ChatBot
        apiEndpoint="http://localhost:8000/chat"
        themeColor="#00f7ff"
        initiallyOpen={false}
        enableTextSelection={true}
        tooltipDelay={500}
      />
    </div>
  );
}
```

## Props

| Prop | Type | Default | Description |
|------|------|---------|-------------|
| `apiEndpoint` | string | `'http://localhost:8000/chat'` | The backend API endpoint for chat requests |
| `themeColor` | string | `'#00f7ff'` | The primary theme color (electric blue) |
| `initiallyOpen` | boolean | `false` | Whether the chat window starts open |
| `enableTextSelection` | boolean | `true` | Whether to enable the text selection feature |
| `tooltipDelay` | number | `500` | Delay in milliseconds before showing the selection tooltip |

## Functionality

### Core Chat
- Click the floating button to open the chat window
- Type questions in the input field and press Enter or click Send
- AI responses are displayed with proper markdown formatting
- "Typing..." indicator shows when waiting for a response

### Text Selection Feature
- Select/highlight any text on the page
- A "Ask AI" tooltip will appear near the selection (with configurable delay)
- Click the tooltip to open the chat and use the selected text as context
- When submitting, the selected text is included as context for more accurate responses

### Persistence
- Chat messages are saved to localStorage and persist across page navigation
- Chat window open/closed state is also persisted

## Accessibility

- Keyboard navigation support (Enter/Space to activate buttons)
- ARIA labels and roles for screen readers
- Proper focus management
- High contrast colors for visibility

## Browser Support

- Chrome, Firefox, Safari, Edge (latest versions)
- Mobile browsers with touch support

## API Integration

The component sends POST requests to the specified `apiEndpoint` with the following structure:
```json
{
  "message": "Context: [selected text]\n\nQuestion: [user question]",
  "context": "[selected text or null]"
}
```

The expected response format is:
```json
{
  "response": "[AI response with markdown formatting]",
  "timestamp": "[ISO date string]"
}
```

## Styling

The component uses CSS Modules with glassmorphism effects and electric blue theming. Customization is possible through CSS variables and the `themeColor` prop.