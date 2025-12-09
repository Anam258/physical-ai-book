# Data Model: Physical AI Tutor Chat Widget with Text Selection Feature

## Entities

### ChatMessage
- **id**: string (unique identifier for the message)
- **content**: string (the message content, can contain markdown)
- **sender**: enum ['user', 'ai'] (who sent the message)
- **timestamp**: datetime (when the message was sent)
- **status**: enum ['sent', 'pending', 'error'] (for user messages only)
- **context**: string (optional context from selected text on the page)

### ChatSession
- **id**: string (unique identifier for the session)
- **messages**: array of ChatMessage (conversation history)
- **createdAt**: datetime (when the session was created)
- **lastActiveAt**: datetime (when the last message was sent)

### ChatWidgetState
- **isOpen**: boolean (whether the chat window is open)
- **isTyping**: boolean (whether the AI is currently responding)
- **session**: ChatSession (current conversation session)
- **isVisible**: boolean (whether the widget is visible on screen)
- **selectedText**: string (currently highlighted text on the page)
- **showTooltip**: boolean (whether to show the selection tooltip)
- **tooltipPosition**: object {x: number, y: number} (coordinates for tooltip positioning)

### SelectionContext
- **selectedText**: string (the text that was selected by the user)
- **selectionRect**: object {top: number, left: number, width: number, height: number} (bounding rectangle of selection)
- **timestamp**: datetime (when the selection was made)
- **pageContext**: string (URL or page identifier where selection was made)

## State Transitions

### ChatWidgetState Transitions:
1. **Initial** → **Visible**: When component mounts
2. **Visible** → **SelectionDetected**: When user selects text on the page
3. **SelectionDetected** → **TooltipShown**: When tooltip is positioned and displayed
4. **TooltipShown** → **Open**: When "Ask AI" button is clicked from tooltip
5. **Visible** → **Open**: When floating button is clicked
6. **Open** → **Typing**: When user submits a message (with or without selected text context)
7. **Typing** → **Open**: When AI response is received
8. **Open** → **Visible**: When user closes the chat window

### ChatMessage Status Transitions:
1. **pending**: When user sends a message
2. **sent**: When API call succeeds
3. **error**: When API call fails

## Validation Rules

### ChatMessage Validation:
- Content must not be empty or exceed 2000 characters
- When context is present, total length (content + context) should not exceed 4000 characters
- Sender must be either 'user' or 'ai'
- Timestamp must be within reasonable bounds

### ChatSession Validation:
- Must have at least one message to be considered active
- Message count should not exceed 100 messages per session
- Session must be created before any messages are added

### SelectionContext Validation:
- Selected text must be between 1 and 1000 characters
- Selection must have valid bounding rectangle coordinates
- Page context must be a valid URL or page identifier