import React, { useState, useEffect, useRef } from 'react';
import { motion } from 'framer-motion';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import { FaPaperPlane, FaTimes } from 'react-icons/fa';
import { Prism as SyntaxHighlighter } from 'react-syntax-highlighter';
import { dracula } from 'react-syntax-highlighter/dist/esm/styles/prism';
import styles from './styles.module.css';

const ChatWindow = ({ messages, isTyping, onSendMessage, onClose, themeColor, selectedText, onSelectedTextHandled }) => {
  const [inputValue, setInputValue] = useState('');
  const messagesEndRef = useRef(null);
  const messagesContainerRef = useRef(null);

  // Auto-scroll to bottom whenever messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages, isTyping]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    if (inputValue.trim() || selectedText) {
      // If there's selected text, create a context message
      let messageToSend = inputValue.trim();
      if (selectedText) {
        messageToSend = `Context: "${selectedText}"\nQuestion: ${messageToSend || "Explain this."}`;
        // Clear the selected text after using it
        if (onSelectedTextHandled) {
          onSelectedTextHandled();
        }
      }
      onSendMessage(messageToSend);
      setInputValue('');
    }
  };

  const components = {
  code({ node, inline, className, children, ...props }) {
    const match = /language-(\w+)/.exec(className || '');
    return !inline && match ? (
      <SyntaxHighlighter
        style={dracula}
        language={match[1]}
        PreTag="div"
        {...props}
      >
        {String(children).replace(/\n$/, '')}
      </SyntaxHighlighter>
    ) : (
      <code className={className} {...props}>
        {children}
      </code>
    );
  }
};

return (
    <div className={styles.chatWindow} style={{ '--theme-color': themeColor }}>
      {/* Chat Header */}
      <div className={styles.chatHeader}>
        <div className={styles.chatTitle}>AI Tutor</div>
        <button className={styles.closeButton} onClick={onClose} aria-label="Close chat">
          <FaTimes />
        </button>
      </div>

      {/* Messages Container */}
      <div ref={messagesContainerRef} className={styles.messagesContainer}>
        {messages.map((message) => (
          <div
            key={message.id}
            className={`${styles.message} ${styles[message.sender]} ${
              message.status === 'error' ? styles.error : ''
            }`}
          >
            {message.sender === 'ai' ? (
              <div className={styles.aiMessageContent}>
                <ReactMarkdown
                  remarkPlugins={[remarkGfm]}
                  className={styles.markdownContent}
                  components={components}
                >
                  {message.content}
                </ReactMarkdown>
              </div>
            ) : (
              <div className={styles.userMessageContent}>
                {message.content}
                {message.status === 'error' && (
                  <span className={styles.errorMessage}>Failed to send</span>
                )}
              </div>
            )}
          </div>
        ))}

        {isTyping && (
          <div className={styles.message + ' ' + styles.ai}>
            <div className={styles.typingIndicator}>
              <motion.div
                className={styles.typingDot}
                animate={{ y: [0, -5, 0] }}
                transition={{
                  duration: 0.6,
                  repeat: Infinity,
                  repeatType: "loop",
                  delay: 0
                }}
              />
              <motion.div
                className={styles.typingDot}
                animate={{ y: [0, -5, 0] }}
                transition={{
                  duration: 0.6,
                  repeat: Infinity,
                  repeatType: "loop",
                  delay: 0.2
                }}
              />
              <motion.div
                className={styles.typingDot}
                animate={{ y: [0, -5, 0] }}
                transition={{
                  duration: 0.6,
                  repeat: Infinity,
                  repeatType: "loop",
                  delay: 0.4
                }}
              />
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {/* Selected Text Indicator */}
      {selectedText && (
        <div className={styles.selectedTextIndicator}>
          <strong>Context:</strong> "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"
          <button
            className={styles.clearContextButton}
            onClick={() => onSelectedTextHandled && onSelectedTextHandled()}
            aria-label="Clear context"
          >
            ✕
          </button>
        </div>
      )}

      {/* Input Area */}
      <form onSubmit={handleSubmit} className={styles.inputForm}>
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder={selectedText ? "Ask about the selected text..." : "Ask a question about the handbook..."}
          className={styles.messageInput}
          autoFocus // Focus the input when chat opens
        />
        <button type="submit" className={styles.sendButton} disabled={!inputValue.trim() && !selectedText}>
          <FaPaperPlane />
        </button>
      </form>
    </div>
  );
};

export default ChatWindow;