import React, { useState, useEffect, useRef } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import styles from './styles.module.css';
import ChatWindow from './ChatWindow'; // Assuming Claude created this subcomponent
import SelectionTooltip from './SelectionTooltip'; // Assuming Claude created this subcomponent

export default function ChatBot() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, sender: 'ai', content: 'Hi! I am your Physical AI Tutor. Ask me anything about the handbook! 🤖' }
  ]);
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [tooltip, setTooltip] = useState({ show: false, x: 0, y: 0 });
  const [nextMessageId, setNextMessageId] = useState(2);
  
  // 🔗 RENDER DEPLOYMENT FIX: REPLACE THIS WITH YOUR PUBLIC RENDER URL
  const PUBLIC_API_URL = 'YOUR_RENDER_URL_HERE/chat';
  // Example: 'https://physical-ai-backend-j8q9.onrender.com/chat'

  // Auto-scroll to bottom
  const messagesEndRef = useRef(null);
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };
  useEffect(scrollToBottom, [messages, isLoading]); // Scroll on new messages or loading state

  // Handle Text Selection (Tooltip Logic)
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      if (text.length > 5) {
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();
        setTooltip({
          show: true,
          x: rect.left + rect.width / 2,
          y: rect.top - 10
        });
        setSelectedText(text);
      } else {
        setTooltip({ show: false, x: 0, y: 0 });
        setSelectedText('');
      }
    };

    const hideTooltip = () => {
      setTooltip({ show: false, x: 0, y: 0 });
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('mousedown', hideTooltip);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('mousedown', hideTooltip);
    };
  }, []);

  // 🚀 CORE FIX: Logic to send message, consuming selected text as context
  const handleSend = async (message) => {
    if (!message.trim() && !selectedText) return;

    // 1. Construct the Final Query for the Backend (422 FIX)
    const context = selectedText;
    const finalQuery = context
      ? `Context: "${context}"\nQuestion: ${message.trim() || "Explain this."}`
      : message;

    // 2. Prepare Display Message (What user sees in the chat window)
    const displayMessage = context 
      ? `(Selected: "${context.substring(0, 30)}...") ${message.trim() || 'Explain this section.'}` 
      : message;
    
    // UI Updates
    const userMessage = { id: nextMessageId, sender: 'user', content: displayMessage };
    setMessages(prev => [...prev, userMessage]);
    setNextMessageId(prev => prev + 1);
    setIsLoading(true);
    setSelectedText(''); 

    try {
      // 3. Fetch API: Use Public URL
      const response = await fetch(PUBLIC_API_URL, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ question: finalQuery }), // Sends correct 'question' key
      });

      if (!response.ok) throw new Error('Network response was not ok');

      const data = await response.json();
      const aiMessage = {
        id: nextMessageId + 1,
        sender: 'ai',
        content: data.answer
      };

      setMessages(prev => [...prev, aiMessage]);
      setNextMessageId(prev => prev + 1);
    } catch (error) {
      const errorMessage = {
        id: nextMessageId + 1,
        sender: 'ai',
        content: '❌ Error: Could not connect to AI Tutor. Check API URL/Server status.',
        status: 'error'
      };

      setMessages(prev => [...prev, errorMessage]);
      console.error("Chat Error:", error);
    } finally {
      setIsLoading(false);
      setSelectedText(''); // Clear context after transmission
    }
  };

  // 🚀 FIX: Clicking 'Ask AI' button simply opens the chat and sets the focus
  const handleAskAI = () => {
    setTooltip({ show: false, x: 0, y: 0 });
    setIsOpen(true);
    // The ChatWindow component should handle setting the input value based on selectedText if needed,
    // but the main logic is now to prepare the context state (which handleSend consumes).
  };

  return (
    <>
      {/* Floating Button with Framer Motion animation */}
      <motion.button
        className={styles.chatButton}
        onClick={() => setIsOpen(!isOpen)}
        // Pulse animation from Framer Motion
        animate={{ scale: [1, 1.05, 1] }}
        transition={{ duration: 2, repeat: Infinity, repeatType: "reverse" }}
      >
        {isOpen ? '✖' : '🤖'}
      </motion.button>

      {/* Chat Window with Framer Motion animation */}
      <AnimatePresence>
        {isOpen && (
          <motion.div
            className={styles.chatWindowContainer}
            initial={{ opacity: 0, scale: 0.8, y: 20 }}
            animate={{ opacity: 1, scale: 1, y: 0 }}
            exit={{ opacity: 0, scale: 0.8, y: 20 }}
            transition={{ type: "spring", damping: 20, stiffness: 300 }}
          >
            <ChatWindow
              messages={messages}
              isTyping={isLoading}
              onSendMessage={handleSend}
              onClose={() => setIsOpen(false)}
              selectedText={selectedText}
              // Pass the message reference for auto-scrolling
              messagesEndRef={messagesEndRef} 
            />
          </motion.div>
        )}
      </AnimatePresence>

      {/* Selection Tooltip */}
      <SelectionTooltip
        show={tooltip.show}
        position={{ x: tooltip.x, y: tooltip.y }}
        onAskAI={handleAskAI}
      />
    </>
  );
}