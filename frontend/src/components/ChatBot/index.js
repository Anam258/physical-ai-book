import React, { useState, useEffect, useRef } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import styles from './styles.module.css';
import ChatWindow from './ChatWindow'; // Assuming this subcomponent exists

export default function ChatBot() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, sender: 'ai', content: 'Hi! I am your Physical AI Tutor. Ask me anything about the handbook! 🤖' }
  ]);
  const [isLoading, setIsLoading] = useState(false);
  const [nextMessageId, setNextMessageId] = useState(2);

  const API_URL = 'https://physical-ai-backend-j8q9.onrender.com/chat';

  // State for drag functionality
  const [position, setPosition] = useState(null); // Initialize to null to use CSS default position
  const [isDragging, setIsDragging] = useState(false);
  const [dragOffset, setDragOffset] = useState({ x: 0, y: 0 });
  const buttonRef = useRef(null);

  // Handle mouse down for dragging
  const handleMouseDown = (e) => {
    if (!buttonRef.current) return;

    const rect = buttonRef.current.getBoundingClientRect();
    setIsDragging(true);
    setDragOffset({
      x: e.clientX - rect.left,
      y: e.clientY - rect.top
    });
    e.preventDefault();
  };

  // Handle mouse move for dragging
  const handleMouseMove = (e) => {
    if (!isDragging) return;

    const newX = e.clientX - dragOffset.x;
    const newY = e.clientY - dragOffset.y;

    // Constrain to viewport
    const clampedX = Math.max(0, Math.min(window.innerWidth - (buttonRef.current?.offsetWidth || 60), newX));
    const clampedY = Math.max(0, Math.min(window.innerHeight - (buttonRef.current?.offsetHeight || 60), newY));

    setPosition({ x: clampedX, y: clampedY });
  };

  // Handle mouse up to stop dragging
  const handleMouseUp = () => {
    setIsDragging(false);
  };

  // Add event listeners for dragging
  useEffect(() => {
    if (isDragging) {
      window.addEventListener('mousemove', handleMouseMove);
      window.addEventListener('mouseup', handleMouseUp);

      return () => {
        window.removeEventListener('mousemove', handleMouseMove);
        window.removeEventListener('mouseup', handleMouseUp);
      };
    }
  }, [isDragging, dragOffset]);

  // Auto-scroll to bottom
  const messagesEndRef = useRef(null);
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };
  useEffect(scrollToBottom, [messages, isLoading]); // Scroll on new messages or loading state

  // Listen for 'open-chatbot' custom event to open chatbot with selected text
  useEffect(() => {
    const handleOpenChatbot = (event) => {
      setIsOpen(true);
      // Note: We can't directly set the input value here since ChatWindow handles input
      // Instead, we'll send the message directly or pass it as a prop
      // For now, we'll just open the chatbot and the ChatWindow component will handle the message
      if (event.detail) {
        // Send the selected text to the chat
        handleSend(`Explain this context: "${event.detail}"`);
      }
    };

    window.addEventListener('open-chatbot', handleOpenChatbot);

    return () => {
      window.removeEventListener('open-chatbot', handleOpenChatbot);
    };
  }, []);

  // Text selection logic removed as per requirements
  // Only the UrduTranslation component handles text selection now

  // 🚀 CORE FIX: Logic to send message
  const handleSend = async (message) => {
    if (!message.trim()) return;

    // Prepare Display Message (What user sees in the chat window)
    const displayMessage = message;

    // UI Updates
    const userMessage = { id: nextMessageId, sender: 'user', content: displayMessage };
    setMessages(prev => [...prev, userMessage]);
    setNextMessageId(prev => prev + 1);
    setIsLoading(true); 

    try {
      const response = await fetch(API_URL, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ question: message }), // Sends correct 'question' key
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
    }
  };

  // handleAskAI function removed as per requirements

  return (
    <>
      {/* Floating Button with Framer Motion animation */}
      <motion.button
        ref={buttonRef}
        className={styles.chatButton}
        style={
          position !== null
            ? {
                position: 'fixed',
                top: `${position.y}px`,
                left: `${position.x}px`,
                bottom: 'auto',
                right: 'auto',
                cursor: isDragging ? 'grabbing' : 'grab',
                zIndex: 99999
              }
            : {
                cursor: isDragging ? 'grabbing' : 'grab',
                zIndex: 99999
              }
        }
        onMouseDown={handleMouseDown}
        onClick={() => !isDragging && setIsOpen(!isOpen)}
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
            style={{
              position: 'fixed',
              right: '20px',
              bottom: '90px', // Fixed position below the button
            }}
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
              // Pass the message reference for auto-scrolling
              messagesEndRef={messagesEndRef}
            />
          </motion.div>
        )}
      </AnimatePresence>

    </>
  );
}