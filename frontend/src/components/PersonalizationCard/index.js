import React, { useState, useEffect } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import AuthModal from '../Auth/AuthModal';
import './styles.css';

// Dynamic API URL based on environment
const isLocal = typeof window !== 'undefined' && (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1');
const API_BASE = isLocal
  ? 'http://127.0.0.1:8000'
  : 'https://physical-ai-backend-j8q9.onrender.com';

const PersonalizationCard = ({ topic }) => {
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [analogy, setAnalogy] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');

  // Check if user is logged in by checking localStorage
  const isLoggedIn = () => {
    try {
      const userData = localStorage.getItem('@auth/user');
      return userData !== null && userData !== 'null' && userData !== undefined;
    } catch (e) {
      console.error('Error checking auth status:', e);
      return false;
    }
  };

  // Get user background from localStorage
  const getUserBackground = () => {
    try {
      const userData = localStorage.getItem('@auth/user');
      if (userData) {
        const user = JSON.parse(userData);
        return `${user.softwareBackground || 'N/A'} software background and ${user.hardwareBackground || 'N/A'} hardware background`;
      }
      return '';
    } catch (e) {
      console.error('Error getting user background:', e);
      return '';
    }
  };

  const handlePersonalizeClick = async () => {
    if (!isLoggedIn()) {
      // Open AuthModal if not logged in
      setIsModalOpen(true);
    } else {
      // Call API to get personalized analogy
      setIsLoading(true);
      setError('');

      try {
        const userBackground = getUserBackground();
        const response = await fetch(`${API_BASE}/personalize`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            topic: topic,
            user_background: userBackground
          })
        });

        if (!response.ok) {
          throw new Error('Failed to get personalization');
        }

        const data = await response.json();
        setAnalogy(data.analogy);
        setIsPersonalizing(true);
      } catch (err) {
        console.error('Personalization error:', err);
        setError('Failed to generate personalized explanation. Please try again.');
      } finally {
        setIsLoading(false);
      }
    };
  };

  const handleCloseModal = () => {
    setIsModalOpen(false);
  };

  return (
    <div className="personalization-container">
      <div className="personalization-button-wrapper">
        {isLoggedIn() ? (
          <motion.button
            className="personalize-btn"
            onClick={handlePersonalizeClick}
            whileHover={{ scale: 1.05 }}
            whileTap={{ scale: 0.95 }}
            disabled={isLoading}
          >
            {isLoading ? '✨ Generating...' : '✨ Explain for My Background'}
          </motion.button>
        ) : (
          <motion.button
            className="login-personalize-btn"
            onClick={() => setIsModalOpen(true)}
            whileHover={{ scale: 1.05 }}
            whileTap={{ scale: 0.95 }}
          >
            🔒 Login to Personalize
          </motion.button>
        )}
      </div>

      <AnimatePresence>
        {isPersonalizing && !isLoading && analogy && (
          <motion.div
            className="personalization-result"
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            exit={{ opacity: 0, y: -20 }}
            transition={{ duration: 0.3 }}
          >
            <div className="glass-card-dark">
              <h4 className="personalization-title">🎯 Personalized Explanation</h4>
              <p className="personalization-content">{analogy}</p>
            </div>
          </motion.div>
        )}
      </AnimatePresence>

      <AnimatePresence>
        {error && (
          <motion.div
            className="error-message"
            initial={{ opacity: 0, y: 10 }}
            animate={{ opacity: 1, y: 0 }}
            exit={{ opacity: 0 }}
          >
            <div className="glass-card-error">
              <p>{error}</p>
            </div>
          </motion.div>
        )}
      </AnimatePresence>

      {/* Auth Modal */}
      <AuthModal isOpen={isModalOpen} onClose={handleCloseModal} />
    </div>
  );
};

export default PersonalizationCard;