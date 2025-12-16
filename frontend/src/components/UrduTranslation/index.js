import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

// Dynamic API URL based on environment
const isLocal = typeof window !== 'undefined' && (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1');
const API_BASE = isLocal
  ? 'http://127.0.0.1:8000'
  : 'https://physical-ai-backend-j8q9.onrender.com';

const UrduTranslation = () => {
  const [selectedText, setSelectedText] = useState('');
  const [translation, setTranslation] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const [showButton, setShowButton] = useState(false);
  const [buttonPosition, setButtonPosition] = useState({ top: 0, left: 0 });

  // Text selection handler
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      if (text.length > 0) {
        setSelectedText(text);

        // Get selection position
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        // Position button near selection
        setButtonPosition({
          top: rect.bottom + window.scrollY + 10,
          left: rect.left + window.scrollX
        });
        setShowButton(true);
      } else {
        setShowButton(false);
        setSelectedText('');
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection);
    };
  }, []);

  // Translation handler
  const handleTranslate = async () => {
    if (!selectedText) return;

    setLoading(true);
    setError('');
    setTranslation('');

    try {
      const response = await fetch(`${API_BASE}/translate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text: selectedText }),
        signal: AbortSignal.timeout(10000) // 10 second timeout
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Translation failed');
      }

      const data = await response.json();
      setTranslation(data.translation);
      setShowButton(false); // Hide button after successful translation
    } catch (err) {
      if (err.name === 'TimeoutError' || err.name === 'AbortError') {
        setError('Translation timed out. Please try again with shorter text.\nترجمہ وقت ختم ہو گیا۔ مختصر متن کے ساتھ دوبارہ کوشش کریں۔');
      } else if (err.message.includes('fetch')) {
        setError('Cannot connect to translation service. Please check your connection.\nترجمہ سروس سے رابطہ نہیں ہو سکا۔ اپنا کنکشن چیک کریں۔');
      } else {
        setError(err.message || 'Translation service unavailable. Please try again later.\nترجمہ سروس دستیاب نہیں ہے۔ بعد میں کوشش کریں۔');
      }
    } finally {
      setLoading(false);
    }
  };

  // Close translation panel
  const handleClose = () => {
    setTranslation('');
    setError('');
    setShowButton(false);
  };

  // Function to handle asking AI
  const handleAskAI = () => {
    if (selectedText) {
      window.dispatchEvent(new CustomEvent('open-chatbot', { detail: selectedText }));
      setShowButton(false); // Hide the button after triggering the event
    }
  };

  return (
    <>
      {/* Translation and Ask AI Buttons */}
      {showButton && !loading && !translation && (
        <div style={{
          position: 'absolute',
          top: `${buttonPosition.top}px`,
          left: `${buttonPosition.left}px`,
          zIndex: 9999,
          display: 'flex',
          gap: '8px'
        }}>
          <button
            className={styles.askAIButton}
            style={{
              background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
              backdropFilter: 'blur(10px)',
              border: '1px solid rgba(102, 126, 234, 0.3)',
              borderRadius: '12px',
              padding: '8px 12px',
              color: '#ffffff',
              fontSize: '14px',
              fontWeight: '600',
              cursor: 'pointer',
              transition: 'all 0.3s ease',
              boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)',
              display: 'flex',
              alignItems: 'center',
              gap: '6px',
              whiteSpace: 'nowrap'
            }}
            onClick={handleAskAI}
          >
            🤖 Ask AI
          </button>
          <button
            className={styles.translateButton}
            style={{
              background: 'rgba(15, 61, 15, 0.7)',
              backdropFilter: 'blur(10px)',
              border: '1px solid rgba(16, 185, 129, 0.3)',
              borderRadius: '12px',
              padding: '8px 12px',
              color: '#ffffff',
              fontSize: '14px',
              fontWeight: '600',
              cursor: 'pointer',
              transition: 'all 0.3s ease',
              boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)',
              display: 'flex',
              alignItems: 'center',
              gap: '6px',
              whiteSpace: 'nowrap'
            }}
            onClick={handleTranslate}
          >
            🇵🇰 اردو میں پڑھیں
          </button>
        </div>
      )}

      {/* Loading State */}
      {loading && (
        <div className={styles.loadingOverlay}>
          <div className={styles.loadingSpinner}></div>
          <p>Translating... / ترجمہ ہو رہا ہے...</p>
        </div>
      )}

      {/* Translation Result */}
      {translation && (
        <div className={styles.translationPanel}>
          <div className={styles.translationHeader}>
            <h3>Urdu Translation / اردو ترجمہ</h3>
            <button className={styles.closeButton} onClick={handleClose}>×</button>
          </div>
          <div className={styles.translationContent}>{translation}</div>
        </div>
      )}

      {/* Error Message */}
      {error && (
        <div className={styles.errorPanel}>
          <div className={styles.translationHeader}>
            <h3>Error / خرابی</h3>
            <button className={styles.closeButton} onClick={handleClose}>×</button>
          </div>
          <div className={styles.errorContent}>{error}</div>
        </div>
      )}
    </>
  );
};

export default UrduTranslation;