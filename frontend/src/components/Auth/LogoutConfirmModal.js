import React, { useEffect } from 'react';
import ReactDOM from 'react-dom';
import './LogoutConfirmModal.css';

const LogoutConfirmModal = ({ isOpen, onClose, onConfirm }) => {
  // Handle Escape key press
  useEffect(() => {
    if (!isOpen) return;

    const handleEscape = (e) => {
      if (e.key === 'Escape') {
        onClose();
      }
    };

    window.addEventListener('keydown', handleEscape);

    // Prevent body scroll when modal is open
    document.body.style.overflow = 'hidden';

    // Cleanup function
    return () => {
      window.removeEventListener('keydown', handleEscape);
      document.body.style.overflow = 'unset';
    };
  }, [isOpen, onClose]);

  // Don't render if not open
  if (!isOpen) return null;

  // Render modal using React Portal
  return ReactDOM.createPortal(
    <div
      className="logout-modal-overlay"
      onClick={onClose}
    >
      <div
        className="logout-modal-container"
        onClick={(e) => e.stopPropagation()}
      >
        <p className="logout-modal-text">
          Are you sure you want to log out?
        </p>
        <div className="logout-modal-buttons">
          <button
            className="logout-cancel-btn"
            onClick={onClose}
          >
            Cancel
          </button>
          <button
            className="logout-confirm-btn"
            onClick={onConfirm}
          >
            Logout
          </button>
        </div>
      </div>
    </div>,
    document.body
  );
};

export default LogoutConfirmModal;
