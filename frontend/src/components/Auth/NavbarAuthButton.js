import React, { useState } from 'react';
import { useAuth } from './AuthProvider';
import AuthModal from './AuthModal';
import LogoutConfirmModal from './LogoutConfirmModal';

const NavbarAuthButton = () => {
  const { user, isAuthenticated, logout } = useAuth();
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [isLogoutModalOpen, setIsLogoutModalOpen] = useState(false);

  const handleOpenModal = () => {
    setIsModalOpen(true);
  };

  const handleCloseModal = () => {
    setIsModalOpen(false);
  };

  const handleLogout = () => {
    logout();
  };

  const handleUserBadgeClick = () => {
    setIsLogoutModalOpen(true);
  };

  const handleLogoutConfirm = () => {
    // Clear ALL localStorage data
    localStorage.clear();

    // Console log for debugging
    console.log('User logged out');

    // Hard navigate to homepage
    window.location.href = '/physical-ai-book/';
  };

  const handleLogoutCancel = () => {
    setIsLogoutModalOpen(false);
  };

  if (isAuthenticated && user) {
    // Show user profile when logged in
    return (
      <>
        <div className="navbar-auth-profile">
          <button
            className="navbar-auth-button authenticated"
            onClick={handleUserBadgeClick}
            title={`Signed in as ${user.email || 'User'}`}
            style={{ cursor: 'pointer' }}
          >
            <span className="user-initial">
              {user.email ? user.email.charAt(0).toUpperCase() : 'U'}
            </span>
            <span className="user-name">{user.email?.split('@')[0] || 'User'}</span>
          </button>
        </div>
        <LogoutConfirmModal
          isOpen={isLogoutModalOpen}
          onClose={handleLogoutCancel}
          onConfirm={handleLogoutConfirm}
        />
      </>
    );
  }

  // Show login/signup button when not authenticated
  return (
    <>
      <button
        className="navbar-auth-button"
        onClick={handleOpenModal}
      >
        Login / Sign Up
      </button>
      <AuthModal isOpen={isModalOpen} onClose={handleCloseModal} />
    </>
  );
};

export default NavbarAuthButton;