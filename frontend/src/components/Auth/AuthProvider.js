import React, { createContext, useContext, useState, useEffect } from 'react';
import { loadUserProfile, saveUserProfile, clearUserProfile } from './utils/storageUtils';

const AuthContext = createContext();

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  const [authStep, setAuthStep] = useState(1); // For multi-step auth flow (1: credentials, 2: personalization)

  // Check for existing session on component mount
  useEffect(() => {
    const storedUser = loadUserProfile();

    if (storedUser) {
      setUser(storedUser);
    }

    setIsLoading(false);
  }, []);

  const login = (userData) => {
    // Store complete user profile in localStorage using utility function
    saveUserProfile(userData);
    setUser(userData);
  };

  const logout = () => {
    // Clear user data from localStorage using utility function
    clearUserProfile();
    setUser(null);
  };

  // Function to update user profile after personalization step
  const updateUserProfile = (profileData) => {
    const updatedUser = {
      ...user,
      ...profileData,
      updatedAt: new Date().toISOString()
    };

    saveUserProfile(updatedUser);
    setUser(updatedUser);
  };

  const value = {
    user,
    login,
    logout,
    updateUserProfile,
    isLoading,
    isAuthenticated: !!user,
    authStep,
    setAuthStep
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};