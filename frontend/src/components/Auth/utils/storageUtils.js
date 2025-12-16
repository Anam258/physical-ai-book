/**
 * User Profile Persistence Utilities
 * Implements localStorage-based user profile persistence for authentication system
 */

/**
 * UserProfile Interface
 * @typedef {Object} UserProfile
 * @property {string} id - Unique identifier (generated or from auth system)
 * @property {string} name - User's full name
 * @property {string} email - User's email address (validated format)
 * @property {string} softwareBackground - User's software experience: "Python", "JS", "C++", or "None"
 * @property {string} hardwareBackground - User's hardware experience: "Arduino", "ROS", "Isaac Sim", or "None"
 * @property {string} createdAt - Timestamp when user was created
 * @property {string} lastLoginAt - Timestamp when user last logged in
 * @property {string} [updatedAt] - Timestamp when profile was last updated
 */

/**
 * Saves user profile to localStorage
 * @param {UserProfile} userProfile - The user profile to save
 * @returns {boolean} - True if successful, false otherwise
 */
export const saveUserProfile = (userProfile) => {
  try {
    // Add timestamp if not present
    const profileToSave = {
      ...userProfile,
      lastLoginAt: userProfile.lastLoginAt || new Date().toISOString(),
      updatedAt: new Date().toISOString()
    };

    localStorage.setItem('@auth/user', JSON.stringify(profileToSave));
    return true;
  } catch (error) {
    console.error('Error saving user profile to localStorage:', error);
    return false;
  }
};

/**
 * Loads user profile from localStorage
 * @returns {UserProfile|null} - The user profile if found, null otherwise
 */
export const loadUserProfile = () => {
  try {
    const stored = localStorage.getItem('@auth/user');
    if (!stored) {
      return null;
    }

    const userProfile = JSON.parse(stored);
    return userProfile;
  } catch (error) {
    console.error('Error loading user profile from localStorage:', error);
    return null;
  }
};

/**
 * Clears user profile from localStorage (logout functionality)
 * @returns {boolean} - True if successful, false otherwise
 */
export const clearUserProfile = () => {
  try {
    localStorage.removeItem('@auth/user');
    return true;
  } catch (error) {
    console.error('Error clearing user profile from localStorage:', error);
    return false;
  }
};

/**
 * Checks if user is authenticated (profile exists in localStorage)
 * @returns {boolean} - True if user is authenticated, false otherwise
 */
export const isAuthenticated = () => {
  const userProfile = loadUserProfile();
  return userProfile !== null;
};