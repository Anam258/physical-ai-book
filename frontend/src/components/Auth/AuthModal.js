import React, { useState, useEffect, useRef } from 'react';
import ReactDOM from 'react-dom';
import { motion, AnimatePresence } from 'framer-motion';
import './AuthModal.css';

const AuthModal = ({ isOpen, onClose }) => {
  const [isSignUp, setIsSignUp] = useState(false); // false = sign-in, true = sign-up
  const [formData, setFormData] = useState({
    name: '',
    email: '',
    password: '',
    softwareBackground: '',
    hardwareBackground: ''
  });
  // T001: Set up validation state structure
  const [validationErrors, setValidationErrors] = useState({
    email: '',
    password: '',
    name: '',
    softwareBackground: '',
    hardwareBackground: ''
  });
  const [isFormValid, setIsFormValid] = useState(false);
  const [touchedFields, setTouchedFields] = useState({});
  // For debouncing validation
  const validationTimeouts = useRef({});
  const [errors, setErrors] = useState({});
  const [isLoading, setIsLoading] = useState(false);

  // T002: Create validation utility functions for email and password
  const validateEmail = (email) => {
    const emailRegex = /^[\w.-]+@[\w.-]+\.\w+$/;
    if (!emailRegex.test(email)) {
      return 'Please enter a valid email address';
    }
    return '';
  };

  const validatePassword = (password) => {
    if (password.length < 6) {
      return 'Password must be at least 6 characters';
    }
    return '';
  };

  const validateRequiredField = (value, fieldName) => {
    if (!value || value.trim() === '') {
      return fieldName === 'dropdown' ? 'Please select an option' : 'This field is required';
    }
    return '';
  };

  // Reset form when modal closes
  useEffect(() => {
    if (!isOpen) {
      setFormData({
        name: '',
        email: '',
        password: '',
        softwareBackground: '',
        hardwareBackground: ''
      });
      setErrors({});
      setIsSignUp(false); // Reset to sign-in mode

      // Clear any remaining validation timeouts
      Object.keys(validationTimeouts.current).forEach(key => {
        if (validationTimeouts.current[key]) {
          clearTimeout(validationTimeouts.current[key]);
          validationTimeouts.current[key] = null;
        }
      });
    }
  }, [isOpen]);

  // Handle escape key press to close modal
  useEffect(() => {
    const handleEscKey = (event) => {
      if (event.key === 'Escape' && isOpen) {
        onClose();
      }
    };

    if (isOpen) {
      document.addEventListener('keydown', handleEscKey);
    }

    return () => {
      document.removeEventListener('keydown', handleEscKey);
    };
  }, [isOpen, onClose]);

  // T035: Reset validation errors when switching between sign-up and sign-in tabs
  useEffect(() => {
    setValidationErrors({
      email: '',
      password: '',
      name: '',
      softwareBackground: '',
      hardwareBackground: ''
    });
    updateFormValidity({
      email: '',
      password: '',
      name: '',
      softwareBackground: '',
      hardwareBackground: ''
    });
  }, [isSignUp]);

  // T034: Handle browser autofill validation
  useEffect(() => {
    if (isOpen) {
      const handleAutoFill = () => {
        // Validate email field after potential autofill
        const emailError = validateEmail(formData.email);
        if (emailError !== validationErrors.email) {
          setValidationErrors(prev => ({ ...prev, email: emailError }));
          updateFormValidity({ ...validationErrors, email: emailError });
        }

        // Validate password field after potential autofill
        const passwordError = validatePassword(formData.password);
        if (passwordError !== validationErrors.password) {
          setValidationErrors(prev => ({ ...prev, password: passwordError }));
          updateFormValidity({ ...validationErrors, password: passwordError });
        }

        // Validate name field after potential autofill (sign-up only)
        if (isSignUp) {
          const nameError = validateRequiredField(formData.name, 'field');
          if (nameError !== validationErrors.name) {
            setValidationErrors(prev => ({ ...prev, name: nameError }));
            updateFormValidity({ ...validationErrors, name: nameError });
          }

          // Validate dropdowns after potential autofill
          const softwareBackgroundError = validateRequiredField(formData.softwareBackground, 'dropdown');
          if (softwareBackgroundError !== validationErrors.softwareBackground) {
            setValidationErrors(prev => ({ ...prev, softwareBackground: softwareBackgroundError }));
            updateFormValidity({ ...validationErrors, softwareBackground: softwareBackgroundError });
          }

          const hardwareBackgroundError = validateRequiredField(formData.hardwareBackground, 'dropdown');
          if (hardwareBackgroundError !== validationErrors.hardwareBackground) {
            setValidationErrors(prev => ({ ...prev, hardwareBackground: hardwareBackgroundError }));
            updateFormValidity({ ...validationErrors, hardwareBackground: hardwareBackgroundError });
          }
        }
      };

      // Check for autofill after a short delay
      const autofillTimer = setTimeout(handleAutoFill, 300);

      // Also listen for the window load event which can trigger after autofill
      window.addEventListener('load', handleAutoFill);

      return () => {
        clearTimeout(autofillTimer);
        window.removeEventListener('load', handleAutoFill);
      };
    }
  }, [isOpen, formData, isSignUp, validationErrors]);

  // T032: Handle paste events in email and password fields
  const handlePaste = (e) => {
    // Add a small delay to allow the paste operation to complete before validation
    setTimeout(() => {
      const { name, value } = e.target;

      // Validate email field after paste
      if (name === 'email') {
        const error = validateEmail(value);
        setValidationErrors(prev => ({ ...prev, email: error }));
        updateFormValidity({ ...validationErrors, email: error });
      }

      // Validate password field after paste
      if (name === 'password') {
        const error = validatePassword(value);
        setValidationErrors(prev => ({ ...prev, password: error }));
        updateFormValidity({ ...validationErrors, password: error });
      }
    }, 10); // Small delay to ensure paste has completed
  };

  // T008, T010: Update password input handler for real-time validation
  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));

    // Clear error when user starts typing
    if (errors[name]) {
      setErrors(prev => ({
        ...prev,
        [name]: ''
      }));
    }

    // T013, T014: Clear email error when user corrects email format
    if (name === 'email' && validationErrors.email) {
      setValidationErrors(prev => ({ ...prev, email: '' }));
      updateFormValidity({ ...validationErrors, email: '' });
    }

    // T010: Add onChange handler for password field validation with debouncing (T033)
    if (name === 'password') {
      // Clear the previous timeout
      if (validationTimeouts.current.password) {
        clearTimeout(validationTimeouts.current.password);
      }

      // Set a new timeout for validation
      validationTimeouts.current.password = setTimeout(() => {
        const error = validatePassword(value);
        setValidationErrors(prev => ({ ...prev, password: error }));
        updateFormValidity({ ...validationErrors, password: error });
      }, 300); // 300ms delay to balance responsiveness with performance
    }

    // T014: Clear password error when user reaches 6+ characters
    if (name === 'password' && validationErrors.password && value.length >= 6) {
      setValidationErrors(prev => ({ ...prev, password: '' }));
      updateFormValidity({ ...validationErrors, password: '' });

      // Clear the timeout since we're clearing the error anyway
      if (validationTimeouts.current.password) {
        clearTimeout(validationTimeouts.current.password);
        validationTimeouts.current.password = null;
      }
    }

    // T015: Add validation for Name field (sign-up only)
    if (name === 'name' && isSignUp) {
      const error = validateRequiredField(value, 'field');
      setValidationErrors(prev => ({ ...prev, name: error }));
      updateFormValidity({ ...validationErrors, name: error });
    }

    // T015: Clear name error when user corrects input
    if (name === 'name' && validationErrors.name) {
      setValidationErrors(prev => ({ ...prev, name: '' }));
      updateFormValidity({ ...validationErrors, name: '' });
    }

    // T016, T017: Add validation for dropdown fields (softwareBackground and hardwareBackground)
    if ((name === 'softwareBackground' || name === 'hardwareBackground') && isSignUp) {
      const error = validateRequiredField(value, 'dropdown');
      setValidationErrors(prev => ({ ...prev, [name]: error }));
      updateFormValidity({ ...validationErrors, [name]: error });
    }

    // T016, T017: Clear dropdown errors when user corrects input
    if ((name === 'softwareBackground' || name === 'hardwareBackground') && validationErrors[name]) {
      setValidationErrors(prev => ({ ...prev, [name]: '' }));
      updateFormValidity({ ...validationErrors, [name]: '' });
    }
  };

  // T007, T009: Add onBlur handler for email field validation
  const handleEmailBlur = (e) => {
    const email = e.target.value;
    const error = validateEmail(email);
    setValidationErrors(prev => ({ ...prev, email: error }));
    updateFormValidity({ ...validationErrors, email: error });
  };

  // Existing validateEmail function has been replaced with the new one defined earlier
  // that returns error messages instead of boolean values

  const validateForm = () => {
    const newErrors = {};

    // Use the new validation functions that return error messages
    const emailError = validateEmail(formData.email);
    if (emailError) {
      newErrors.email = emailError;
    }

    const passwordError = validatePassword(formData.password);
    if (passwordError) {
      newErrors.password = passwordError;
    }

    // Sign-up specific validations
    if (isSignUp) {
      const nameError = validateRequiredField(formData.name, 'field');
      if (nameError) {
        newErrors.name = nameError;
      }

      const softwareBackgroundError = validateRequiredField(formData.softwareBackground, 'dropdown');
      if (softwareBackgroundError) {
        newErrors.softwareBackground = softwareBackgroundError;
      }

      const hardwareBackgroundError = validateRequiredField(formData.hardwareBackground, 'dropdown');
      if (hardwareBackgroundError) {
        newErrors.hardwareBackground = hardwareBackgroundError;
      }
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  // T006: Add form validity state tracking function
  const updateFormValidity = (currentErrors = validationErrors) => {
    const hasErrors = Object.values(currentErrors).some(error => error !== '');
    setIsFormValid(!hasErrors);
  };

  // T004: Update form submission handler to prevent submission with validation errors
  const handleSubmit = async (e) => {
    e.preventDefault();

    // Run full validation before submission
    const allErrors = {
      email: validateEmail(formData.email),
      password: validatePassword(formData.password),
      name: isSignUp ? validateRequiredField(formData.name, 'field') : '',
      softwareBackground: isSignUp ? validateRequiredField(formData.softwareBackground, 'dropdown') : '',
      hardwareBackground: isSignUp ? validateRequiredField(formData.hardwareBackground, 'dropdown') : ''
    };

    setValidationErrors(allErrors);

    // Check if there are any errors
    const hasErrors = Object.values(allErrors).some(error => error !== '');

    if (hasErrors) {
      // T029: Trigger button shake when form submission is attempted with errors
      const submitButton = document.querySelector('button[type="submit"]');
      if (submitButton) {
        submitButton.classList.add('shake-animation');
        setTimeout(() => {
          submitButton.classList.remove('shake-animation');
        }, 500);
      }

      // T028: Show toast message "Please fix errors" when form submission fails validation
      const toast = document.createElement('div');
      toast.className = 'toast';
      toast.textContent = 'Please fix errors';
      document.body.appendChild(toast);

      // Remove toast after 3 seconds
      setTimeout(() => {
        if (toast.parentNode) {
          toast.parentNode.removeChild(toast);
        }
      }, 3000);

      // T037: Scroll to first error field when submit is attempted with errors
      const firstErrorField = Object.keys(validationErrors).find(field => validationErrors[field]) ||
                             Object.keys(errors).find(field => errors[field]);

      if (firstErrorField) {
        const element = document.querySelector(`[name="${firstErrorField}"]`);
        if (element) {
          element.focus();
          element.scrollIntoView({ behavior: 'smooth', block: 'center' });
        }
      }

      return; // Don't submit if there are errors
    }

    setIsLoading(true);

    try {
      if (isSignUp) {
        // Sign-up flow: Create new user profile
        const userData = {
          id: Date.now().toString(),
          name: formData.name,
          email: formData.email,
          password: formData.password, // Mock auth - NOT production-ready
          softwareBackground: formData.softwareBackground,
          hardwareBackground: formData.hardwareBackground,
          createdAt: new Date().toISOString(),
          lastLoginAt: new Date().toISOString()
        };

        // Save to localStorage with error handling
        localStorage.setItem('@auth/user', JSON.stringify(userData));

        // Reload page to update navbar
        window.location.reload();
      } else {
        // Sign-in flow: Validate against stored user
        const storedUserJson = localStorage.getItem('@auth/user');

        if (!storedUserJson) {
          setValidationErrors({ email: 'No account found. Please sign up first.' });
          setIsLoading(false);
          return;
        }

        const storedUser = JSON.parse(storedUserJson);

        // Validate credentials
        if (storedUser.email !== formData.email || storedUser.password !== formData.password) {
          setValidationErrors({ email: 'Invalid email or password' });
          setIsLoading(false);
          return;
        }

        // Update lastLoginAt
        storedUser.lastLoginAt = new Date().toISOString();
        localStorage.setItem('@auth/user', JSON.stringify(storedUser));

        // Reload page to update navbar
        window.location.reload();
      }
    } catch (error) {
      console.error('Authentication error:', error);

      // Handle localStorage errors (quota exceeded, security error, etc.)
      if (error.name === 'QuotaExceededError') {
        setValidationErrors({ email: 'Storage full. Please clear browser data and try again.' });
      } else if (error.name === 'SecurityError') {
        setValidationErrors({ email: 'Browser security settings prevent login. Please check your browser settings.' });
      } else {
        setValidationErrors({ email: 'An error occurred. Please try again.' });
      }

      setIsLoading(false);
    }
  };

  if (!isOpen) return null;

  return ReactDOM.createPortal(
    <AnimatePresence>
      <>
        <motion.div
          className="auth-modal-overlay"
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          exit={{ opacity: 0 }}
          transition={{ duration: 0.2 }}
          onClick={onClose}
          aria-modal="true"
          role="dialog"
        >
          <motion.div
            className="auth-modal"
            initial={{ scale: 0.9, opacity: 0 }}
            animate={{ scale: 1, opacity: 1 }}
            exit={{ scale: 0.9, opacity: 0 }}
            transition={{ duration: 0.2, ease: "easeOut" }}
            onClick={(e) => e.stopPropagation()}
            role="document"
            aria-label={isSignUp ? "Sign Up" : "Sign In"}
            tabIndex="-1"
          >
            <div className="auth-modal-header">
              <h3 style={{ color: 'white', margin: 0, fontSize: '24px', fontWeight: 600 }}>
                {isSignUp ? 'Create Account' : 'Welcome Back'}
              </h3>
              <button className="auth-close-btn" onClick={onClose} aria-label="Close modal">
                ×
              </button>
            </div>

            <form className="auth-form" onSubmit={handleSubmit} role="form">
              {/* Sign-up only: Name */}
              {isSignUp && (
                <div className="auth-input-group">
                  <label htmlFor="name">Full Name</label>
                  <input
                    type="text"
                    id="name"
                    name="name"
                    value={formData.name}
                    onChange={handleInputChange}
                    className={`${errors.name || validationErrors.name ? 'error input-error' : ''}`}
                    placeholder="Enter your full name"
                    required={isSignUp}
                    aria-invalid={!!(validationErrors.name || errors.name)}
                    aria-describedby={validationErrors.name ? "name-error" : (errors.name ? "name-error-submission" : undefined)}
                  />
                  {validationErrors.name && <span id="name-error" className="error-text" role="alert">{validationErrors.name}</span>}
                  {errors.name && !validationErrors.name && <span id="name-error-submission" className="error-message" role="alert">{errors.name}</span>}
                </div>
              )}

              {/* Always shown: Email */}
              <div className="auth-input-group">
                <label htmlFor="email">Email</label>
                <input
                  type="email"
                  id="email"
                  name="email"
                  value={formData.email}
                  onChange={handleInputChange}
                  onPaste={handlePaste}
                  onBlur={handleEmailBlur}
                  className={`${errors.email || validationErrors.email ? 'error input-error' : ''}`}
                  placeholder="Enter your email"
                  required
                  aria-invalid={!!(validationErrors.email || errors.email)}
                  aria-describedby={validationErrors.email ? "email-error" : (errors.email ? "email-error-submission" : undefined)}
                />
                {validationErrors.email && <span id="email-error" className="error-text" role="alert">{validationErrors.email}</span>}
                {errors.email && !validationErrors.email && <span id="email-error-submission" className="error-message" role="alert">{errors.email}</span>}
              </div>

              {/* Always shown: Password */}
              <div className="auth-input-group">
                <label htmlFor="password">Password</label>
                <input
                  type="password"
                  id="password"
                  name="password"
                  value={formData.password}
                  onChange={handleInputChange}
                  onPaste={handlePaste}
                  onKeyDown={(e) => {
                    // T036: Handle Enter key submission with validation
                    if (e.key === 'Enter') {
                      handleSubmit(e);
                    }
                  }}
                  className={`${errors.password || validationErrors.password ? 'error input-error' : ''}`}
                  placeholder={isSignUp ? "Create a password" : "Enter your password"}
                  required
                  aria-invalid={!!(validationErrors.password || errors.password)}
                  aria-describedby={validationErrors.password ? "password-error" : (errors.password ? "password-error-submission" : undefined)}
                />
                {validationErrors.password && <span id="password-error" className="error-text" role="alert">{validationErrors.password}</span>}
                {errors.password && !validationErrors.password && <span id="password-error-submission" className="error-message" role="alert">{errors.password}</span>}
              </div>

              {/* Sign-up only: Software Background */}
              {isSignUp && (
                <div className="auth-input-group">
                  <label htmlFor="softwareBackground">Software Background</label>
                  <select
                    id="softwareBackground"
                    name="softwareBackground"
                    value={formData.softwareBackground}
                    onChange={handleInputChange}
                    className={`${errors.softwareBackground || validationErrors.softwareBackground ? 'error input-error' : ''}`}
                    required={isSignUp}
                    aria-invalid={!!(validationErrors.softwareBackground || errors.softwareBackground)}
                    aria-describedby={validationErrors.softwareBackground ? "softwareBackground-error" : (errors.softwareBackground ? "softwareBackground-error-submission" : undefined)}
                  >
                    <option value="">Select your software background</option>
                    <option value="Python">Python</option>
                    <option value="JS">JavaScript</option>
                    <option value="C++">C++</option>
                    <option value="None">None</option>
                  </select>
                  {validationErrors.softwareBackground && <span id="softwareBackground-error" className="error-text" role="alert">{validationErrors.softwareBackground}</span>}
                  {errors.softwareBackground && !validationErrors.softwareBackground && <span id="softwareBackground-error-submission" className="error-message" role="alert">{errors.softwareBackground}</span>}
                </div>
              )}

              {/* Sign-up only: Hardware Background */}
              {isSignUp && (
                <div className="auth-input-group">
                  <label htmlFor="hardwareBackground">Hardware Background</label>
                  <select
                    id="hardwareBackground"
                    name="hardwareBackground"
                    value={formData.hardwareBackground}
                    onChange={handleInputChange}
                    className={`${errors.hardwareBackground || validationErrors.hardwareBackground ? 'error input-error' : ''}`}
                    required={isSignUp}
                    aria-invalid={!!(validationErrors.hardwareBackground || errors.hardwareBackground)}
                    aria-describedby={validationErrors.hardwareBackground ? "hardwareBackground-error" : (errors.hardwareBackground ? "hardwareBackground-error-submission" : undefined)}
                  >
                    <option value="">Select your hardware background</option>
                    <option value="Arduino">Arduino</option>
                    <option value="ROS">ROS</option>
                    <option value="Isaac Sim">Isaac Sim</option>
                    <option value="None">None</option>
                  </select>
                  {validationErrors.hardwareBackground && <span id="hardwareBackground-error" className="error-text" role="alert">{validationErrors.hardwareBackground}</span>}
                  {errors.hardwareBackground && !validationErrors.hardwareBackground && <span id="hardwareBackground-error-submission" className="error-message" role="alert">{errors.hardwareBackground}</span>}
                </div>
              )}

              <button type="submit" className="auth-submit-btn" disabled={isLoading}>
                {isLoading ? (isSignUp ? 'Creating Account...' : 'Signing In...') : (isSignUp ? 'Sign Up' : 'Sign In')}
              </button>

              {/* Toggle text */}
              <p
                className="auth-toggle-text"
                onClick={() => setIsSignUp(!isSignUp)}
                style={{ cursor: 'pointer' }}
              >
                {isSignUp
                  ? 'Already have an account? Sign in'
                  : "Don't have an account? Sign up"}
              </p>
            </form>
          </motion.div>
        </motion.div>
      </>
    </AnimatePresence>,
    document.body
  );
};

export default AuthModal;
