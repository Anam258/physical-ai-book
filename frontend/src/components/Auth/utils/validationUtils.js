/**
 * Form Validation Utilities
 * Implements validation functions for email, password, and dropdown validation
 */

// Valid software background options
const VALID_SOFTWARE_BACKGROUNDS = ['Python', 'JS', 'C++', 'None'];

// Valid hardware background options
const VALID_HARDWARE_BACKGROUNDS = ['Arduino', 'ROS', 'Isaac Sim', 'None'];

/**
 * Validates email format using a comprehensive regex
 * @param {string} email - Email address to validate
 * @returns {boolean} - True if email format is valid, false otherwise
 */
export const validateEmail = (email) => {
  const emailRegex = /^[a-zA-Z0-9.!#$%&'*+/=?^_`{|}~-]+@[a-zA-Z0-9](?:[a-zA-Z0-9-]{0,61}[a-zA-Z0-9])?(?:\.[a-zA-Z0-9](?:[a-zA-Z0-9-]{0,61}[a-zA-Z0-9])?)*$/;
  return emailRegex.test(email);
};

/**
 * Validates password strength
 * @param {string} password - Password to validate
 * @returns {Object} - Object with isValid boolean and error message
 */
export const validatePassword = (password) => {
  if (password.length < 8) {
    return {
      isValid: false,
      message: 'Password must be at least 8 characters long'
    };
  }

  if (!/(?=.*[a-z])/.test(password)) {
    return {
      isValid: false,
      message: 'Password must contain at least one lowercase letter'
    };
  }

  if (!/(?=.*[A-Z])/.test(password)) {
    return {
      isValid: false,
      message: 'Password must contain at least one uppercase letter'
    };
  }

  if (!/(?=.*\d)/.test(password)) {
    return {
      isValid: false,
      message: 'Password must contain at least one number'
    };
  }

  if (!/(?=.*[@$!%*?&])/.test(password)) {
    return {
      isValid: false,
      message: 'Password must contain at least one special character (@$!%*?&)'
    };
  }

  return {
    isValid: true,
    message: ''
  };
};

/**
 * Validates name (minimum 2 characters)
 * @param {string} name - Name to validate
 * @returns {Object} - Object with isValid boolean and error message
 */
export const validateName = (name) => {
  if (!name || name.trim().length < 2) {
    return {
      isValid: false,
      message: 'Name must be at least 2 characters long'
    };
  }

  if (name.trim().length > 50) {
    return {
      isValid: false,
      message: 'Name must be less than 50 characters'
    };
  }

  return {
    isValid: true,
    message: ''
  };
};

/**
 * Validates software background selection
 * @param {string} softwareBackground - Software background to validate
 * @returns {Object} - Object with isValid boolean and error message
 */
export const validateSoftwareBackground = (softwareBackground) => {
  if (!VALID_SOFTWARE_BACKGROUNDS.includes(softwareBackground)) {
    return {
      isValid: false,
      message: `Software background must be one of: ${VALID_SOFTWARE_BACKGROUNDS.join(', ')}`
    };
  }

  return {
    isValid: true,
    message: ''
  };
};

/**
 * Validates hardware background selection
 * @param {string} hardwareBackground - Hardware background to validate
 * @returns {Object} - Object with isValid boolean and error message
 */
export const validateHardwareBackground = (hardwareBackground) => {
  if (!VALID_HARDWARE_BACKGROUNDS.includes(hardwareBackground)) {
    return {
      isValid: false,
      message: `Hardware background must be one of: ${VALID_HARDWARE_BACKGROUNDS.join(', ')}`
    };
  }

  return {
    isValid: true,
    message: ''
  };
};

/**
 * Comprehensive form validation for sign-up step 1 (credentials)
 * @param {Object} formData - Form data containing name, email, password
 * @returns {Object} - Object with validation results for each field
 */
export const validateSignUpStep1 = (formData) => {
  const { name, email, password } = formData;

  return {
    name: validateName(name),
    email: validateEmail(email) ? { isValid: true, message: '' } : { isValid: false, message: 'Please enter a valid email address' },
    password: validatePassword(password)
  };
};

/**
 * Comprehensive form validation for sign-up step 2 (personalization)
 * @param {Object} formData - Form data containing softwareBackground, hardwareBackground
 * @returns {Object} - Object with validation results for each field
 */
export const validateSignUpStep2 = (formData) => {
  const { softwareBackground, hardwareBackground } = formData;

  return {
    softwareBackground: validateSoftwareBackground(softwareBackground),
    hardwareBackground: validateHardwareBackground(hardwareBackground)
  };
};

/**
 * Form validation for sign-in
 * @param {Object} formData - Form data containing email, password
 * @returns {Object} - Object with validation results for each field
 */
export const validateSignIn = (formData) => {
  const { email, password } = formData;

  return {
    email: validateEmail(email) ? { isValid: true, message: '' } : { isValid: false, message: 'Please enter a valid email address' },
    password: password ? { isValid: true, message: '' } : { isValid: false, message: 'Password is required' }
  };
};

/**
 * Checks if all validations in an object are valid
 * @param {Object} validationResults - Object containing validation results
 * @returns {boolean} - True if all validations pass, false otherwise
 */
export const areAllValidationsPassed = (validationResults) => {
  return Object.values(validationResults).every(result => result.isValid);
};