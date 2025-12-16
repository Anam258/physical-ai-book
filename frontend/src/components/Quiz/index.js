import React, { useState } from 'react';
import styles from './styles.module.css';
import { quizData } from './quizData';

const Quiz = ({ topic }) => {
  const questions = quizData[topic] || [];
  const [currentQuestionIndex, setCurrentQuestionIndex] = useState(0);
  const [selectedOption, setSelectedOption] = useState(null);
  const [isAnswered, setIsAnswered] = useState(false);
  const [isCorrect, setIsCorrect] = useState(false);
  const [showExplanation, setShowExplanation] = useState(false);

  const currentQuestion = questions[currentQuestionIndex];

  const handleOptionSelect = (optionIndex) => {
    if (!currentQuestion) return; // Prevent selections if no question

    setSelectedOption(optionIndex);
    const correct = optionIndex === currentQuestion.correctAnswer;
    setIsCorrect(correct);

    // For both correct and incorrect answers, mark as answered
    // but only proceed to next question for correct answers
    setIsAnswered(true);

    if (correct) {
      // Show success message briefly before moving to next question
      setTimeout(() => {
        if (currentQuestionIndex < questions.length - 1) {
          setCurrentQuestionIndex(currentQuestionIndex + 1);
          resetQuestionState();
        }
      }, 1500);
    }
    // For incorrect answers, keep state as answered but don't move forward
    // This allows the user to see which option was wrong and try again
    // but in our case, we want to show the explanation immediately for incorrect answers too
    // So we won't reset for incorrect answers, just let the user see the feedback
  };

  const resetQuestionState = () => {
    setSelectedOption(null);
    setIsAnswered(false);
    setIsCorrect(false);
    setShowExplanation(false);
  };

  const handleNextQuestion = () => {
    if (currentQuestionIndex < questions.length - 1) {
      setCurrentQuestionIndex(currentQuestionIndex + 1);
      resetQuestionState();
    }
  };

  const handleRestartQuiz = () => {
    setCurrentQuestionIndex(0);
    resetQuestionState();
  };

  // If no questions are available for this topic, show fallback
  if (!questions || questions.length === 0) {
    return (
      <div className={styles.quizContainer}>
        <div className={styles.fallbackCard}>
          <p className={styles.fallbackMessage}>Quiz coming soon!</p>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.quizContainer}>
      <div className={styles.questionCard}>
        <h3 className={styles.questionNumber}>
          Question {currentQuestionIndex + 1} of {questions.length}
        </h3>
        <h2 className={styles.questionText}>{currentQuestion.question}</h2>

        <div className={styles.optionsContainer}>
          {currentQuestion.options.map((option, index) => (
            <button
              key={index}
              className={`${styles.optionButton} ${
                selectedOption === index
                  ? isAnswered
                    ? isCorrect
                      ? styles.correctOption
                      : styles.incorrectOption
                    : styles.selectedOption
                  : ''
              } ${isAnswered && index === currentQuestion.correctAnswer ? styles.correctOption : ''}`}
              onClick={() => handleOptionSelect(index)}
              disabled={isAnswered && isCorrect} // Only disable options when the correct answer has been selected
            >
              {option}
            </button>
          ))}
        </div>

        {isAnswered && (
          <div className={styles.feedbackContainer}>
            {isCorrect ? (
              <div className={styles.successMessage}>
                <div className={styles.confetti}>
                  🎉 Correct! Good job.
                </div>
              </div>
            ) : (
              <div className={styles.errorMessage}>
                <p>❌ Incorrect, try another option.</p>
              </div>
            )}

            {/* Show explanation for both correct and incorrect answers, but only after user has seen the feedback */}
            {(isCorrect || showExplanation) && (
              <div className={styles.explanation}>
                <strong>Explanation:</strong> {currentQuestion.explanation}
              </div>
            )}

            {/* Show "Show Explanation" button only for incorrect answers if explanation not shown yet */}
            {!isCorrect && !showExplanation && (
              <button
                className={styles.showExplanationButton}
                onClick={() => setShowExplanation(true)}
              >
                Show Explanation
              </button>
            )}

            {/* Show "Next Question" button only for correct answers and if there are more questions */}
            {isCorrect && currentQuestionIndex < questions.length - 1 && (
              <button
                className={styles.nextButton}
                onClick={handleNextQuestion}
              >
                Next Question →
              </button>
            )}

            {/* Show "Restart Quiz" button for the last question when answered correctly */}
            {isCorrect && currentQuestionIndex === questions.length - 1 && (
              <button
                className={styles.restartButton}
                onClick={handleRestartQuiz}
              >
                Restart Quiz
              </button>
            )}
          </div>
        )}

        {/* Show restart button if quiz is completed */}
        {currentQuestionIndex === questions.length - 1 && isAnswered && isCorrect && (
          <div className={styles.completionSection}>
            <h3>🎉 Quiz Completed! 🎉</h3>
            <p>You've completed all questions successfully!</p>
          </div>
        )}
      </div>
    </div>
  );
};

export default Quiz;