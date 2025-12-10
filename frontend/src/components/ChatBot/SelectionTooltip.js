import React from 'react';
import styles from './styles.module.css';

const SelectionTooltip = ({
  show,
  position,
  onAskAI,
  themeColor = '#00f7ff'
}) => {
  if (!show) {
    return null;
  }

  return (
    <div
      className={styles.selectionTooltip}
      style={{
        left: `${position.x}px`,
        top: `${position.y}px`,
        '--theme-color': themeColor
      }}
      onClick={onAskAI}
    >
      Ask AI
    </div>
  );
};

export default SelectionTooltip;