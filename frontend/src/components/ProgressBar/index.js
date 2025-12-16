import React, { useState, useEffect } from 'react';

const ProgressBar = () => {
  const [scrollWidth, setScrollWidth] = useState(0);

  useEffect(() => {
    const handleScroll = () => {
      // Calculate scroll percentage
      const scrollTop = document.documentElement.scrollTop || document.body.scrollTop;
      const scrollHeight = document.documentElement.scrollHeight - document.documentElement.clientHeight;
      const scrolled = (scrollTop / scrollHeight) * 100;

      // Update state with scroll percentage
      setScrollWidth(scrolled > 100 ? 100 : scrolled);
    };

    // Add scroll event listener
    window.addEventListener('scroll', handleScroll);

    // Cleanup event listener on component unmount
    return () => {
      window.removeEventListener('scroll', handleScroll);
    };
  }, []);

  return (
    <div
      style={{
        position: 'fixed',
        top: 0,
        left: 0,
        width: `${scrollWidth}%`,
        height: '4px',
        background: 'linear-gradient(90deg, #00f260 0%, #0575e6 100%)',
        transition: 'width 0.1s ease-out',
        zIndex: 99999,
        pointerEvents: 'none', // Allow clicks to pass through
      }}
    />
  );
};

export default ProgressBar;