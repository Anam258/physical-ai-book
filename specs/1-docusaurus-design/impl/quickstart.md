# Quickstart Guide: Electric Blue & Dark Mode Docusaurus Design

## Prerequisites

- Node.js 16+ installed
- npm or yarn package manager
- Access to the `/frontend` directory of the Physical AI & Humanoid Robotics Handbook project

## Installation Steps

### 1. Navigate to Frontend Directory
```bash
cd frontend
```

### 2. Install Dependencies (if not already installed)
```bash
npm install
```

### 3. Update Custom CSS File
Replace the content of `src/css/custom.css` with the following:

```css
/**
 * Custom CSS for Electric Blue & Dark Mode Theme
 * Physical AI & Humanoid Robotics Handbook
 */

/* Color Variables */
:root {
  --ifm-color-primary: #00F5FF;
  --ifm-color-primary-dark: rgb(0, 221, 230);
  --ifm-color-primary-darker: rgb(0, 208, 217);
  --ifm-color-primary-darkest: rgb(0, 172, 180);
  --ifm-color-primary-light: rgb(38, 247, 255);
  --ifm-color-primary-lighter: rgb(51, 248, 255);
  --ifm-color-primary-lightest: rgb(89, 251, 255);
  --ifm-background-color: #0A0E2A;
  --ifm-background-surface-color: #0A0E2A;
  --ifm-navbar-background-color: rgba(10, 14, 42, 0.7);
  --ifm-sidebar-background-color: rgba(10, 14, 42, 0.7);
  --ifm-footer-background-color: #0A0E2A;
  --ifm-text-color: #FFFFFF;
  --ifm-text-color-secondary: #B0B0B0;
}

/* Glassmorphism for Navbar */
.navbar {
  backdrop-filter: blur(10px);
  background: rgba(10, 14, 42, 0.7) !important;
  border-bottom: 1px solid rgba(0, 245, 255, 0.1);
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
}

/* Glassmorphism for Sidebar */
.menu {
  backdrop-filter: blur(10px);
  background: rgba(10, 14, 42, 0.7) !important;
  border-right: 1px solid rgba(0, 245, 255, 0.1);
}

/* Gradient Text Utility */
.gradient-text {
  background: linear-gradient(90deg, #00F5FF, #8A2BE2);
  -webkit-background-clip: text;
  background-clip: text;
  -webkit-text-fill-color: transparent;
}

/* Card Hover Effects */
.card {
  transition: transform 0.3s ease, box-shadow 0.3s ease;
  background: rgba(10, 14, 42, 0.5);
  border: 1px solid rgba(0, 245, 255, 0.1);
  border-radius: 12px;
}

.card:hover {
  transform: translateY(-5px);
  box-shadow: 0 10px 20px rgba(0, 245, 255, 0.3);
}

/* Custom Button Styles */
.button--primary {
  background: linear-gradient(90deg, #00F5FF, #8A2BE2);
  border: none;
}

.button--primary:hover {
  transform: translateY(-2px);
  box-shadow: 0 6px 12px rgba(0, 245, 255, 0.4);
}

/* Dark Mode Specific Adjustments */
html[data-theme='dark'] {
  --ifm-background-color: #0A0E2A;
  --ifm-background-surface-color: #0A0E2A;
  --ifm-navbar-background-color: rgba(10, 14, 42, 0.7);
  --ifm-sidebar-background-color: rgba(10, 14, 42, 0.7);
}

/* Code Block Styling */
.docusaurus-highlight-code-line {
  background-color: rgba(0, 245, 255, 0.1);
}

/* Link Styling */
a {
  color: var(--ifm-color-primary);
}

a:hover {
  color: var(--ifm-color-primary-light);
  text-decoration: underline;
}

/* Table Styling */
.table-wrapper {
  background: rgba(10, 14, 42, 0.5);
  border-radius: 8px;
  overflow: hidden;
}

/* Footer Styling */
.footer {
  background: #0A0E2A;
  color: #B0B0B0;
}

/* Responsive Adjustments */
@media (max-width: 996px) {
  .navbar {
    background: #0A0E2A !important;
  }

  .menu {
    background: #0A0E2A !important;
  }
}
```

### 4. Create Enhanced Homepage
Create or replace `src/pages/index.js` with the following content:

```jsx
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title gradient-text">
          {siteConfig.title}
        </h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg gradient-button"
            to="/docs/weeks-1-2-introduction-to-physical-ai">
            Begin Your Journey - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Handbook: The Future of Embodied Intelligence">
      <HomepageHeader />
      <main>
        <section className={styles.featureGrid}>
          <div className="container">
            <h2 className="gradient-text">Course Structure</h2>
            <p className={styles.sectionSubtitle}>14 Comprehensive Chapters Across 13 Weeks</p>

            <div className={styles.weekGrid}>
              {/* Week 1-2 */}
              <div className="card">
                <div className="card__body">
                  <h3>Weeks 1-2: Introduction</h3>
                  <p>Foundational concepts of Physical AI and embodied intelligence</p>
                </div>
              </div>

              {/* Weeks 3-5 */}
              <div className="card">
                <div className="card__body">
                  <h3>Weeks 3-5: ROS 2 Fundamentals</h3>
                  <p>Core architecture, nodes, topics, services, and actions</p>
                </div>
              </div>

              {/* Weeks 6-7 */}
              <div className="card">
                <div className="card__body">
                  <h3>Weeks 6-7: Robot Simulation</h3>
                  <p>Gazebo & Unity for physics simulation and visualization</p>
                </div>
              </div>

              {/* Weeks 8-10 */}
              <div className="card">
                <div className="card__body">
                  <h3>Weeks 8-10: NVIDIA Isaac Platform</h3>
                  <p>Isaac Sim, Isaac ROS, and reinforcement learning for robotics</p>
                </div>
              </div>

              {/* Weeks 11-12 */}
              <div className="card">
                <div className="card__body">
                  <h3>Weeks 11-12: Humanoid Development</h3>
                  <p>Kinematics, dynamics, and locomotion control</p>
                </div>
              </div>

              {/* Week 13 */}
              <div className="card">
                <div className="card__body">
                  <h3>Week 13: Conversational Robotics</h3>
                  <p>Integrating LLMs and speech recognition for HRI</p>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
```

### 5. Create Homepage CSS Module
Create `src/pages/index.module.css` with the following content:

```css
/**
 * CSS files with the .module.css suffix will be treated as CSS modules
 * locally scoped CSS for the homepage
 */

.heroBanner {
  padding: 4rem 0;
  text-align: center;
  position: relative;
  overflow: hidden;
  background: linear-gradient(135deg, #0A0E2A 0%, #1A1E3A 100%);
}

@media screen and (max-width: 996px) {
  .heroBanner {
    padding: 2rem;
  }
}

.buttons {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 1rem;
}

.gradient-button {
  background: linear-gradient(90deg, #00F5FF, #8A2BE2) !important;
  border: none;
  color: white !important;
  padding: 1rem 2rem;
  font-size: 1.2rem;
  transition: all 0.3s ease;
}

.gradient-button:hover {
  transform: translateY(-3px);
  box-shadow: 0 10px 20px rgba(0, 245, 255, 0.3);
  text-decoration: none;
  color: white !important;
}

.featureGrid {
  padding: 4rem 0;
  background-color: #0A0E2A;
}

.sectionSubtitle {
  text-align: center;
  color: #B0B0B0;
  margin-bottom: 3rem;
  font-size: 1.2rem;
}

.weekGrid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
  margin-top: 2rem;
}

.weekGrid .card {
  transition: transform 0.3s ease, box-shadow 0.3s ease;
  background: rgba(10, 14, 42, 0.5);
  border: 1px solid rgba(0, 245, 255, 0.1);
  border-radius: 12px;
}

.weekGrid .card:hover {
  transform: translateY(-5px);
  box-shadow: 0 10px 20px rgba(0, 245, 255, 0.3);
}

.weekGrid .card__body {
  padding: 1.5rem;
}

.weekGrid .card h3 {
  color: #00F5FF;
  margin-bottom: 0.5rem;
}

.weekGrid .card p {
  color: #B0B0B0;
}

/* Gradient text styling */
.gradient-text {
  background: linear-gradient(90deg, #00F5FF, #8A2BE2);
  -webkit-background-clip: text;
  background-clip: text;
  -webkit-text-fill-color: transparent;
  display: inline-block;
}
```

### 6. Update Docusaurus Configuration
In `docusaurus.config.js`, ensure the following settings are present:

```javascript
// Make sure these settings exist in your docusaurus.config.js
const config = {
  // ... other config
  themeConfig: {
    // ... other theme config
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: false, // Always use dark mode as default
    },
    // ... rest of config
  }
};
```

### 7. Start Development Server
```bash
npm run start
```

## Verification Steps

1. Visit `http://localhost:3000` in your browser
2. Verify the dark mode theme is applied by default
3. Check that navbar and sidebar have glassmorphism effect
4. Verify gradient text in the hero section
5. Test hover effects on cards and buttons
6. Confirm responsive behavior on different screen sizes
7. Ensure all 14 chapters are accessible through navigation

## Deployment

When ready for production:

```bash
npm run build
```

The build artifacts will be in the `build` folder and ready for deployment to GitHub Pages.