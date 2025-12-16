import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import { motion, useReducedMotion } from 'framer-motion';

import Heading from '@theme/Heading';
import styles from './index.module.css';

// ✅ Hamara Feature Component
import HomepageFeatures from '@site/src/components/HomepageFeatures';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  const shouldReduceMotion = useReducedMotion();

  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContainer}>
          <div className={styles.heroText}>
            {/* ✨ Updated Simple & Catchy Heading */}
            <Heading as="h1" className="hero__title gradient-text">
              Build the Robots of Tomorrow
            </Heading>
            
            {/* ✨ Clear Subtitle */}
            <p className="hero__subtitle gradient-text" style={{ fontSize: '1.3rem', marginTop: '10px' }}>
              The Interactive Handbook for Physical AI, ROS 2, and Humanoid Robotics.
            </p>
            
            <div className={styles.buttons} style={{ marginTop: '2rem' }}>
              <Link
                className="button button--primary button--lg"
                to="docs/Introduction/intro-physical-ai"
                // ✨ Button Styling (Neon Glow)
                style={{
                  backgroundColor: '#00f7ff',
                  color: '#0f172a',
                  fontWeight: 'bold',
                  border: 'none',
                  boxShadow: '0 0 20px rgba(0, 247, 255, 0.6)',
                  transition: 'all 0.3s ease',
                  padding: '15px 30px',
                  fontSize: '1.2rem',
                  borderRadius: '50px'
                }}>
                🚀 Start Your Journey
              </Link>
            </div>
          </div>
          
          <motion.div
            className={styles.heroRobotContainer}
            animate={shouldReduceMotion ? {} : { y: [-10, 10, -10] }}
            transition={{
              duration: 3,
              repeat: Infinity,
              ease: "easeInOut"
            }}
          >
            <img
              src="img/hero-robot-cute.png"
              alt="Cute AI Robot Assistant"
              className="hero-robot-static"
            />
          </motion.div>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home - ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Handbook">
      <HomepageHeader />
      
      <main>
        {/* ✅ Separator Line & Heading for Features */}
        <div className="container" style={{ textAlign: 'center', marginTop: '4rem', marginBottom: '1rem' }}>
          <hr style={{ border: '0', height: '1px', background: 'linear-gradient(90deg, transparent, #00f7ff, transparent)', marginBottom: '3rem', opacity: 0.5 }} />
          
          <Heading as="h2" className="gradient-text" style={{ fontSize: '2.5rem', fontWeight: 'bold', marginBottom: '0.5rem' }}>
            Everything You Need to Master Physical AI
          </Heading>
          
          <p style={{ fontSize: '1.2rem', color: '#cbd5e1', maxWidth: '600px', margin: '0 auto 2rem auto' }}>
            Experience an intelligent textbook that adapts to you.
          </p>
        </div>

        {/* ✅ Glass Features */}
        <HomepageFeatures />
      </main>
    </Layout>
  );
}