import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContainer}>
          <div className={styles.heroText}>
            <Heading as="h1" className="hero__title gradient-text">
              The Future of Embodied Intelligence
            </Heading>
            <p className="hero__subtitle gradient-text">Master Physical AI, ROS 2, and Humanoids.</p>
            <div className={styles.buttons}>
              <Link
                className="button button--primary button--lg glowing-button"
                to="docs/Introduction/intro-physical-ai">
                Start Learning
              </Link>
            </div>
          </div>
          <img
            src="img/hero-robot-cute.png"
            alt="Cute AI Robot Assistant"
            className="hero-robot-static"
          />
        </div>
      </div>
    </header>
  );
}

function TechStackStrip() {
  return (
    <section className={styles.techStackStrip}>
      <div className="container">
        <div className={styles.techStackContent}>
          <p className={styles.techStackText}>Powered By Industry Standards:</p>
          <div className={styles.techStackItems}>
            <span className={styles.techItem}>NVIDIA Isaac Sim</span>
            <span className={styles.techItem}>ROS 2</span>
            <span className={styles.techItem}>Gazebo</span>
            <span className={styles.techItem}>Python</span>
            <span className={styles.techItem}>OpenCV</span>
          </div>
        </div>
      </div>
    </section>
  );
}

function ValueProps() {
  const valueProps = [
    {
      title: "Cutting-Edge Technology",
      description: "Learn with the latest tools and frameworks used in modern robotics development.",
      icon: "🤖"
    },
    {
      title: "Industry-Aligned Curriculum",
      description: "Master skills that match real-world robotics and AI industry requirements.",
      icon: "💼"
    },
    {
      title: "Hands-On Learning",
      description: "Build and simulate humanoid robots with practical exercises and projects.",
      icon: "🛠️"
    }
  ];

  return (
    <section className={styles.valuePropsSection}>
      <div className="container">
        <div className={styles.sectionTitle}>
          <Heading as="h2" className="gradient-text">Why Physical AI?</Heading>
        </div>
        <div className={styles.valuePropsGrid}>
          {valueProps.map((prop, index) => (
            <div className={styles.glassCard} key={index}>
              <div className={styles.glassCardHeader}>
                <span className={styles.icon}>{prop.icon}</span>
                <Heading as="h3">{prop.title}</Heading>
              </div>
              <div className={styles.glassCardBody}>
                <p>{prop.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home - ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Handbook: The Future of Embodied Intelligence">
      <HomepageHeader />
      <TechStackStrip />
      <main>
        <ValueProps />
      </main>
    </Layout>
  );
}
