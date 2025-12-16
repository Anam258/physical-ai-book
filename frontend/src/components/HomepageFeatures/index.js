import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

function RobotIcon() {
  return (
    <svg xmlns="http://www.w3.org/2000/svg" width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
      <circle cx="12" cy="10" r="2"></circle>
      <path d="M12 12v8l-4-2"></path>
      <path d="M8 10V7a4 4 0 0 1 8 0v3"></path>
      <path d="M8 21v-1a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v1"></path>
      <path d="M20 12c0 1.7-1.3 3-3 3s-3-1.3-3-3 1.3-3 3-3 3 1.3 3 3"></path>
      <path d="M4 12c0 1.7 1.3 3 3 3s3-1.3 3-3-1.3-3-3-3-3 1.3-3 3"></path>
    </svg>
  );
}

function ClipboardIcon() {
  return (
    <svg xmlns="http://www.w3.org/2000/svg" width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
      <path d="M9 12l2 2 4-4"/>
      <path d="M21 12a9 9 0 0 0-9-9 9 9 0 0 0-9 9 9 9 0 0 0 9 9 9 9 0 0 0 9-9z"/>
      <circle cx="12" cy="12" r="3"/>
      <path d="M18 12a6 6 0 0 1-6 6"/>
      <path d="M6 12a6 6 0 0 1 6 6"/>
    </svg>
  );
}

function LanguageIcon() {
  return (
    <svg xmlns="http://www.w3.org/2000/svg" width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
      <path d="M12 2a10 10 0 0 0-8 4 10 10 0 0 0 8 16 10 10 0 0 0 8-16 10 10 0 0 0-8-4z"/>
      <path d="M2 12h20"/>
      <path d="M12 2a10 10 0 0 0 0 20 10 10 0 0 0 0-20z"/>
      <path d="M8 16l4-4 4 4"/>
      <path d="M8 8l4 4 4-4"/>
    </svg>
  );
}

function UserIcon() {
  return (
    <svg xmlns="http://www.w3.org/2000/svg" width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
      <path d="M19 21v-2a4 4 0 0 0-4-4H9a4 4 0 0 0-4 4v2"/>
      <circle cx="12" cy="7" r="4"/>
    </svg>
  );
}

const FeatureList = [
  {
    title: '🤖 AI RAG Tutor',
    icon: RobotIcon,
    description: (
      <>
        Chat with our AI trained specifically on this handbook. Ask complex questions and get instant, context-aware answers.
      </>
    ),
  },
  {
    title: '📝 Smart Quizzes',
    icon: ClipboardIcon,
    description: (
      <>
        Challenge yourself with interactive quizzes at the end of every chapter. Get instant feedback and track your progress.
      </>
    ),
  },
  {
    title: '🇵🇰 Smart Urdu Translation',
    icon: LanguageIcon,
    description: (
      <>
        Select any text to get instant explanations in professional Urdu script, while keeping technical terms like ROS 2 intact.
      </>
    ),
  },
  {
    title: '🎯 Personalized Learning',
    icon: UserIcon,
    description: (
      <>
        AI adapts concepts to your background—perfect for both Python experts and beginners.      </>
    ),
  },
];

function Feature({icon: Icon, title, description}) {
  return (
    <div className={clsx('col col--6', 'margin-bottom--lg')}>
      <div className={styles.glassCard}>
        <div className="text--center">
          <div className={styles.iconWrapper}>
            <Icon />
          </div>
        </div>
        <div className="text--center padding-horiz--md">
          <Heading as="h3">{title}</Heading>
          <p>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
