import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import { useColorMode } from '@docusaurus/theme-common';

import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  const { colorMode, setColorMode } = useColorMode();

  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        
        <div className={styles.buttons} style={{ display: 'flex', gap: '15px', justifyContent: 'center', flexWrap: 'wrap' }}>
          
          {/* Your requested link */}
          <Link
            className="button button--secondary button--lg"
            to="/module-1/intro-nervous-system"> 
            Start Learning Physical AI 🤖
          </Link>

          <button
            className="button button--outline button--secondary button--lg"
            onClick={() => setColorMode(colorMode === 'dark' ? 'light' : 'dark')}
            style={{ minWidth: '160px' }}
          >
            {colorMode === 'dark' ? '☀️ Light Mode' : '🌙 Dark Mode'}
          </button>

        </div>
      </div>
    </header>
  );
}

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="A Hands-on Course in Humanoid Robotics">
      
      <HomepageHeader />
      
      <main>
        <div style={{ padding: '4rem', textAlign: 'center', fontSize: '1.2rem' }}>
          <h3>Welcome to the Future of Robotics</h3>
          <p>
            This textbook covers ROS 2, Gazebo Simulation, NVIDIA Isaac, and Humanoid Control using Generative AI.
          </p>
        </div>
      </main>

    </Layout>
  );
}