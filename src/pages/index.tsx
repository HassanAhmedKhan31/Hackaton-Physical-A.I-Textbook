import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';
// ✅ Correct Import: We import the component, not the client logic
import UserMenu from '@site/src/components/Auth/UserMenu'; 

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        
        {/* Main Title */}
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        
        <p className="hero__subtitle">{siteConfig.tagline}</p>

        {/* Start Learning Button */}
        <div className={styles.buttons} style={{ marginTop: '30px' }}>
         <Link
            className="button button--secondary button--lg"
            to="/module-1/intro-nervous-system">
            Start Learning Physical AI 🤖
          </Link>
        </div>

      </div>
    </header>
  );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <div className="container padding-vert--xl">
          <div className="text--center">
            <h2>Welcome to the Future of Robotics</h2>
            <p>
              This textbook covers ROS 2, Gazebo Simulation, NVIDIA Isaac, 
              and Humanoid Control using Generative AI.
            </p>
          </div>
        </div>
      </main>
    </Layout>
  );
}