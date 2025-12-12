import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

// Module data - mapping modules to their chapters
const modulesData = [
  {
    id: 1,
    title: "Module 1: ROS 2 Fundamentals",
    description: "Weeks 1-5: Learn the fundamentals of ROS 2, the middleware for robotics applications",
    chapters: [
      "Introduction to ROS 2",
      "ROS 2 Architecture",
      "Nodes and Topics",
      "Services and Actions",
      "Launch Files and Parameters"
    ],
    link: "/docs/module-1-ros2/intro"
  },
  {
    id: 2,
    title: "Module 2: Simulation Environments",
    description: "Weeks 6-7: Explore robotics simulation environments and tools",
    chapters: [
      "Introduction to Simulation",
      "Gazebo and Ignition"
    ],
    link: "/docs/module-2-simulation/intro"
  },
  {
    id: 3,
    title: "Module 3: NVIDIA Isaac Platform",
    description: "Weeks 8-10: Develop robotics applications with NVIDIA Isaac platform",
    chapters: [
      "Introduction to Isaac",
      "Isaac Sim",
      "Isaac ROS"
    ],
    link: "/docs/module-3-isaac/intro"
  },
  {
    id: 4,
    title: "Module 4: Vision-Language-Action Integration",
    description: "Weeks 11-13: Build integrated systems combining perception, reasoning, and action",
    chapters: [
      "Vision and Perception",
      "Language Understanding",
      "Action Execution"
    ],
    link: "/docs/module-4-vla/intro"
  }
];

function ModuleCard({ module }: { module: (typeof modulesData)[0] }) {
  return (
    <div className={clsx('col col--6 margin-bottom--lg')}>
      <div className={clsx('card', styles.moduleCard)}>
        <div className="card__header">
          <Heading as="h3">{module.title}</Heading>
          <p>{module.description}</p>
        </div>
        <div className="card__body">
          <ul className={styles.chapterList}>
            {module.chapters.map((chapter, index) => (
              <li key={index}>{chapter}</li>
            ))}
          </ul>
        </div>
        <div className="card__footer">
          <Link className="button button--primary" to={module.link}>
            Start Module
          </Link>
        </div>
      </div>
    </div>
  );
}

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container text--center">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">
          {siteConfig.tagline}
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Reading - Introduction
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="A 13-Week Graduate Course on Advanced Robotics">
      <HomepageHeader />
      <main>
        <section className={styles.modulesSection}>
          <div className="container">
            <div className="row">
              <div className="col col--12">
                <Heading as="h2" className={clsx('margin-bottom--lg', styles.sectionTitle)}>
                  Course Modules
                </Heading>
                <p className="margin-bottom--xl">
                  This comprehensive course is divided into four interconnected modules,
                  each building upon the previous one to provide you with a deep understanding
                  of physical AI and humanoid robotics.
                </p>
              </div>
            </div>
            <div className="row">
              {modulesData.map((module) => (
                <ModuleCard key={module.id} module={module} />
              ))}
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
