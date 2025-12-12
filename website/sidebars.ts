import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      collapsed: false,
      items: [
        'intro',
        'syllabus',
        'schedule',
        'glossary',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      collapsed: false,
      items: [
        'module-1-ros2/intro',
        'module-1-ros2/chapter-1',
        'module-1-ros2/chapter-2',
        'module-1-ros2/chapter-3',
        'module-1-ros2/chapter-4',
        'module-1-ros2/chapter-5',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation Environments',
      collapsed: false,
      items: [
        'module-2-simulation/intro',
        'module-2-simulation/chapter-1',
        'module-2-simulation/chapter-2',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Platform',
      collapsed: false,
      items: [
        'module-3-isaac/intro',
        'module-3-isaac/chapter-1',
        'module-3-isaac/chapter-2',
        'module-3-isaac/chapter-3',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Integration',
      collapsed: false,
      items: [
        'module-4-vla/intro',
        'module-4-vla/chapter-1',
        'module-4-vla/chapter-2',
        'module-4-vla/chapter-3',
      ],
    },
    {
      type: 'category',
      label: 'Resources',
      collapsed: false,
      items: [
        'exercises',
        'references',
        'appendix',
      ],
    },
  ],
};

export default sidebars;
