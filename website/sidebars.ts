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
  // Main book structure for Physical AI & Humanoid Robotics
  bookSidebar: [
    'intro',
    {
      type: 'category',
      label: '🤖 Chapter 1: Introduction to Physical AI & Humanoid Robotics',
      collapsed: true,
      className: 'chapter-1-category',
      items: [
        'chapter-1/1.1-introduction',
        'chapter-1/1.2-section-one',
        'chapter-1/1.3-section-two',
        'chapter-1/conclusion',
      ],
    },
    {
      type: 'category',
      label: '🦾 Chapter 2: Fundamentals of Humanoid Design',
      collapsed: true,
      className: 'chapter-2-category',
      items: [
        'chapter-2/2.1-introduction',
        'chapter-2/2.2-biomechanics',
        'chapter-2/2.3-actuators',
        'chapter-2/conclusion',
      ],
    },
    {
      type: 'category',
      label: '⚙️ Chapter 3: Control Systems for Humanoid Robots',
      collapsed: true,
      className: 'chapter-3-category',
      items: [
        'chapter-3/3.1-introduction',
        'chapter-3/3.2-locomotion-control',
        'chapter-3/3.3-balance-maintenance',
        'chapter-3/conclusion',
      ],
    },
    {
      type: 'category',
      label: '👁️ Chapter 4: Sensing and Perception',
      collapsed: true,
      className: 'chapter-4-category',
      items: [
        'chapter-4/4.1-introduction',
        'chapter-4/4.2-sensors',
        'chapter-4/4.3-environment-perception',
        'chapter-4/conclusion',
      ],
    },
    {
      type: 'category',
      label: '🧠 Chapter 5: AI and Learning in Humanoid Robotics',
      collapsed: true,
      className: 'chapter-5-category',
      items: [
        'chapter-5/5.1-introduction',
        'chapter-5/5.2-machine-learning',
        'chapter-5/5.3-reinforcement-learning',
        'chapter-5/conclusion',
      ],
    },
    {
      type: 'category',
      label: '🚀 Chapter 6: Applications and Future Directions',
      collapsed: true,
      className: 'chapter-6-category',
      items: [
        'chapter-6/6.1-introduction',
        'chapter-6/6.2-applications',
        'chapter-6/6.3-future-directions',
        'chapter-6/conclusion',
      ],
    },
  ],
};

export default sidebars;
