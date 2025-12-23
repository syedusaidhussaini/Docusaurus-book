// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  modulesSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Middleware for Humanoid Control',
      items: [
        'ros2-middleware/constitution',
        'ros2-middleware/plan',
        {
          type: 'category',
          label: 'Chapter 1: Introduction to ROS 2',
          items: ['ros2-middleware/chapter-1-introduction-to-ros2/index']
        },
        {
          type: 'category',
          label: 'Chapter 2: Distributed Control Patterns',
          items: ['ros2-middleware/chapter-2-distributed-control/index']
        },
        {
          type: 'category',
          label: 'Chapter 3: Advanced ROS 2 Patterns',
          items: ['ros2-middleware/chapter-3-advanced-ros2-patterns/index']
        }
      ]
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin Simulation for Humanoid Robotics',
      items: [
        'digital-twin/constitution',
        'digital-twin/plan',
        {
          type: 'category',
          label: 'Chapter 4: Gazebo Simulation',
          items: ['digital-twin/chapter-4-gazebo-simulation/index']
        },
        {
          type: 'category',
          label: 'Chapter 5: Unity for Human-Robot Interaction',
          items: ['digital-twin/chapter-5-unity-hri/index']
        },
        {
          type: 'category',
          label: 'Chapter 6: Simulation-to-Reality Transfer',
          items: ['digital-twin/chapter-6-simulation-to-reality/index']
        }
      ]
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'isaac-ai-brain/constitution',
        'isaac-ai-brain/plan',
        {
          type: 'category',
          label: 'Chapter 7: Isaac Sim for Photorealistic Simulation',
          items: ['isaac-ai-brain/chapter-7-isaac-sim-photorealistic-simulation/index']
        },
        {
          type: 'category',
          label: 'Chapter 8: Synthetic Data Generation for Perception Models',
          items: ['isaac-ai-brain/chapter-8-synthetic-data-generation/index']
        },
        {
          type: 'category',
          label: 'Chapter 9: Isaac ROS and Nav2 Integration',
          items: ['isaac-ai-brain/chapter-9-isaac-ros-vslam-nav2/index']
        }
      ]
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'vla/constitution',
        'vla/plan',
        {
          type: 'category',
          label: 'Chapter 10: Voice Input and Natural Language Processing',
          items: ['vla/chapter-10-voice-input-natural-language-processing/index']
        },
        {
          type: 'category',
          label: 'Chapter 11: Vision Perception and Scene Understanding',
          items: ['vla/chapter-11-vision-perception-scene-understanding/index']
        },
        {
          type: 'category',
          label: 'Chapter 12: Cognitive Planning and Action Execution',
          items: ['vla/chapter-12-cognitive-planning-action-execution/index']
        }
      ]
    }
  ],
};

export default sidebars;