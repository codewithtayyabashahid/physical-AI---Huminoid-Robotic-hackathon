/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction to Physical AI',
      items: ['chapters/01-physical-ai'],
    },
    {
      type: 'category',
      label: 'Fundamentals of Robotics',
      items: ['chapters/02-fundamentals'],
    },
    {
      type: 'category',
      label: 'Perception Systems',
      items: ['chapters/03-perception'],
    },
    {
      type: 'category',
      label: 'Motion Planning & Control',
      items: ['chapters/04-motion-planning'],
    },
    {
      type: 'category',
      label: 'Machine Learning for Robotics',
      items: ['chapters/05-machine-learning'],
    },
    {
      type: 'category',
      label: 'Humanoid Locomotion',
      items: ['chapters/06-locomotion'],
    },
    {
      type: 'category',
      label: 'Manipulation & Grasping',
      items: ['chapters/07-manipulation'],
    },
    {
      type: 'category',
      label: 'Human-Robot Interaction',
      items: ['chapters/08-interaction'],
    },
    {
      type: 'category',
      label: 'Sensors and Actuators',
      items: ['chapters/09-sensors-actuators'],
    },
    {
      type: 'category',
      label: 'Simulation and Digital Twins',
      items: ['chapters/10-simulation'],
    },
      {
       type: 'category',
       label: 'System Integration',
       items: ['chapters/11-integration'],
     },
    {
      type: 'category',
      label: 'Deployment & Real-World',
      items: ['chapters/12-deployment'],
     },
  ],
};

module.exports = sidebars;