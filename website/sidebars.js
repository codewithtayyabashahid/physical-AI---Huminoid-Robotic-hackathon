// Sidebar configuration for Docusaurus documentation structure

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction to Physical AI',
      items: ['chapters/01-physical-ai-ur'],
    },
  ],
};

module.exports = sidebars;