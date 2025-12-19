// Docusaurus configuration for Physical AI & Humanoid Robotics platform
// @ts-check

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'A Comprehensive Guide to Embodied Intelligence',
  favicon: 'img/favicon.ico',
  url: 'https://your-username.github.io',
  baseUrl:'/',
  organizationName: 'panaversity',
  projectName: 'panaversity-hackathon',
  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {any} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          routeBasePath: 'docs',
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {any} */
    ({
      navbar: {
        title: 'Physical AI & Robotics',
        items: [
          {
            type: 'doc',
            docId: 'chapters/01-physical-ai',
            position: 'left',
            label: 'Book',
          },
          {
            href: 'https://github.com/panaversity/panaversity-hackathon',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        copyright: `Copyright © ${new Date().getFullYear()}. Built with Docusaurus.`,
      },
      colorMode: {
        defaultMode: 'dark',
        disableSwitch: true,
        respectPrefersColorScheme: false,
      },
    }),
};

module.exports = config;