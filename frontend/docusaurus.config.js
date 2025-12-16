// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'The Future of Embodied Intelligence',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://Anam258.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/physical-ai-book/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Anam258', // Usually your GitHub org/user name.
  projectName: 'physical-ai-book', // Usually your repo name.
  deploymentBranch: 'gh-pages',
  onBrokenLinks: 'ignore',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      colorMode: {
        defaultMode: 'dark',
        disableSwitch: false,
        respectPrefersColorScheme: false,
      },
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Physical AI & Humanoid Robotics Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Handbook',
          },
          {
            href: 'https://github.com/Anam258/',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
footer: {
        style: 'dark',
        // Footer Logo (for branding)
        logo: {
          alt: 'Physical AI Robot Logo',
          src: 'img/logo.svg',
          href: '/',
          width: 160,
          height: 51,
        },
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Introduction',
                to: 'docs/Introduction/intro-physical-ai',
              },
              {
                label: 'ROS 2 Module',
                to: 'docs/Module-01-ROS2/ros2-fundamentals-1',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/Anam258/physical-ai-book',
              },
              {
                label: 'LinkedIn',
                href: 'https://www.linkedin.com/in/anum-a-1b82082b5',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'Quiz',
                to: 'docs/Introduction/intro-physical-ai', // Placeholder for quiz feature
              },
            ],
          },
        ],
        // Copyright section
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Handbook. Built with ❤️ and AI by Anum.`,
      },
      
        prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;
