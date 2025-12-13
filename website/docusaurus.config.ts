import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'A 13-Week Graduate Course on Advanced Robotics',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://Hackathone01withqwen.github.io', // Replace with your base URL
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/hackathoneWithQWEN_V2/', // Adjust based on your repository name

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Hackathone01withqwen', // Usually your GitHub org/user name.
  projectName: 'hackathoneWithQWEN_V2', // Usually your repo name.

  onBrokenLinks: 'throw',
  markdown: {
    format: 'mdx',
    mermaid: true,
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },
  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.13.24/dist/katex.min.css',
      type: 'text/css',
      integrity:
        'sha384-odtC+0UGzzFL/6PNoE8rX/SPcQDXBJ+uRepguP4QkPCm2LBxH3FA3y+fKSiJ+AmM',
      crossorigin: 'anonymous',
    },
  ],

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
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Hackathone01withqwen/hackathoneWithQWEN_V2',
          // Optional: for search functionality
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
          remarkPlugins: [remarkMath],
          rehypePlugins: [rehypeKatex],
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Hackathone01withqwen/hackathoneWithQWEN_V2',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
        sitemap: {
          changefreq: 'weekly',
          priority: 0.5,
          ignorePatterns: ['/tags/**'],
          filename: 'sitemap.xml',
        },
        gtag: {
          trackingID: 'G-XXXXXXXXXX',
          anonymizeIP: true,
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Robotics',
      logo: {
        alt: 'Physical AI and Humanoid Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Modules',
        },
        {
          href: 'https://github.com/Hackathone01withqwen/hackathoneWithQWEN_V2',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Modules',
          items: [
            {
              label: 'Module 1: ROS 2 Fundamentals',
              to: '/docs/module-1-ros2/intro',
            },
            {
              label: 'Module 2: Simulation Environments',
              to: '/docs/module-2-simulation/intro',
            },
            {
              label: 'Module 3: NVIDIA Isaac Platform',
              to: '/docs/module-3-isaac/intro',
            },
            {
              label: 'Module 4: Vision-Language-Action Integration',
              to: '/docs/module-4-vla/intro',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Glossary',
              to: '/docs/glossary',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/Hackathone01withqwen/hackathoneWithQWEN_V2',
            },
          ],
        },
        {
          title: 'Course Info',
          items: [
            {
              label: 'Syllabus',
              to: '/docs/syllabus',
            },
            {
              label: 'Schedule',
              to: '/docs/schedule',
            },
            {
              label: 'Exercises',
              to: '/docs/exercises',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI and Humanoid Robotics Course. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'cpp', 'bash', 'json', 'yaml'], // Add relevant languages for robotics
    },
    algolia: {
      // The application ID provided by Algolia
      appId: 'YOUR_ALGOLIA_APP_ID',

      // Public API key (it is safe to commit this)
      apiKey: 'YOUR_ALGOLIA_API_KEY',

      indexName: 'physical-ai-book',

      // Optional: see doc section below
      contextualSearch: true,

      // Optional: Specify domains where the navigation should occur through
      // window.location instead of history.push. Useful when our Algolia
      // config crawls multiple documentation sites and we want to navigate
      // with window.location.href to them.
      // externalUrlRegex: 'external\\.example\\.com|thirdparty\\.example\\.com',

      // Optional: Replace parts of the item URLs from Algolia. Useful when
      // using the same search index for multiple deployments using a
      // different baseUrl. You can use regexp or string in the `from` param.
      // For example: localhost:3000 vs myCompany.com/docs
      // replaceSearchResultPathname: {
      //   from: '/docs/', // or as RegExp: /\/docs\//
      //   to: '/',
      // },

      // Optional: Algolia search parameters
      searchParameters: {},

      // Optional: path for search page that enabled by default (`false` to disable it)
      searchPagePath: 'search',
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
