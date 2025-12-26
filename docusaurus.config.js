module.exports = {
  title: 'Physical AI Textbook',
  tagline: 'A Hands-on Course in Humanoid Robotics',
  url: 'https://your-docusaurus-test-site.com',
  baseUrl: '/',
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',
  organizationName: 'physical-ai', // Usually your GitHub org/user name.
  projectName: 'physical-ai-textbook', // Usually your repo name.

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          routeBasePath: '/', // Serve the docs at the site's root
        },
        theme: {
          customCss: require.resolve('./src/components/ChatWidget.css'), // Using existing CSS file as custom CSS base
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      navbar: {
        title: 'Physical AI Textbook',
        items: [
          {
  type: 'doc',
  docId: 'module-1/intro-nervous-system', // <--- Match the ID from the error log
  position: 'left',
  label: 'Start Reading',
},
        ],
      },
      footer: {
        style: 'dark',
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook. Built with Docusaurus.`,
      },
    }),
};
