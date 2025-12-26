import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '7c2'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '3e5'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'bbf'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'd75'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '724'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', 'cb8'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '339'),
    exact: true
  },
  {
    path: '/',
    component: ComponentCreator('/', '6ba'),
    exact: true
  },
  {
    path: '/',
    component: ComponentCreator('/', '2b2'),
    routes: [
      {
        path: '/',
        component: ComponentCreator('/', '77b'),
        routes: [
          {
            path: '/',
            component: ComponentCreator('/', '498'),
            routes: [
              {
                path: '/module-1/intro-nervous-system',
                component: ComponentCreator('/module-1/intro-nervous-system', 'cc7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-1/nodes-and-rclpy',
                component: ComponentCreator('/module-1/nodes-and-rclpy', '62e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-1/services-actions',
                component: ComponentCreator('/module-1/services-actions', '1fc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-1/topics-and-messages',
                component: ComponentCreator('/module-1/topics-and-messages', '080'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-1/urdf-humanoid-body',
                component: ComponentCreator('/module-1/urdf-humanoid-body', 'fef'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-2/actions',
                component: ComponentCreator('/module-2/actions', 'a9e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-2/custom-interfaces',
                component: ComponentCreator('/module-2/custom-interfaces', 'a70'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-2/launch-files',
                component: ComponentCreator('/module-2/launch-files', '442'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-2/parameters',
                component: ComponentCreator('/module-2/parameters', 'e59'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-2/workspaces-and-packages',
                component: ComponentCreator('/module-2/workspaces-and-packages', '4e6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-3/gazebo-environment',
                component: ComponentCreator('/module-3/gazebo-environment', '51f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-3/physics-and-worlds',
                component: ComponentCreator('/module-3/physics-and-worlds', '349'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-3/simulating-sensors',
                component: ComponentCreator('/module-3/simulating-sensors', '85e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-3/unity-visualization',
                component: ComponentCreator('/module-3/unity-visualization', '5f9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-3/urdf-vs-sdf',
                component: ComponentCreator('/module-3/urdf-vs-sdf', '6c2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-4/isaac-ros-gems',
                component: ComponentCreator('/module-4/isaac-ros-gems', '3b3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-4/isaac-sim-omniverse',
                component: ComponentCreator('/module-4/isaac-sim-omniverse', 'e62'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-4/nav2-and-vslam',
                component: ComponentCreator('/module-4/nav2-and-vslam', 'edf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-4/rl-and-sim-to-real',
                component: ComponentCreator('/module-4/rl-and-sim-to-real', '028'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-4/synthetic-data-generation',
                component: ComponentCreator('/module-4/synthetic-data-generation', 'c27'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-5/bipedal-locomotion',
                component: ComponentCreator('/module-5/bipedal-locomotion', 'd49'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-5/capstone-integration',
                component: ComponentCreator('/module-5/capstone-integration', '2ab'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-5/human-robot-interaction',
                component: ComponentCreator('/module-5/human-robot-interaction', 'b22'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-5/kinematics-dynamics',
                component: ComponentCreator('/module-5/kinematics-dynamics', '57e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-5/manipulation-grasping',
                component: ComponentCreator('/module-5/manipulation-grasping', '20b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-6/future-ethics',
                component: ComponentCreator('/module-6/future-ethics', '785'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-6/hearing-with-whisper',
                component: ComponentCreator('/module-6/hearing-with-whisper', 'ef6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-6/intro-conversational-robotics',
                component: ComponentCreator('/module-6/intro-conversational-robotics', 'b1d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-6/multimodal-interaction',
                component: ComponentCreator('/module-6/multimodal-interaction', 'cf8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-6/thinking-with-llms',
                component: ComponentCreator('/module-6/thinking-with-llms', 'ede'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
