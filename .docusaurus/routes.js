import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/',
    component: ComponentCreator('/', 'e5f'),
    exact: true
  },
  {
    path: '/',
    component: ComponentCreator('/', '220'),
    routes: [
      {
        path: '/',
        component: ComponentCreator('/', '862'),
        routes: [
          {
            path: '/',
            component: ComponentCreator('/', '84a'),
            routes: [
              {
                path: '/module-1/intro-nervous-system',
                component: ComponentCreator('/module-1/intro-nervous-system', '18a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-1/nodes-and-rclpy',
                component: ComponentCreator('/module-1/nodes-and-rclpy', '96e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-1/services-actions',
                component: ComponentCreator('/module-1/services-actions', '5f6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-1/topics-and-messages',
                component: ComponentCreator('/module-1/topics-and-messages', 'f33'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-1/urdf-humanoid-body',
                component: ComponentCreator('/module-1/urdf-humanoid-body', '51b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-2/actions',
                component: ComponentCreator('/module-2/actions', '140'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-2/custom-interfaces',
                component: ComponentCreator('/module-2/custom-interfaces', '89e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-2/launch-files',
                component: ComponentCreator('/module-2/launch-files', '5ea'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-2/parameters',
                component: ComponentCreator('/module-2/parameters', '331'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-2/workspaces-and-packages',
                component: ComponentCreator('/module-2/workspaces-and-packages', '222'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-3/gazebo-environment',
                component: ComponentCreator('/module-3/gazebo-environment', '338'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-3/physics-and-worlds',
                component: ComponentCreator('/module-3/physics-and-worlds', 'af0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-3/simulating-sensors',
                component: ComponentCreator('/module-3/simulating-sensors', '7fc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-3/unity-visualization',
                component: ComponentCreator('/module-3/unity-visualization', '8b2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-3/urdf-vs-sdf',
                component: ComponentCreator('/module-3/urdf-vs-sdf', '950'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-4/isaac-ros-gems',
                component: ComponentCreator('/module-4/isaac-ros-gems', '615'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-4/isaac-sim-omniverse',
                component: ComponentCreator('/module-4/isaac-sim-omniverse', '00a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-4/nav2-and-vslam',
                component: ComponentCreator('/module-4/nav2-and-vslam', '63e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-4/rl-and-sim-to-real',
                component: ComponentCreator('/module-4/rl-and-sim-to-real', 'd05'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-4/synthetic-data-generation',
                component: ComponentCreator('/module-4/synthetic-data-generation', '722'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-5/bipedal-locomotion',
                component: ComponentCreator('/module-5/bipedal-locomotion', 'c52'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-5/capstone-integration',
                component: ComponentCreator('/module-5/capstone-integration', '01c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-5/human-robot-interaction',
                component: ComponentCreator('/module-5/human-robot-interaction', 'ed8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-5/kinematics-dynamics',
                component: ComponentCreator('/module-5/kinematics-dynamics', '436'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-5/manipulation-grasping',
                component: ComponentCreator('/module-5/manipulation-grasping', 'd1d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-6/future-ethics',
                component: ComponentCreator('/module-6/future-ethics', '6a7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-6/hearing-with-whisper',
                component: ComponentCreator('/module-6/hearing-with-whisper', 'd58'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-6/intro-conversational-robotics',
                component: ComponentCreator('/module-6/intro-conversational-robotics', '62e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-6/multimodal-interaction',
                component: ComponentCreator('/module-6/multimodal-interaction', '465'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-6/thinking-with-llms',
                component: ComponentCreator('/module-6/thinking-with-llms', 'c72'),
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
