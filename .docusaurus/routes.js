import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/github.com/Anam258/my-handbook/markdown-page',
    component: ComponentCreator('/github.com/Anam258/my-handbook/markdown-page', 'b40'),
    exact: true
  },
  {
    path: '/github.com/Anam258/my-handbook/docs',
    component: ComponentCreator('/github.com/Anam258/my-handbook/docs', 'cd4'),
    routes: [
      {
        path: '/github.com/Anam258/my-handbook/docs',
        component: ComponentCreator('/github.com/Anam258/my-handbook/docs', '15a'),
        routes: [
          {
            path: '/github.com/Anam258/my-handbook/docs',
            component: ComponentCreator('/github.com/Anam258/my-handbook/docs', '072'),
            routes: [
              {
                path: '/github.com/Anam258/my-handbook/docs/Advanced-Topics/capstone-autonomous-humanoid',
                component: ComponentCreator('/github.com/Anam258/my-handbook/docs/Advanced-Topics/capstone-autonomous-humanoid', '0b4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/github.com/Anam258/my-handbook/docs/Advanced-Topics/hardware-lab-setup',
                component: ComponentCreator('/github.com/Anam258/my-handbook/docs/Advanced-Topics/hardware-lab-setup', 'fa1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/github.com/Anam258/my-handbook/docs/Advanced-Topics/sim-to-real-transfer',
                component: ComponentCreator('/github.com/Anam258/my-handbook/docs/Advanced-Topics/sim-to-real-transfer', 'b4e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/github.com/Anam258/my-handbook/docs/Introduction/intro-physical-ai',
                component: ComponentCreator('/github.com/Anam258/my-handbook/docs/Introduction/intro-physical-ai', '2ec'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/github.com/Anam258/my-handbook/docs/Module-01-ROS2/ros2-fundamentals-1',
                component: ComponentCreator('/github.com/Anam258/my-handbook/docs/Module-01-ROS2/ros2-fundamentals-1', '24b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/github.com/Anam258/my-handbook/docs/Module-01-ROS2/ros2-fundamentals-2',
                component: ComponentCreator('/github.com/Anam258/my-handbook/docs/Module-01-ROS2/ros2-fundamentals-2', '2c9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/github.com/Anam258/my-handbook/docs/Module-01-ROS2/ros2-fundamentals-3',
                component: ComponentCreator('/github.com/Anam258/my-handbook/docs/Module-01-ROS2/ros2-fundamentals-3', '399'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/github.com/Anam258/my-handbook/docs/Module-02-Simulation/gazebo-simulation-1',
                component: ComponentCreator('/github.com/Anam258/my-handbook/docs/Module-02-Simulation/gazebo-simulation-1', '830'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/github.com/Anam258/my-handbook/docs/Module-02-Simulation/gazebo-simulation-2',
                component: ComponentCreator('/github.com/Anam258/my-handbook/docs/Module-02-Simulation/gazebo-simulation-2', 'f7a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/github.com/Anam258/my-handbook/docs/Module-03-Isaac-Sim/isaac-sim-1',
                component: ComponentCreator('/github.com/Anam258/my-handbook/docs/Module-03-Isaac-Sim/isaac-sim-1', '643'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/github.com/Anam258/my-handbook/docs/Module-03-Isaac-Sim/isaac-sim-2',
                component: ComponentCreator('/github.com/Anam258/my-handbook/docs/Module-03-Isaac-Sim/isaac-sim-2', '55b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/github.com/Anam258/my-handbook/docs/Module-03-Isaac-Sim/isaac-sim-3',
                component: ComponentCreator('/github.com/Anam258/my-handbook/docs/Module-03-Isaac-Sim/isaac-sim-3', 'a03'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/github.com/Anam258/my-handbook/docs/Module-04-VLA-Voice/vla-cognitive-planning',
                component: ComponentCreator('/github.com/Anam258/my-handbook/docs/Module-04-VLA-Voice/vla-cognitive-planning', '6da'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/github.com/Anam258/my-handbook/docs/Module-04-VLA-Voice/vla-voice-action',
                component: ComponentCreator('/github.com/Anam258/my-handbook/docs/Module-04-VLA-Voice/vla-voice-action', 'fa6'),
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
    path: '/github.com/Anam258/my-handbook/',
    component: ComponentCreator('/github.com/Anam258/my-handbook/', '1d2'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
