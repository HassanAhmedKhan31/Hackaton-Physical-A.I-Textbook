/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      items: [
        'module-1/intro-nervous-system',
        'module-1/nodes-and-rclpy',
        'module-1/topics-and-messages',
        'module-1/services-actions',
        'module-1/urdf-humanoid-body',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: ROS 2 Fundamentals',
      items: [
        'module-2/workspaces-and-packages',
        'module-2/launch-files',
        'module-2/parameters',
        'module-2/actions',
        'module-2/custom-interfaces',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Robot Simulation',
      items: [
        'module-3/gazebo-environment',
        'module-3/urdf-vs-sdf',
        'module-3/physics-and-worlds',
        'module-3/simulating-sensors',
        'module-3/unity-visualization',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: NVIDIA Isaac AI',
      items: [
        'module-4/isaac-sim-omniverse',
        'module-4/isaac-ros-gems',
        'module-4/nav2-and-vslam',
        'module-4/synthetic-data-generation',
        'module-4/rl-and-sim-to-real',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Humanoid Development',
      items: [
        'module-5/kinematics-dynamics',
        'module-5/bipedal-locomotion',
        'module-5/manipulation-grasping',
        'module-5/human-robot-interaction',
        'module-5/capstone-integration',
      ],
    },
    {
      type: 'category',
      label: 'Module 6: Conversational Robotics',
      items: [
        'module-6/intro-conversational-robotics',
        'module-6/hearing-with-whisper',
        'module-6/thinking-with-llms',
        'module-6/multimodal-interaction',
        'module-6/future-ethics',
      ],
    },
  ],
};

export default sidebars;