# Spatial Awareness (VSLAM & Nav2)

For a humanoid robot to be useful, it must answer two questions: "Where am I?" and "How do I get there?" In this section, we replace traditional Lidar-based SLAM with Visual SLAM using Isaac ROS.

## 1. Visual SLAM (VSLAM)
Traditional SLAM uses 2D Lidar to map a floor plan. **Visual SLAM** uses cameras to track thousands of visual features (corners, edges, textures) in 3D space.

We use the `isaac_ros_visual_slam` GEM.
*   **Input:** Stereo Camera images (Left + Right).
*   **Process:** The GPU tracks feature points across frames to calculate the robot's movement (Odometry).
*   **Output:** The `map` -> `odom` -> `base_link` TF tree.

## 2. Nav2: The Planning Brain
Once the robot knows where it is (thanks to VSLAM), **Nav2** (The ROS 2 Navigation Stack) plans the path.
*   **Global Planner:** Calculates the shortest path from A to B (like Google Maps).
*   **Local Planner:** Controls the motors to follow that path while avoiding sudden obstacles (like a dog running in front).

## 3. The Architecture

Below is the data flow for a visual navigation system.

```mermaid
flowchart TD
    Camera[Stereo Camera\n(Isaac Sim / RealSense)] -->|Raw Images| VSLAM[isaac_ros_visual_slam\n(GPU Node)]
    VSLAM -->|Odometry & Map| TF[/tf Tree\n(Map->Odom->Base)]
    TF --> Nav2[Nav2 Stack\n(Planner & Controller)]
    Lidar[Lidar / Depth Camera] -->|Obstacle Data| Nav2
    Nav2 -->|Velocity Cmd| Motor[Motor Controller\n(Twist Msg)]
```

### Integration Steps
1.  **Simulation:** Your robot in Isaac Sim publishes camera images over the ROS bridge.
2.  **Perception:** The `isaac_ros_visual_slam` node (running in Docker) subscribes to these images.
3.  **Localization:** VSLAM publishes the transform ensuring the robot is correctly placed on the map.
4.  **Action:** You send a "Goal Pose" in RViz, and Nav2 drives the simulated robot to the destination.

```