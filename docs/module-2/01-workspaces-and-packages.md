---
sidebar_position: 1
title: The Workspace (The Robot's Home)
---

# The Workspace (The Robot's Home)

Before we start building the brain of our humanoid, we need a place to work. In ROS 2, this is called a **Workspace**.

## The Kitchen Analogy

Managing software for a complex robot can be messy. To keep things organized, we can think of the workspace as a **Kitchen**.

*   **Workspace (`~/ros2_ws`)**: The entire kitchen where you work.
*   **Package**: A specific **Recipe**. You might have a recipe for "Leg Control" and another for "Vision Processing."
*   **Source Code (`src/`)**: The raw ingredients (Python scripts, C++ files).
*   **`colcon build`**: The act of **Cooking**. You can't eat raw ingredients; you must process them so the robot can consume them.
*   **`install/`**: The serving counter. Once the food is cooked (built), it is placed here, ready to be served.

## Workspace Structure

When you create a workspace, it typically has four main directories:

1.  **`src/` (Source Space):** This is where your code lives. This is the *only* folder you should manually edit.
2.  **`build/` (Build Space):** Intermediate files where `colcon` does its work. (Don't touch this).
3.  **`install/` (Install Space):** Where the final executable programs and libraries end up.
4.  **`log/` (Log Space):** Records of the build process, useful for debugging if a build fails.

```text
~/ros2_ws/
├── src/                # YOUR CODE HERE
│   ├── my_humanoid_control/
│   └── my_vision_pkg/
├── build/              # CMake cache, intermediate files
├── install/            # Executables and setup files
└── log/                # Build logs
```

## Creating Your First Package

Let's create the first recipe for our robot: the control package.

1.  **Go to your source folder:**
    ```bash
    cd ~/ros2_ws/src
    ```

2.  **Create the package:**
    We use `ros2 pkg create` to generate the skeleton. We specify `ament_python` because we are using Python.

    ```bash
    ros2 pkg create --build-type ament_python my_humanoid_control
    ```

3.  **Build the workspace (Cook the food):**
    Go back to the root of the workspace to build.

    ```bash
    cd ~/ros2_ws
    colcon build
    ```

4.  **Source the workspace (Serve the food):**
    Before ROS 2 can "taste" (find) your new package, you must source the setup file.

    ```bash
    source install/setup.bash
    ```

    *Note: On Windows, you would use `call install/setup.bat`.*

## Overlay Workspaces

You might wonder, "doesn't ROS 2 already have packages installed?" Yes, those live in the **Underlay** (usually in `/opt/ros/humble`). Your workspace is an **Overlay**.

If you define a package in your Overlay with the same name as one in the Underlay, your version takes precedence. This is like deciding to cook your own "Pizza" at home instead of ordering the standard "Pizza" from the store.

In the next section, we will learn how to wake up our robot using Launch Files.
