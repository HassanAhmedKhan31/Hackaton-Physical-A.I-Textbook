# Grasping the World (MoveIt 2)

While the legs handle locomotion, the arms handle **Manipulation**. In ROS 2, the gold standard for controlling robotic arms is **MoveIt 2**.

## 1. Why MoveIt 2?
If you simply tell a robot arm to "go to coordinate X," it might swing its elbow right through its own head or hit a nearby table. MoveIt 2 solves three main problems:
1.  **Motion Planning:** Finding a path from A to B.
2.  **Collision Checking:** Ensuring the path doesn't hit obstacles or the robot itself.
3.  **Kinematics:** Calculating the joint angles (IK) needed for the move.

## 2. The MoveIt Setup Assistant
Configuring MoveIt 2 manually is incredibly difficult. Fortunately, we use the **MoveIt Setup Assistant GUI**.

### The Primary Tool
The **MoveIt Setup Assistant** is a graphical tool that takes your URDF file and generates all the configuration files MoveIt 2 needs. 

**Key Steps in the GUI:**
1.  **Self-Collisions:** The tool calculates which parts of your robot can never hit each other (e.g., adjacent arm links) to save processing power.
2.  **Planning Groups:** This is where you define the humanoid's structure. You create groups like `left_arm`, `right_arm`, and `head`.
3.  **Robot Poses:** You can define named positions like "Home," "Wave," or "Stow."
4.  **Author Information:** Essential for generating the final package.

## 3. The SRDF
The result of the Setup Assistant is the **SRDF (Semantic Robot Description Format)**. While the URDF describes what the robot *looks like*, the SRDF describes what the robot *can do*—it defines the planning groups and the "virtual joints" that connect the robot to the world.

Once configured, you can use the MoveIt 2 Motion Planning plugin in RViz to drag the robot's hand around and watch it plan safe, collision-free paths in real-time.