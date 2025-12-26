# Supercharging ROS (Isaac ROS GEMs)

Standard ROS 2 nodes often struggle with high-bandwidth data like 4K video streams or point clouds because they process data on the CPU. **Isaac ROS GEMs** are a collection of GPU-accelerated packages that replace these slow standard nodes.

## 1. The Need for Speed: GPU vs. CPU
A standard computer vision node (like OpenCV) might process 15 frames per second (FPS) on a CPU. By moving that computation to the CUDA cores of an NVIDIA GPU, Isaac ROS can often process the same stream at 60+ FPS with lower latency.

### NITROS: Zero-Copy Memory
The secret sauce behind this speed is **NITROS** (NVIDIA Isaac Transport for ROS).
*   **Standard ROS:** When Node A sends an image to Node B, the data is often copied in memory. This "copy" takes time.
*   **NITROS:** Uses GPU memory directly. Node A places the image in GPU memory, and hands Node B a "pointer" to that location. No actual pixels are moved. This is effectively **zero-copy transfer**, enabling massive throughput.

## 2. The Docker Requirement
Here is the most critical operational constraint for using Isaac ROS:

:::danger Docker is NOT Optional
**Isaac ROS requires Docker to run effectively.**
:::

Because Isaac ROS relies on specific versions of CUDA, TensorRT, and other low-level NVIDIA libraries, installing them directly on your host operating system (bare metal) is a recipe for disaster (dependency hell).

We utilize the `isaac_ros_common` Docker container. This container comes pre-configured with:
*   Ubuntu 22.04 (or compatible base)
*   ROS 2 Humble
*   CUDA Toolkit
*   TensorRT
*   VPI (Vision Programming Interface)

### Practical: Setting up the Container
On your Jetson or Desktop:

1.  **Clone the common repo:**
    ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
    ```
2.  **Configure the workspace:**
    Create a `src` directory inside your project and clone the specific GEMs you need (e.g., `isaac_ros_image_pipeline`).
3.  **Launch the container:**
    ```bash
    cd isaac_ros_common
    ./scripts/run_dev.sh <path_to_your_ros_workspace>
    ```

Once inside this container, you can compile and run your GPU-accelerated nodes just like normal ROS nodes, but with supercharged performance.