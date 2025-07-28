# Anomaly Detection in a Simulated Multi-Robot System

This project implements a multi-robot system in a simulated environment to perform autonomous mapping and anomaly detection. The system deploys multiple Clearpath Jackal UGVs equipped with sensors to map unknown environments, navigate autonomously, and identify visual anomalies using computer vision techniques. The project leverages **ROS (Robot Operating System)**, **Gazebo simulation**, and **Clearpath Jackal** robots to demonstrate advanced robotics concepts including SLAM, autonomous navigation, and multi-robot coordination.

## Project Overview

The primary goal of this project is to deploy multiple Jackal UGVs in a custom Gazebo world where they can:

- **Independently map the environment** using SLAM (Simultaneous Localization and Mapping) techniques
- **Navigate autonomously** using goal-setting and path planning
- **Identify predefined visual anomalies** (specifically a red sphere) using onboard cameras and computer vision algorithms

The project integrates several key robotics concepts: **SLAM**, **autonomous navigation (NAV2)**, **computer vision**, and **multi-robot deployment** to create a comprehensive demonstration of modern robotics capabilities.

## Key Features

- **Custom Gazebo World:** A two-room environment with various obstacles designed for testing mapping and navigation capabilities
- **Custom Teleoperation Controller (`jackal_keypilot`):** A smooth, "game-style" keyboard controller for precise manual robot movement, developed to improve usability over basic incremental controllers
- **Visual Anomaly Detection:** A dedicated ROS package that uses **OpenCV** to detect red spherical objects in real-time from the robot's camera feed based on color (HSV) and circularity analysis
- **Full Single-Robot Autonomy Stack (ROS2):** Complete configuration for a single Jackal to perform SLAM using **`slam_toolbox`** and navigate autonomously using the **NAV2** stack
- **Multi-Robot Deployment (ROS1):** A functional setup for launching and managing three Jackal robots simultaneously in the same simulation, each running its own perception and navigation pipeline

## Tech Stack

- **Operating System:** Ubuntu 20.04 (Focal Fossa)
- **Virtualization:** Tested on UTM for macOS (or any standard VM software like VirtualBox/VMware)
- **ROS Versions:** **ROS2 Foxy Fitzroy** (for single-robot modules) and **ROS1 Noetic Ninjemys** (for the multi-robot integration)
- **Simulation:** Gazebo 11
- **Core Libraries:** OpenCV, `slam_toolbox`, `nav2_bringup`, `pointcloud_to_laserscan`

## System Setup and Installation

1. **Virtual Machine:** Set up a VM with Ubuntu 20.04 LTS, allocating at least 8 GB RAM and 64 GB storage
2. **ROS2 Foxy Installation:** Install the desktop version:
   ```bash
   sudo apt update
   sudo apt install -y ros-foxy-desktop
   ```
3. **Gazebo11 Installation:** Install Gazebo and ROS integration:
   ```bash
   sudo apt install -y gazebo11 libgazebo11-dev ros-foxy-gazebo-ros-pkgs
   ```
4. **Clearpath and Jackal Packages:** Install Jackal-specific packages:
   ```bash
   sudo apt install -y ros-foxy-jackal-desktop ros-foxy-jackal-simulator ros-foxy-jackal-control
   ```
5. **ROS1 Noetic Installation:** For multi-robot simulation, install ROS1 Noetic following the [official installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu)
6. **Build Tools:** Install `colcon` (for ROS2) and `catkin_make`/`catkin_tools` (for ROS1):
   ```bash
   sudo apt install -y python3-colcon-common-extensions python3-rosdep
   ```
7. **Repository Cloning:** Clone this repository into your ROS workspace's `src` folder:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/daksh204singh/jackal-path-finder.git
   ```
8. **Build the Workspace:** Build the custom packages:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

## Usage and Launching

### Single-Robot Simulation (ROS2 Foxy)

1. **Launch Simulation:** Start the custom Gazebo world with a single Jackal:
   ```bash
   ros2 launch jackal_gazebo jackal_world.launch.py
   ```

2. **Teleoperation:** Run the custom `jackal_keypilot` node for smooth manual control:
   ```bash
   ros2 run jackal_keypilot keypilot
   ```
   Use WASD keys for movement with game-like smooth acceleration and deceleration.

3. **SLAM (Mapping):**
   - Launch `slam_toolbox` to generate a map:
     ```bash
     ros2 launch slam_toolbox online_async_launch.py
     ```
   - The system uses `pointcloud_to_laserscan` to convert 3D Lidar data for SLAM processing
   - Save the generated map:
     ```bash
     ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: 'my_map'"
     ```
   
   **Watch the SLAM process in action:**
   
   https://github.com/daksh204singh/jackal-path-finder/assets/main/videos/SLAM.mp4

4. **Anomaly Detection:**
   - Launch the anomaly detection node:
     ```bash
     ros2 run anomaly_detector anomaly_detector_node
     ```
   - Visualize the output using `rqt_image_view`:
     ```bash
     ros2 run rqt_image_view rqt_image_view
     ```
   - Subscribe to `/anomaly_detection/image` topic to see real-time detection results
   
   **See the anomaly detector identifying the target:**
   
   https://github.com/daksh204singh/jackal-path-finder/assets/main/videos/Anomaly_Detection.mp4

5. **Navigation (NAV2):** Launch the NAV2 stack with the previously saved map:
   ```bash
   ros2 launch jackal_nav jackal_nav.launch.py
   ```
   Use RViz to set navigation goals and visualize the robot's path planning.

### Multi-Robot Simulation (ROS1 Noetic)

1. **The Switch to ROS1:** The multi-robot system was implemented in **ROS1** for stability and to overcome the complexities of namespacing the full navigation stack for multiple robots in **ROS2 Foxy**.

2. **Launch Multi-Robot World:** Start the ROS1-based simulation with three robots:
   ```bash
   roslaunch jackal_multi multi_jackal_world.launch
   ```

3. **Functionality:** In this setup, each robot runs its own instance of localization, trajectory planning, and anomaly detection within its own namespace (e.g., `/robot1`, `/robot2`, `/robot3`).

4. **Shared Map:** The robots operate on a shared, static map that is provided to all of them at launch.

**View the multi-robot system in operation:**

<video width="100%" controls>
  <source src="assets/videos/Individual_Robot_Camera_Feed.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

## Repository Structure

- `/src`: Contains all custom ROS packages
  - `jackal_keypilot`: The smooth teleoperation controller
  - `anomaly_detector`: The vision-based anomaly detection node
  - `jackal_nav`: Navigation configuration and launch files
  - `jackal_config`: Robot configuration files
  - `pointcloud_to_laserscan`: Sensor data conversion utilities
- `/launch`: Contains top-level launch files for running single and multi-robot simulations
- `/worlds`: Contains the custom `.world` file for Gazebo (red_sphere.world)
- `/maps`: Stores the pre-generated map files (`.pgm` and `.yaml`)
- `/config`: Holds parameter files, such as `nav2_params.yaml`
- `/urdf`: Contains the URDF extras for mounting the camera sensor
- `/ros_ws`: ROS1 workspace containing multi-robot packages and Jackal base packages

## Key Challenges and Learnings

- **Time Synchronization:** The critical importance of correctly setting the `use_sim_time` parameter to `true` across all nodes in a Gazebo simulation to ensure stable SLAM and NAV2 performance
- **Data Conversion:** The need to bridge the gap between sensor output (`PointCloud2`) and node input (`LaserScan`) for `slam_toolbox` using the `pointcloud_to_laserscan` package
- **Multi-Robot Complexity:** The significant effort required to correctly namespace all topics, services, and TF frames for a multi-robot setup in ROS2, which led to the pragmatic choice of ROS1 for that portion of the project
