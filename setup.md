# Clearpath-Jackal Setup on Mac Environment

This guide outlines the steps to set up the Clearpath-Jackal environment on a Mac. In this guide, you will install a virtual machine on your Mac, install Ubuntu 20.04.5 (ARM64) with the desktop environment, set up ROS Foxy, install Clearpath Robotics packages, and configure Gazebo for simulation.

> **Note:** The recommended system requirements for the virtual machine are:
> - **RAM:** 8 GB (Allocate at least 4 GB to the virtual machine)
> - **CPU:** 4 cores (Allocate at least 2 cores to the virtual machine)
> - **Storage:** 64 GB (Allocate at least 32 GB to the virtual machine)

---

## 1. Download and Setup Ubuntu Server

1. Install a virtual machine manager or hypervisor such as **UTM** on your Mac.
2. Download the `ubuntu-20.04.5-live-server-arm64` image.
3. Install Ubuntu on your Mac using the virtual machine manager.
4. After installation, ensure you have the Ubuntu Desktop environment installed.

---

## 2. Set Up ROS Foxy on Ubuntu Desktop

### a. Update Packages and Install Dependencies
Run the following command to update your package lists and install required tools:

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
```

### b. Add ROS2 Repository Key and Source List

Add the ROS2 GPG key and repository:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list
```

### c. Install ROS Foxy Desktop

Update the package list and install the desktop version of ROS Foxy:

```bash
sudo apt update
sudo apt install -y ros-foxy-desktop
```

### d. Install Python3 Extensions for Colcon and Rosdep

Install necessary Python packages:

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

### e. Source ROS Setup

Append the ROS setup to your `.bashrc` and source it:

```bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. Set Up Clearpath Robotics Packages

### a. Add Clearpath Robotics Repository Key and Source List

Run the following commands to add Clearpath’s package repository:

```bash
wget -qO - https://packages.clearpathrobotics.com/public.key | sudo apt-key add -
echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/clearpath.list
```

### b. Install Jackal Packages

Update the package list and install the Jackal packages:

```bash
sudo apt update
sudo apt install -y ros-foxy-jackal-desktop ros-foxy-jackal-simulator ros-foxy-jackal-control
```

### c. Install Gazebo and ROS Gazebo Packages

Install Gazebo 11 and its ROS integration:

```bash
sudo apt update
sudo apt install -y gazebo11 libgazebo11-dev ros-foxy-gazebo-ros-pkgs
```

### d. Source ROS Setup Again (if necessary)

Ensure the ROS environment is loaded:

```bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 4. Launch the Jackal Simulation

Run the following command to launch the Jackal world in Gazebo:

```bash
ros2 launch jackal_gazebo jackal_world.launch.py
```