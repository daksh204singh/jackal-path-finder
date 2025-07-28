#!/bin/bash

# Get the package installation path
PACKAGE_PATH=$(ros2 pkg prefix jackal_config)

# Set the path to your URDF extras file
export JACKAL_URDF_EXTRAS="${PACKAGE_PATH}/share/jackal_config/urdf/camera_mount.urdf.xacro"

# Optional: Add a check to ensure the file exists
if [ ! -f "$JACKAL_URDF_EXTRAS" ]; then
    echo "Error: URDF extras file not found at $JACKAL_URDF_EXTRAS"
    exit 1
fi

echo "Path: $JACKAL_URDF_EXTRAS"

# Launch Jackal Gazebo world
ros2 launch jackal_gazebo jackal_world.launch.py
