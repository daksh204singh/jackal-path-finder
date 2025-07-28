cmake_minimum_required(VERSION 3.8)
project(jackal_config)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install scripts with execution permissions
install(
  PROGRAMS 
    scripts/setup_jackal_env.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Optional: Install URDF files
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
