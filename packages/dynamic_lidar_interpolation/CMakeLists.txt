cmake_minimum_required(VERSION 3.8)
project(dynamic_lidar_interpolation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

# Find Python and pybind11 for Python bindings
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})


# Add executable
add_executable(pointcloud_interpolation_node
  src/pointcloud_interpolation_node.cpp
)

 ament_target_dependencies(pointcloud_interpolation_node
  rclcpp
  sensor_msgs
  PCL
  pcl_conversions
  Eigen3
)

target_link_libraries(pointcloud_interpolation_node
  ${PCL_LIBRARIES}
)

# Build Python bindings if pybind11 is available
if(pybind11_FOUND)
  message(STATUS "pybind11 found - building Python bindings")

  pybind11_add_module(interpolation_core
    src/interpolation_bindings.cpp
  )

  target_include_directories(interpolation_core PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
  )

  target_link_libraries(interpolation_core PRIVATE
    ${PCL_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
  )

  # Install the Python module to the package's Python directory
  install(TARGETS interpolation_core
    DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages
  )
else()
  message(WARNING "pybind11 not found - Python bindings will not be built. Install with: pip install pybind11")
endif()

# Install executables
install(TARGETS
  pointcloud_interpolation_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration and launch files
install(DIRECTORY
  config
  launch
  rviz
  foxglove
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts for .bin file processing
install(PROGRAMS
  scripts/bin_file_utils.py
  scripts/interpolate_bin_file.py
  scripts/interpolate_bin_simple.py
  scripts/interpolate_with_config.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install scripts README
install(FILES
  scripts/README.md
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
