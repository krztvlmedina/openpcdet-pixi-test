cmake_minimum_required(VERSION 3.8)
project(pointcloud_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)

# Bin file publisher node
add_executable(bin_publisher_node src/bin_publisher_node.cpp)
ament_target_dependencies(bin_publisher_node
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
)
target_link_libraries(bin_publisher_node ${PCL_LIBRARIES})
target_include_directories(bin_publisher_node PUBLIC ${PCL_INCLUDE_DIRS})

# Install C++ executables
install(TARGETS
  bin_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/bin_file_utils.py
  scripts/downsample_64_to_16.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
