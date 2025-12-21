cmake_minimum_required(VERSION 3.8)
project(livox_to_pc2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# Add executable
add_executable(livox_to_pc2_node src/livox_to_pc2_node.cpp)

target_include_directories(livox_to_pc2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(livox_to_pc2_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(livox_to_pc2_node
  rclcpp
  sensor_msgs
  std_msgs
  livox_ros_driver2
  pcl_conversions
)

# Install executable
install(TARGETS livox_to_pc2_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
