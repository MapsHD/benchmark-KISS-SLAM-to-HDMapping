cmake_minimum_required(VERSION 3.8)
project(kiss-slam-to-hdmapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# Build bundled LASzip
option(BUILD_WITH_BUNDLED_LIBLASZIP "Build with bundled LASzip library" ON)

if(BUILD_WITH_BUNDLED_LIBLASZIP)
  set(LASZIP_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/3rdparty/LASzip)

  # Add LASzip as subdirectory
  add_subdirectory(${LASZIP_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/LASzip)

  set(LASZIP_INCLUDE_DIRS ${LASZIP_SOURCE_DIR}/include)
  set(LASZIP_LIBRARIES laszip)
else()
  find_package(LASzip REQUIRED)
endif()

# Add executable
add_executable(listener
  src/kiss_slam_subscriber.cpp
  src/laz_writer.cpp
)

target_include_directories(listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${LASZIP_INCLUDE_DIRS}
)

target_link_libraries(listener
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  ${LASZIP_LIBRARIES}
)

ament_target_dependencies(listener
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  rosbag2_cpp
)

# Install executable
install(TARGETS listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
