cmake_minimum_required(VERSION 3.5)
project(ig_lio)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Release)
add_compile_options(-Wall -Wextra -Wpedantic -O3)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(TBB REQUIRED)

# Optionally, handle any custom find_package modules like 'glog.cmake'
include(cmake/glog.cmake)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/thirdparty/sophus
)

# Declare a C++ executable
add_executable(${PROJECT_NAME}_node
  src/ig_lio_node.cpp
  # src/pointcloud_preprocess.cpp
  # src/lio.cpp
  src/voxel_map.cpp
  src/SymmetricEigenSolver.cpp
  src/timer.cpp
  src/faster_voxel_grid.cpp
)

# Specify libraries to link a library or executable target against
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  pcl_conversions
  TBB
  livox_ros_driver2
)

target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  ${TBB_LIBRARIES}
  glog
  gflags
)

# Install the executable
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install various directory contents
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
