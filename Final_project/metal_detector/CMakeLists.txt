cmake_minimum_required(VERSION 3.0.2)
project(metal_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_python_setup()

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(metal_detector_extract nodes/metal_detector_extract.cpp)

target_link_libraries(metal_detector_extract
  ${catkin_LIBRARIES}
)
