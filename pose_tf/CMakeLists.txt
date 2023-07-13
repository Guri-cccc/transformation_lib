cmake_minimum_required(VERSION 3.0.2)
project(pose_tf)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-W -Wall -O3 -std=c++11 -DNDEBUG")


find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2
    tf2_ros
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES hyundai_local_waypoint_tracker_lib
 CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf geometry_msgs tf2 tf2_ros
#  DEPENDS system_lib
)
find_package(OpenCV REQUIRED QUIET)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
  src/euler2quaternion.cpp
)
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib
 ${catkin_LIBRARIES}
)
