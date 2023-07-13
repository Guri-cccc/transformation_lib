cmake_minimum_required(VERSION 3.0.2)
project(fast_tf)

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
    tf2_msgs
    pose_tf
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf geometry_msgs tf2 tf2_ros
 LIBRARIES pose_tf_lib
)

find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Eigen REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
## Declare a C++ library
add_library(${PROJECT_NAME}_lib
  src/fast_tf.cpp
)
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib
 ${catkin_LIBRARIES}
 ${Eigen_LIBRARIES}
)
