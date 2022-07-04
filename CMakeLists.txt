cmake_minimum_required(VERSION 3.0.2)
project(study_slam_with_ros)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 3.3)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES study_slam_with_ros
 CATKIN_DEPENDS 
 cv_bridge
 geometry_msgs
 message_generation
 roscpp
 rospy
 sensor_msgs
 std_msgs
 tf
 visualization_msgs
 DEPENDS system_lib
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
	include
  ${OpenCV_LIBRARY_DIRS}
)

add_library(parameter_loader
            src/ParameterLoader.cpp
            include/ParameterLoader.h
            include/opencv_lambda_parallel_for.h
)
target_link_libraries(parameter_loader ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} Eigen3::Eigen)

add_executable(${PROJECT_NAME}_FeatureTracker src/FeatureTracker.cpp)
target_link_libraries(${PROJECT_NAME}_FeatureTracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} parameter_loader Eigen3::Eigen)