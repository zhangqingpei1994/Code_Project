cmake_minimum_required(VERSION 2.8.3)
project(cv_bridge_kinect2)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET( CMAKE_BUILD_TYPE Debug  )

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include )

LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)
ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/src )

