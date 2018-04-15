cmake_minimum_required(VERSION 2.8.3)
project(3D_information)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package(catkin REQUIRED COMPONENTS roscpp rostime std_msgs sensor_msgs message_filters cv_bridge image_transport compressed_image_transport compressed_depth_image_transport kinect2_bridge)


find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED)

catkin_package()


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${kinect2_bridge_INCLUDE_DIRS}
)


add_executable(click_rgb src/click_rgb.cpp)
target_link_libraries(click_rgb
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${kinect2_bridge_LIBRARIES}
)

install(TARGETS click_rgb
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
