cmake_minimum_required(VERSION 2.8.3)
project(facial_feature_points_3d)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS roscpp rostime std_msgs sensor_msgs message_filters cv_bridge image_transport compressed_image_transport compressed_depth_image_transport kinect2_bridge)

find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED)
find_package(dlib REQUIRED )

catkin_package()

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${kinect2_bridge_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include

)

set(source src/main.cpp src/grab_image.cpp   src/points_3d_show.cpp
    include/grab_image.h    include/points_3d_show.h
    )

add_executable(facial_feature_points_3d  ${source})
target_link_libraries(facial_feature_points_3d
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${kinect2_bridge_LIBRARIES}
   ${dlib_LIBRARIES}
)

#install(TARGETS click_rgb
 # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)



