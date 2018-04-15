cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project("3D_head_position_Dlib")

set(CMAKE_CXX_FLAGS "-std=c++11")


set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)
find_package(freenect2 REQUIRED)

find_package( PCL REQUIRED COMPONENTS common io visualization )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")   # use this in Ubuntu 16.04
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(  dlib REQUIRED )

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )


include_directories("/usr/include/libusb-1.0/" )
include_directories( ${freenect2_INCLUDE_DIR} )
include_directories(  ${dlib_INCLUDE_DIRS} )

include_directories( ${PROJECT_SOURCE_DIR}/include )

#opengl只需要添加.so就行
set(TARGET_LIB
     "/usr/lib/x86_64-linux-gnu/libGL.so"
     "/usr/lib/x86_64-linux-gnu/libGLU.so"
     "/usr/lib/x86_64-linux-gnu/libglut.so"
     )

 set(source src/main.cpp src/grab_image.cpp
     include/grab_image.h                 )

add_executable(head_position_dlib ${source})
target_link_libraries(head_position_dlib ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${dlib_LIBRARIES} )
