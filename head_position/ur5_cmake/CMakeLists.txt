cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)
add_compile_options(-std=c++11)
project(helloworld)

set(CMAKE_INCLUDE_CURRENT_DIR ON)#自动添加 CMAKE_CURRENT_BINARY_DIR 和CMAKE_CURRENT_SOURCE_DIR 到当前处理的 CMakeLists.txt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(RESOURCE_DIR resources/robot_resource.qrc)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

find_package(Qt5 REQUIRED Widgets Network)
include_directories(bp_nn PD mythread Face_recognition UR5_kin robotState pose_detect force_sensor)

qt5_wrap_cpp( MOC mainwindow.h)

qt5_wrap_ui( UIC mainwindow.ui)
qt5_add_resources(RCC robot_resource.qrc)
set(force_api lib/LINUX_API_V1.5.1/lib)
set(artool lib/ARToolKitPlus-2.3.0/build/lib)

# Kinect2 driver
find_package(freenect2 REQUIRED)
include_directories( ${freenect2_INCLUDE_DIR})
# 3d reconstruction
include_directories(Reconstruct_3d/include)

# PCL Library
#find_package( PCL REQUIRED)
#list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")   # use this in Ubuntu 16.04
#ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
#INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
#LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )
find_package(PCL 1.8 REQUIRED)
get_directory_property(dir_defs DIRECTORY ${CMAKE_SOURCE_DIR} COMPILE_DEFINITIONS)
set(vtk_flags)
foreach(it ${dir_defs})
    if(it MATCHES "vtk*")
    list(APPEND vtk_flags ${it})
    endif()
endforeach()
foreach(d ${vtk_flags})
    remove_definitions(-D${d})
endforeach()
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Dlib Library
find_package(  dlib REQUIRED )
include_directories(  ${dlib_INCLUDE_DIRS} )

#opencv2.4.13
set(OpenCV_DIR /home/zhang/software_install/opencv-2.4.13/build/)
find_package(OpenCV 2.4.13 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#cuda 8.0
find_package(CUDA 8.0 REQUIRED)
include_directories( ${CUDA_INCLUDE_DIRS})

#sensor
LINK_DIRECTORIES(lib/LINUX_API_V1.5.1/lib)
include_directories(lib/LINUX_API_V1.5.1/include/omd)

#artoolkit
LINK_DIRECTORIES(lib/ARToolKitPlus-2.3.0/build/lib)
include_directories(lib/ARToolKitPlus-2.3.0/include/ARToolKitPlus)

LINK_DIRECTORIES(lib/qcustomplot-sharedlib/sharedlib-compilation)
include_directories(lib/qcustomplot-sharedlib)

#eigen
include_directories(lib/eigen-eigen-67e894c6cd8f/Eigen )

include_directories(facexml lib Mydatas omd bp_nn)
include_directories(lib/ARToolKitPlus-2.3.0)




add_executable(brain_ur5 main.cpp mainwindow  mainwindow.ui ${RESOURCE_DIR} tcpip_robot.cpp  bp_nn/bp.cpp
    mythread/mythread.cpp force_sensor/ftusb.cpp PD/force_control.cpp pose_detect/cameraposition.cpp robotState/UR5_state.cpp
    UR5_kin/robotkin.cpp Face_recognition/facemain.cpp Face_recognition/facepreprocess.cpp Face_recognition/facerecognition.cpp
    kinect2_grab/grab_kinect2.cpp         kinect2_grab/grab_kinect2.h       head_track/track_head.cpp     head_track/track_head.h
    Reconstruct_3d/reconstruct_3d.cpp     Reconstruct_3d/reconstruct_3d.h                                      )

target_link_libraries(brain_ur5 Qt5::Widgets Qt5::Network ${OpenCV_LIBS} libOMD.so libARToolKitPlus.so libqcustomplot.so libqcustomplotd.so
                      ${freenect2_LIBRARIES}  ${dlib_LIBRARIES} ${PCL_LIBRARIES}  ${CUDA_LIBRARIES}
                      /home/zhang/桌面/build-kinfu_remake-master-Desktop_Qt_5_6_1_GCC_64bit-Release/kfusion/libkfusion11.so)


